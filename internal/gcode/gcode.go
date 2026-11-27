// Package gcode implements a minimal G-code interpreter for gimbal motion.
//
// Coordinate model (degrees): X=pan/yaw, Y=tilt/pitch, Z=roll.
// Supported: G0/G1 motion (F feedrate deg/min or T ms), G4 P dwell, G28 home,
// G90/G91 abs/relative, G92 set-datum, G20/G21 no-op, M2/M30 program end,
// comments (; and ()), line numbers (N). Anything else is reported as an
// error rather than silently ignored, so typos surface instead of being
// dropped.
package gcode

import (
	"errors"
	"fmt"
	"math"
	"regexp"
	"strconv"
	"strings"
)

// Kind classifies a parsed line.
type Kind int

const (
	KindIgnore     Kind = iota // comment / blank
	KindMove                   // G0/G1 (or bare X/Y/Z)
	KindDwell                  // G4 P<ms>
	KindHome                   // G28
	KindSetDatum               // G92
	KindProgramEnd             // M2/M30
	KindError                  // rejected
)

// Limits are the per-axis ±degrees the gimbal accepts. Moves beyond these are
// rejected (never silently wrapped).
type Limits struct {
	Pan, Tilt, Roll float64
}

// Command is one parsed line's result.
type Command struct {
	Kind Kind
	Err  error

	// motion (absolute degrees, fully resolved)
	X, Y, Z  float64
	HasX     bool
	HasY     bool
	HasZ     bool
	Rapid    bool
	TimeMS   int
	MovedAny bool

	// dwell
	DwellMS int
}

var (
	wordRe  = regexp.MustCompile(`(?i)([A-Za-z])([-+]?[0-9]*\.?[0-9]+)`)
	blockRe = regexp.MustCompile(`\([^)]*\)`)
)

// Parser holds modal state across lines. It is NOT safe for concurrent use;
// the daemon runs a single reader goroutine over it.
type Parser struct {
	abs              bool
	feed             float64 // deg/min, modal
	defTime          int     // ms used when no F/T given
	minTime          int     // ms floor
	maxTime          int     // ms ceiling
	curX, curY, curZ float64
	lim              Limits
}

// NewParser returns a parser defaulting to absolute positioning.
func NewParser(lim Limits, defTime, minTime, maxTime int) *Parser {
	return &Parser{abs: true, defTime: defTime, minTime: minTime, maxTime: maxTime, lim: lim}
}

// SetPosition overrides the datum without moving (G92).
func (p *Parser) SetPosition(x, y, z float64) { p.curX, p.curY, p.curZ = x, y, z }

// Position returns the last commanded absolute pose.
func (p *Parser) Position() (x, y, z float64) { return p.curX, p.curY, p.curZ }

// Parse sanitizes comments and returns the Command for a single line.
func (p *Parser) Parse(line string) Command {
	s := sanitize(line)
	if s == "" {
		return Command{Kind: KindIgnore}
	}
	words := wordRe.FindAllStringSubmatch(s, -1)
	if len(words) == 0 {
		return Command{Kind: KindError, Err: errors.New("unparseable line")}
	}

	var (
		gs, ms           []int
		x, y, z          float64
		hasX, hasY, hasZ bool
		f                float64
		t                int
		p_               int // G4 P
		haveT            bool
	)
	for _, w := range words {
		letter := strings.ToUpper(w[1])
		val, err := strconv.ParseFloat(w[2], 64)
		if err != nil {
			return Command{Kind: KindError, Err: fmt.Errorf("bad value %q", w[0])}
		}
		switch letter {
		case "G":
			gs = append(gs, int(val))
		case "M":
			ms = append(ms, int(val))
		case "X":
			x, hasX = val, true
		case "Y":
			y, hasY = val, true
		case "Z":
			z, hasZ = val, true
		case "F":
			f = val
		case "T":
			t, haveT = int(val), true
		case "P":
			p_ = int(val)
		case "N", "S":
			// line number / spindle speed: ignore
		default:
			return Command{Kind: KindError, Err: fmt.Errorf("unknown word %q", w[0])}
		}
	}

	// Program / mode controls first.
	for _, m := range ms {
		switch m {
		case 2, 30:
			return Command{Kind: KindProgramEnd}
		default:
			return Command{Kind: KindError, Err: fmt.Errorf("unsupported M%d", m)}
		}
	}
	for _, g := range gs {
		switch g {
		case 90:
			p.abs = true
		case 91:
			p.abs = false
		case 20, 21:
			// units: degrees regardless; accept+ignore
		case 92:
			if hasX {
				p.curX = x
			}
			if hasY {
				p.curY = y
			}
			if hasZ {
				p.curZ = z
			}
			return Command{Kind: KindSetDatum}
		case 4:
			if p_ <= 0 {
				return Command{Kind: KindError, Err: errors.New("G4 requires P<ms>")}
			}
			return Command{Kind: KindDwell, DwellMS: p_}
		case 28:
			return p.move(0, 0, 0, true, true, false, false, f, t, haveT)
		case 0, 1:
			// motion handled below (coordinates required)
		default:
			return Command{Kind: KindError, Err: fmt.Errorf("unsupported G%d", g)}
		}
	}

	if !hasX && !hasY && !hasZ {
		return Command{Kind: KindIgnore}
	}
	// Bare coordinates or a G0/G1: compute absolute targets + time.
	tx, ty, tz := x, y, z
	if !p.abs {
		if hasX {
			tx += p.curX
		}
		if hasY {
			ty += p.curY
		}
		if hasZ {
			tz += p.curZ
		}
	}
	rapid := len(gs) > 0 && gs[0] == 0
	return p.move(tx, ty, tz, hasX, hasY, hasZ, rapid, f, t, haveT)
}

// move validates axes, computes motion time, and updates the datum.
func (p *Parser) move(tx, ty, tz float64, hx, hy, hz, rapid bool, f float64, t int, haveT bool) Command {
	if hx && math.Abs(tx) > p.lim.Pan {
		return Command{Kind: KindError, Err: fmt.Errorf("pan %.2f out of range ±%.1f", tx, p.lim.Pan)}
	}
	if hy && math.Abs(ty) > p.lim.Tilt {
		return Command{Kind: KindError, Err: fmt.Errorf("tilt %.2f out of range ±%.1f", ty, p.lim.Tilt)}
	}
	if hz && math.Abs(tz) > p.lim.Roll {
		return Command{Kind: KindError, Err: fmt.Errorf("roll %.2f out of range ±%.1f", tz, p.lim.Roll)}
	}

	// Feed modal: an explicit F updates it.
	if f > 0 {
		p.feed = f
	}

	// Motion time: rapid uses the floor; explicit T wins; else derive from
	// feed + travel distance; else default.
	var tm int
	switch {
	case rapid:
		tm = p.minTime
	case haveT:
		tm = t
	case p.feed > 0:
		dist := math.Abs(tx - p.curX)
		if d := math.Abs(ty - p.curY); d > dist {
			dist = d
		}
		if d := math.Abs(tz - p.curZ); d > dist {
			dist = d
		}
		if dist < 0.001 {
			tm = p.minTime
		} else {
			tm = int(60000.0 * dist / p.feed)
		}
	default:
		tm = p.defTime
	}
	if tm < p.minTime {
		tm = p.minTime
	}
	if tm > p.maxTime {
		tm = p.maxTime
	}

	// Commit datum.
	if hx {
		p.curX = tx
	}
	if hy {
		p.curY = ty
	}
	if hz {
		p.curZ = tz
	}
	return Command{
		Kind: KindMove,
		X:    p.curX, Y: p.curY, Z: p.curZ,
		HasX: hx, HasY: hy, HasZ: hz,
		Rapid:    rapid,
		TimeMS:   tm,
		MovedAny: hx || hy || hz,
	}
}

// sanitize strips ';' line comments and '(...)' block comments.
func sanitize(line string) string {
	if i := strings.IndexByte(line, ';'); i >= 0 {
		line = line[:i]
	}
	line = blockRe.ReplaceAllString(line, " ")
	return strings.TrimSpace(line)
}
