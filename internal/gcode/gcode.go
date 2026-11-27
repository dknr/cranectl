// Package gcode implements a minimal G-code interpreter for gimbal motion.
//
// Coordinate model (degrees): X=pan/yaw, Y=tilt/pitch, Z=roll.
//
// Two frames are involved:
//   - firmware/machine frame: what the *_ROTATE_ANGLE writes and *_ANGLE_R
//     reads use. Fixed to the gimbal's internal datum.
//   - user frame: absolute G1 targets are relative to a settable origin.
//     Homing (G28 → FUNCTION_EVENT BACK_CENTER) drives the gimbal to its
//     physical home and defines the origin, so after G28 "X0 Y0" IS home.
//
// Parsing is a pure lexing step (Lexer). Coordinate resolution happens later,
// sequentially, in the Controller — so a move's coordinates reflect the origin
// in effect at execution time (e.g. right after a G28).
package gcode

import (
	"errors"
	"fmt"
	"math"
	"regexp"
	"strconv"
	"strings"
)

// Kind classifies a lexed action / plan step.
type Kind int

const (
	KindIgnore Kind = iota
	KindMotion      // G0/G1 (or bare X/Y/Z); G53 sets Machine
	KindDwell       // G4 P<ms>
	KindHome        // G28 (gimbal home / BACK_CENTER)
	KindSetPos      // G92
	KindAbs         // G90/G91 (modal)
	KindEnd         // M2/M30
	KindRecord      // M3/M5 (record on/off) or M240 (photo); see RecOp
)

// RecOp selects the camera action for a KindRecord plan.
type RecOp int

const (
	RecNone  RecOp = iota
	RecOn          // M3  — start recording
	RecOff         // M5  — stop recording
	RecPhoto       // M240 — take a still photo
)

// Action is one lexed line, before coordinate resolution.
type Action struct {
	Kind             Kind
	Err              error
	X, Y, Z          float64
	HasX, HasY, HasZ bool
	Rapid            bool    // G0
	Machine          bool    // G53 (raw firmware coords)
	Absolute         bool    // for KindAbs: true=G90 false=G91
	Feed             float64 // F value (0 = none)
	TimeMS           int
	HaveTime         bool
	DwellMS          int
	RecOp            RecOp
}

// Plan is a resolved action ready to execute.
type Plan struct {
	Kind             Kind
	Err              error
	X, Y, Z          float64
	HasX, HasY, HasZ bool
	MovedAny         bool
	TimeMS           int
	DwellMS          int
	RecOp            RecOp
}

// Limits are per-axis ±degrees (firmware frame). Moves beyond these are
// rejected, never silently wrapped.
type Limits struct{ Pan, Tilt, Roll float64 }

var (
	wordRe  = regexp.MustCompile(`(?i)([A-Za-z])([-+]?[0-9]*\.?[0-9]+)`)
	blockRe = regexp.MustCompile(`\([^)]*\)`)
)

// Lexer turns lines into Actions. Stateless; safe to keep in one goroutine.
type Lexer struct{}

// Parse lexes a single line (stripping ';' and '(...)' comments).
func (Lexer) Parse(line string) Action {
	s := sanitize(line)
	if s == "" {
		return Action{Kind: KindIgnore}
	}
	words := wordRe.FindAllStringSubmatch(s, -1)
	if len(words) == 0 {
		return Action{Kind: KindIgnore, Err: errors.New("unparseable line")}
	}

	var (
		a      Action
		gs, ms []int
		f      float64
		t      int
		haveT  bool
		p_     int
	)
	for _, w := range words {
		letter := strings.ToUpper(w[1])
		val, err := strconv.ParseFloat(w[2], 64)
		if err != nil {
			return Action{Err: fmt.Errorf("bad value %q", w[0])}
		}
		switch letter {
		case "G":
			gs = append(gs, int(val))
		case "M":
			ms = append(ms, int(val))
		case "X":
			a.X, a.HasX = val, true
		case "Y":
			a.Y, a.HasY = val, true
		case "Z":
			a.Z, a.HasZ = val, true
		case "F":
			f = val
		case "T":
			t, haveT = int(val), true
		case "P":
			p_ = int(val)
		case "N", "S":
			// ignore
		default:
			return Action{Err: fmt.Errorf("unknown word %q", w[0])}
		}
	}
	a.Feed, a.TimeMS, a.HaveTime = f, t, haveT

	for _, m := range ms {
		switch m {
		case 2, 30:
			a.Kind = KindEnd
			return a
		case 3:
			a.Kind, a.RecOp = KindRecord, RecOn
			return a
		case 5:
			a.Kind, a.RecOp = KindRecord, RecOff
			return a
		case 240:
			a.Kind, a.RecOp = KindRecord, RecPhoto
			return a
		default:
			return Action{Err: fmt.Errorf("unsupported M%d", m)}
		}
	}

	// Mode / control G-codes (mutually exclusive with motion in v1).
	for _, g := range gs {
		switch g {
		case 90:
			return Action{Kind: KindAbs, Absolute: true}
		case 91:
			return Action{Kind: KindAbs, Absolute: false}
		case 20, 21:
			// units: accepted+ignored
		case 92:
			a.Kind = KindSetPos
			return a
		case 4:
			if p_ <= 0 {
				return Action{Err: errors.New("G4 requires P<ms>")}
			}
			return Action{Kind: KindDwell, DwellMS: p_}
		case 28:
			return Action{Kind: KindHome}
		case 53:
			a.Kind, a.Machine = KindMotion, true
		case 0:
			a.Kind, a.Rapid = KindMotion, true
		case 1:
			a.Kind = KindMotion
		default:
			return Action{Err: fmt.Errorf("unsupported G%d", g)}
		}
	}

	if !a.HasX && !a.HasY && !a.HasZ {
		return Action{Kind: KindIgnore}
	}
	if a.Kind == 0 { // bare coordinates → linear move
		a.Kind = KindMotion
	}
	return a
}

// Controller resolves actions to plans using the current modal state and
// origin. Not safe for concurrent use; the daemon's single executor owns it.
type Controller struct {
	lim                       Limits
	defTime, minTime, maxTime int
	abs                       bool
	feed                      float64
	recording                 bool    // desired record state (M3/M5), see CameraRecord toggle caveat
	ox, oy, oz                float64 // origin in firmware coords
	px, py, pz                float64 // current position in user coords
}

// NewController returns a controller defaulting to absolute, origin at the
// firmware datum (so pre-home, user frame == firmware frame).
func NewController(lim Limits, defTime, minTime, maxTime int) *Controller {
	return &Controller{lim: lim, defTime: defTime, minTime: minTime, maxTime: maxTime, abs: true}
}

// SetOriginFromMeasured adopts a measured firmware pose as the origin/user-zero
// (called by the daemon after a G28 homing settles).
func (c *Controller) SetOriginFromMeasured(yaw, pitch, roll float64) {
	c.ox, c.oy, c.oz = yaw, pitch, roll
	c.px, c.py, c.pz = 0, 0, 0
}

// Position returns the current user-frame position.
func (c *Controller) Position() (x, y, z float64) { return c.px, c.py, c.pz }

// Origin returns the current origin in firmware coords.
func (c *Controller) Origin() (x, y, z float64) { return c.ox, c.oy, c.oz }

// SetFeed seeds the modal feedrate (deg/min).
func (c *Controller) SetFeed(f float64) { c.feed = f }

// Recording reports the tracked desired record state.
func (c *Controller) Recording() bool { return c.recording }

// SetRecording overrides the tracked record state (e.g. to reconcile with a
// status flag once the definitive "recording?" signal is known).
func (c *Controller) SetRecording(v bool) { c.recording = v }

// Plan resolves an action, updating modal state (abs/feed) as it goes.
func (c *Controller) Plan(a Action) Plan {
	switch a.Kind {
	case KindIgnore:
		return Plan{Kind: KindIgnore, Err: a.Err}
	case KindEnd:
		return Plan{Kind: KindEnd}
	case KindDwell:
		return Plan{Kind: KindDwell, DwellMS: a.DwellMS}
	case KindAbs:
		c.abs = a.Absolute
		return Plan{Kind: KindIgnore}
	case KindHome:
		return Plan{Kind: KindHome}
	case KindRecord:
		return c.planRecord(a)
	case KindSetPos:
		return c.planSetPos(a)
	case KindMotion:
		return c.planMotion(a)
	}
	if a.Err != nil {
		return Plan{Kind: KindIgnore, Err: a.Err}
	}
	return Plan{Kind: KindIgnore}
}

func (c *Controller) planRecord(a Action) Plan {
	// M3/M5 are set-semantics over a hardware toggle: only emit the key on a
	// transition, so they are idempotent. M240 (photo) always fires.
	switch a.RecOp {
	case RecOn:
		if c.recording {
			return Plan{Kind: KindIgnore}
		}
		c.recording = true
		return Plan{Kind: KindRecord, RecOp: RecOn}
	case RecOff:
		if !c.recording {
			return Plan{Kind: KindIgnore}
		}
		c.recording = false
		return Plan{Kind: KindRecord, RecOp: RecOff}
	case RecPhoto:
		return Plan{Kind: KindRecord, RecOp: RecPhoto}
	}
	return Plan{Kind: KindIgnore, Err: errors.New("record: no op")}
}

func (c *Controller) planSetPos(a Action) Plan {
	// Redefine current position to (a.X,a.Y,a.Z); shift origin so the physical
	// point is unchanged.
	if a.HasX {
		fwNow := c.ox + c.px
		c.ox = fwNow - a.X
		c.px = a.X
	}
	if a.HasY {
		fwNow := c.oy + c.py
		c.oy = fwNow - a.Y
		c.py = a.Y
	}
	if a.HasZ {
		fwNow := c.oz + c.pz
		c.oz = fwNow - a.Z
		c.pz = a.Z
	}
	return Plan{Kind: KindIgnore}
}

func (c *Controller) planMotion(a Action) Plan {
	if a.Feed > 0 {
		c.feed = a.Feed
	}
	fx, fy, fz := c.ox+c.px, c.oy+c.py, c.oz+c.pz
	nx, ny, nz := c.px, c.py, c.pz

	set := func(g float64, has, machine bool, origin, pos, lim float64) (fw, newUser float64, err error) {
		if !has {
			return origin + pos, pos, nil
		}
		if machine {
			fw = g
			newUser = g - origin
		} else {
			if c.abs {
				newUser = g
			} else {
				newUser = pos + g
			}
			fw = origin + newUser
		}
		if math.Abs(fw) > lim {
			return 0, 0, fmt.Errorf("axis %.2f out of range ±%.1f", fw, lim)
		}
		return fw, newUser, nil
	}

	var err error
	if fx, nx, err = set(a.X, a.HasX, a.Machine, c.ox, c.px, c.lim.Pan); err != nil {
		return Plan{Kind: KindIgnore, Err: err}
	}
	if fy, ny, err = set(a.Y, a.HasY, a.Machine, c.oy, c.py, c.lim.Tilt); err != nil {
		return Plan{Kind: KindIgnore, Err: err}
	}
	if fz, nz, err = set(a.Z, a.HasZ, a.Machine, c.oz, c.pz, c.lim.Roll); err != nil {
		return Plan{Kind: KindIgnore, Err: err}
	}

	// Motion time.
	tm := c.timeFor(a, nx-c.px, ny-c.py, nz-c.pz)

	// Commit user position for moved axes (machine moves recompute from fw).
	if a.HasX {
		c.px = nx
	}
	if a.HasY {
		c.py = ny
	}
	if a.HasZ {
		c.pz = nz
	}
	return Plan{
		Kind: KindMotion,
		X:    fx, Y: fy, Z: fz,
		HasX: a.HasX, HasY: a.HasY, HasZ: a.HasZ,
		MovedAny: a.HasX || a.HasY || a.HasZ,
		TimeMS:   tm,
	}
}

func (c *Controller) timeFor(a Action, dx, dy, dz float64) int {
	var tm int
	switch {
	case a.Rapid:
		tm = c.minTime
	case a.HaveTime:
		tm = a.TimeMS
	case c.feed > 0:
		dist := math.Abs(dx)
		if d := math.Abs(dy); d > dist {
			dist = d
		}
		if d := math.Abs(dz); d > dist {
			dist = d
		}
		if dist < 0.001 {
			tm = c.minTime
		} else {
			tm = int(60000.0 * dist / c.feed)
		}
	default:
		tm = c.defTime
	}
	if tm < c.minTime {
		tm = c.minTime
	}
	if tm > c.maxTime {
		tm = c.maxTime
	}
	return tm
}

// sanitize strips ';' line comments and '(...)' block comments.
func sanitize(line string) string {
	if i := strings.IndexByte(line, ';'); i >= 0 {
		line = line[:i]
	}
	line = blockRe.ReplaceAllString(line, " ")
	return strings.TrimSpace(line)
}
