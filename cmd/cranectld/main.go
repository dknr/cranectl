// cranectld — long-running G-code daemon for a CRANEM2S gimbal.
//
// It owns a single persistent BLE connection and drains G-code lines from a
// named pipe (FIFO), mapping each motion block to the closed-loop move
// sequence. GRBL-style real-time commands are handled immediately:
//
//	?       report state + measured position
//	!       feed hold (stop dequeue)
//	~       resume from hold
//	^X      soft reset (drain queue, clear hold/fault)
//
// Responses (ok / error:<n> / status) go to stdout (redirect to a log).
//
// Usage:
//
//	cranectld <MAC> --pipe /run/user/$UID/cranectld.fifo [flags]
package main

import (
	"bufio"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"regexp"
	"strings"
	"sync/atomic"
	"syscall"
	"time"

	"cranectl/internal/gcode"
	"cranectl/internal/gimbal"
)

type state struct {
	hold    int32
	fault   int32
	resetRe int32
}

func main() {
	var (
		pipe    = flag.String("pipe", defaultPipePath(), "named pipe to read G-code from")
		pidfile = flag.String("pidfile", "", "write daemon PID here")
		defTime = flag.Int("def-time", 1500, "default motion time (ms) when no F/T given")
		minTime = flag.Int("min-time", 60, "minimum motion time (ms)")
		maxTime = flag.Int("max-time", 60000, "maximum motion time (ms)")
		panLim  = flag.Float64("max-pan", 180, "pan (yaw) axis limit ±deg")
		tiltLim = flag.Float64("max-tilt", 90, "tilt (pitch) axis limit ±deg")
		rollLim = flag.Float64("max-roll", 45, "roll axis limit ±deg")
		feedDef = flag.Float64("feed", 0, "initial feedrate (deg/min); 0 = use --def-time")
	)
	flag.Usage = func() {
		fmt.Fprintln(os.Stderr, "usage: cranectld <MAC> [--pipe PATH] [flags]  (see --help)")
		flag.PrintDefaults()
	}
	// MAC-first: pull the MAC out before flag parsing (stdlib flag stops at the
	// first positional), then parse flags from the remainder.
	mac, rest := extractMAC(os.Args[1:])
	flag.CommandLine.Parse(rest)
	if mac == "" || flag.NArg() != 0 {
		flag.Usage()
		os.Exit(2)
	}

	// Connect once and keep the link alive.
	dev, err := gimbal.Connect(mac)
	if err != nil {
		logf("fatal: connect %s: %v", mac, err)
		os.Exit(1)
	}
	defer dev.Close()

	lim := gcode.Limits{Pan: *panLim, Tilt: *tiltLim, Roll: *rollLim}
	parser := gcode.NewParser(lim, *defTime, *minTime, *maxTime)
	if *feedDef > 0 {
		parser.Parse(fmt.Sprintf("G1 X0 F%.4g", *feedDef)) // seed modal feed without moving
	}

	pipeFile, err := openFIFO(*pipe)
	if err != nil {
		logf("fatal: fifo %s: %v", *pipe, err)
		os.Exit(1)
	}
	defer pipeFile.Close()

	if *pidfile != "" {
		if err := os.WriteFile(*pidfile, []byte(fmt.Sprint(os.Getpid())), 0o644); err != nil {
			logf("warn: pidfile: %v", err)
		}
		defer os.Remove(*pidfile)
	}

	var st state
	cmdCh := make(chan gcode.Command, 10000)

	// SIGINT/SIGTERM -> clean shutdown (every executing move disarms on its
	// own; the executor's deferred cleanup closes the link).
	sigs := make(chan os.Signal, 1)
	signal.Notify(sigs, syscall.SIGINT, syscall.SIGTERM)
	go func() {
		<-sigs
		logf("shutdown: signal received")
		dev.Close()
		if *pidfile != "" {
			os.Remove(*pidfile)
		}
		os.Exit(0)
	}()

	go executor(dev, &st, cmdCh)
	reader(pipeFile, dev, parser, &st, cmdCh, mac, lim)
}

// executor owns the connection and runs queued commands, single-threaded.
func executor(dev *gimbal.Device, st *state, ch <-chan gcode.Command) {
	var pending *gcode.Command
	held := func() bool {
		return atomic.LoadInt32(&st.hold) == 1 || atomic.LoadInt32(&st.fault) == 1
	}
	for {
		if atomic.SwapInt32(&st.resetRe, 0) == 1 {
			drain(ch)
			pending = nil
			atomic.StoreInt32(&st.fault, 0)
			atomic.StoreInt32(&st.hold, 0)
			logf("reset")
			continue
		}
		if held() {
			time.Sleep(50 * time.Millisecond)
			continue
		}
		if pending == nil {
			select {
			case c := <-ch:
				cc := c
				pending = &cc
			case <-time.After(50 * time.Millisecond):
				continue
			}
		}
		// A reset or hold may have arrived while we waited to acquire pending.
		if atomic.SwapInt32(&st.resetRe, 0) == 1 {
			drain(ch)
			pending = nil
			atomic.StoreInt32(&st.fault, 0)
			atomic.StoreInt32(&st.hold, 0)
			logf("reset")
			continue
		}
		if held() {
			continue // keep pending, wait for resume/reset
		}
		run(dev, st, *pending)
		pending = nil
	}
}

func run(dev *gimbal.Device, st *state, cmd gcode.Command) {
	switch cmd.Kind {
	case gcode.KindMove:
		if !cmd.MovedAny {
			logf("ok")
			return
		}
		mp := gcodeToParams(cmd)
		if err := dev.Move(mp); err != nil {
			logf("error: %v", err)
			atomic.StoreInt32(&st.fault, 1)
			atomic.StoreInt32(&st.hold, 1)
			return
		}
		logf("ok")
	case gcode.KindDwell:
		time.Sleep(time.Duration(cmd.DwellMS) * time.Millisecond)
		logf("ok")
	case gcode.KindProgramEnd:
		logf("ok (program end)")
	case gcode.KindSetDatum, gcode.KindIgnore:
		// datum applied at parse time; nothing to do.
	case gcode.KindError:
		logf("error: %v", cmd.Err)
		atomic.StoreInt32(&st.fault, 1)
		atomic.StoreInt32(&st.hold, 1)
	}
}

func gcodeToParams(cmd gcode.Command) gimbal.MoveParams {
	mp := gimbal.MoveParams{Time: uint32(cmd.TimeMS)}
	if cmd.HasX {
		v := cmd.X
		mp.Yaw = &v
	}
	if cmd.HasY {
		v := cmd.Y
		mp.Pitch = &v
	}
	if cmd.HasZ {
		v := cmd.Z
		mp.Roll = &v
	}
	return mp
}

// reader parses lines into commands and handles real-time commands inline.
func reader(f *os.File, dev *gimbal.Device, parser *gcode.Parser, st *state, ch chan<- gcode.Command, mac string, lim gcode.Limits) {
	logf("ready: mac=%s pan=±%.0f tilt=±%.0f roll=±%.0f", mac, lim.Pan, lim.Tilt, lim.Roll)
	sc := bufio.NewScanner(f)
	sc.Buffer(make([]byte, 0, 4096), 64*1024)
	for sc.Scan() {
		line := sc.Text()
		if realtime(strings.TrimSpace(line), dev, parser, st) {
			continue
		}
		ch <- parser.Parse(line)
	}
	if err := sc.Err(); err != nil {
		logf("fifo read error: %v", err)
	}
}

// realtime handles immediate commands; returns true if the line was one.
func realtime(t string, dev *gimbal.Device, parser *gcode.Parser, st *state) bool {
	switch {
	case t == "?" || strings.EqualFold(t, "STATUS"):
		x, y, z := parser.Position()
		if ax, ay, az, err := dev.Angles(); err == nil {
			x, y, z = ax, ay, az
		}
		logf("<%s,MPos:%.2f,%.2f,%.2f>", stateName(st), x, y, z)
		return true
	case t == "!":
		atomic.StoreInt32(&st.hold, 1)
		logf("hold")
		return true
	case t == "~":
		atomic.StoreInt32(&st.hold, 0)
		logf("resume")
		return true
	case t == string(rune(0x18)) || strings.EqualFold(t, "RESET") || strings.EqualFold(t, "^X"):
		atomic.StoreInt32(&st.resetRe, 1)
		return true
	}
	return false
}

func stateName(st *state) string {
	switch {
	case atomic.LoadInt32(&st.fault) == 1:
		return "Alarm"
	case atomic.LoadInt32(&st.hold) == 1:
		return "Hold"
	default:
		return "Idle"
	}
}

func drain(ch <-chan gcode.Command) {
	for {
		select {
		case <-ch:
		default:
			return
		}
	}
}

// openFIFO creates the named pipe if needed and opens it read-write so reads
// never see EOF when writers open/close between commands.
func openFIFO(path string) (*os.File, error) {
	if st, err := os.Stat(path); err != nil {
		if err := syscall.Mkfifo(path, 0o644); err != nil {
			return nil, err
		}
	} else if st.Mode()&os.ModeNamedPipe == 0 {
		return nil, fmt.Errorf("%s exists and is not a FIFO", path)
	}
	return os.OpenFile(path, os.O_RDWR, 0o644)
}

var macRe = regexp.MustCompile(`^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$`)

// extractMAC removes and returns the first MAC-looking token, keeping the
// remaining args in order for flag parsing.
func extractMAC(args []string) (string, []string) {
	mac := ""
	out := make([]string, 0, len(args))
	for _, a := range args {
		if mac == "" && macRe.MatchString(a) {
			mac = a
			continue
		}
		out = append(out, a)
	}
	return mac, out
}

func defaultPipePath() string {
	if d := os.Getenv("XDG_RUNTIME_DIR"); d != "" {
		return d + "/cranectld.fifo"
	}
	return "/tmp/cranectld.fifo"
}

func logf(f string, a ...any) {
	fmt.Printf("%s %s\n", time.Now().Format("15:04:05.000"), fmt.Sprintf(f, a...))
}
