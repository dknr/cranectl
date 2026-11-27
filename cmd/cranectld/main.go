// cranectld — long-running G-code daemon for a CRANEM2S gimbal.
//
// It owns a single persistent BLE connection and drains G-code lines from a
// named pipe (FIFO). Coordinate resolution (origin, feedrate, abs/relative)
// happens sequentially in the executor, so a move reflects the coordinate
// state in effect at execution time. GRBL-style real-time commands are handled
// immediately:
//
//	?       report state + user and measured position
//	!       feed hold (stop dequeue)
//	~       resume from hold
//	^X/RESET soft reset (drain queue, clear hold/fault)
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
	"sync"
	"sync/atomic"
	"syscall"
	"time"

	"cranectl/internal/gcode"
	"cranectl/internal/gimbal"
)

// homeSettle is how long to wait for the gimbal to reach its physical home
// (BACK_CENTER) before reading the pose and adopting it as the origin.
const homeSettle = 3 * time.Second

type state struct {
	hold    int32
	fault   int32
	resetRe int32
}

var macRe = regexp.MustCompile(`^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$`)

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
	mac, rest := extractMAC(os.Args[1:])
	flag.CommandLine.Parse(rest)
	if mac == "" || flag.NArg() != 0 {
		flag.Usage()
		os.Exit(2)
	}

	dev, err := gimbal.Connect(mac)
	if err != nil {
		logf("fatal: connect %s: %v", mac, err)
		os.Exit(1)
	}
	defer dev.Close()

	lim := gcode.Limits{Pan: *panLim, Tilt: *tiltLim, Roll: *rollLim}
	ctl := gcode.NewController(lim, *defTime, *minTime, *maxTime)
	if *feedDef > 0 {
		ctl.SetFeed(*feedDef)
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
	var mu sync.Mutex
	cmdCh := make(chan gcode.Action, 10000)

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

	go executor(dev, &st, &mu, ctl, cmdCh)
	reader(pipeFile, dev, &mu, ctl, &st, cmdCh, mac, lim)
}

// executor owns the connection and the controller, running one command at a
// time (sequential resolution of the origin/feed state).
func executor(dev *gimbal.Device, st *state, mu *sync.Mutex, ctl *gcode.Controller, ch <-chan gcode.Action) {
	var pending *gcode.Action
	held := func() bool {
		return atomic.LoadInt32(&st.hold) == 1 || atomic.LoadInt32(&st.fault) == 1
	}
	reset := func() {
		drain(ch)
		pending = nil
		atomic.StoreInt32(&st.fault, 0)
		atomic.StoreInt32(&st.hold, 0)
		logf("reset")
	}
	for {
		if atomic.SwapInt32(&st.resetRe, 0) == 1 {
			reset()
			continue
		}
		if held() {
			time.Sleep(50 * time.Millisecond)
			continue
		}
		if pending == nil {
			select {
			case a := <-ch:
				aa := a
				pending = &aa
			case <-time.After(50 * time.Millisecond):
				continue
			}
		}
		if atomic.SwapInt32(&st.resetRe, 0) == 1 {
			reset()
			continue
		}
		if held() {
			continue // keep pending, wait for resume/reset
		}
		// Resolve against current controller state, sequentially.
		mu.Lock()
		plan := ctl.Plan(*pending)
		mu.Unlock()
		pending = nil
		run(dev, st, mu, ctl, plan)
	}
}

func run(dev *gimbal.Device, st *state, mu *sync.Mutex, ctl *gcode.Controller, p gcode.Plan) {
	if p.Err != nil {
		logf("error: %v", p.Err)
		atomic.StoreInt32(&st.fault, 1)
		atomic.StoreInt32(&st.hold, 1)
		return
	}
	switch p.Kind {
	case gcode.KindMotion:
		if !p.MovedAny {
			logf("ok")
			return
		}
		if err := dev.Move(planToParams(p)); err != nil {
			logf("error: %v", err)
			atomic.StoreInt32(&st.fault, 1)
			atomic.StoreInt32(&st.hold, 1)
			return
		}
		logf("ok")
	case gcode.KindHome:
		if err := dev.Home(); err != nil {
			logf("error: %v", err)
			atomic.StoreInt32(&st.fault, 1)
			atomic.StoreInt32(&st.hold, 1)
			return
		}
		time.Sleep(homeSettle)
		y, pt, r, err := dev.Angles()
		if err != nil {
			logf("error: home readback: %v", err)
			atomic.StoreInt32(&st.fault, 1)
			atomic.StoreInt32(&st.hold, 1)
			return
		}
		mu.Lock()
		ctl.SetOriginFromMeasured(y, pt, r)
		mu.Unlock()
		logf("ok (home: origin pan=%+.2f tilt=%+.2f roll=%+.2f)", y, pt, r)
	case gcode.KindDwell:
		time.Sleep(time.Duration(p.DwellMS) * time.Millisecond)
		logf("ok")
	case gcode.KindEnd:
		logf("ok (program end)")
	case gcode.KindIgnore:
		// modal / no-op
	}
}

func planToParams(p gcode.Plan) gimbal.MoveParams {
	mp := gimbal.MoveParams{Time: uint32(p.TimeMS)}
	if p.HasX {
		v := p.X
		mp.Yaw = &v
	}
	if p.HasY {
		v := p.Y
		mp.Pitch = &v
	}
	if p.HasZ {
		v := p.Z
		mp.Roll = &v
	}
	return mp
}

// reader lexes lines into actions and handles real-time commands inline.
func reader(f *os.File, dev *gimbal.Device, mu *sync.Mutex, ctl *gcode.Controller, st *state, ch chan<- gcode.Action, mac string, lim gcode.Limits) {
	var lx gcode.Lexer
	logf("ready: mac=%s pan=±%.0f tilt=±%.0f roll=±%.0f", mac, lim.Pan, lim.Tilt, lim.Roll)
	sc := bufio.NewScanner(f)
	sc.Buffer(make([]byte, 0, 4096), 64*1024)
	for sc.Scan() {
		line := sc.Text()
		if realtime(strings.TrimSpace(line), dev, mu, ctl, st) {
			continue
		}
		ch <- lx.Parse(line)
	}
	if err := sc.Err(); err != nil {
		logf("fifo read error: %v", err)
	}
}

// realtime handles immediate commands; returns true if the line was one.
func realtime(t string, dev *gimbal.Device, mu *sync.Mutex, ctl *gcode.Controller, st *state) bool {
	switch {
	case t == "?" || strings.EqualFold(t, "STATUS"):
		ux, uy, uz := func() (float64, float64, float64) {
			mu.Lock()
			defer mu.Unlock()
			return ctl.Position()
		}()
		mx, my, mz, _ := dev.Angles()
		logf("<%s,MPos:%.2f,%.2f,%.2f|Pos:%.2f,%.2f,%.2f>",
			stateName(st), ux, uy, uz, mx, my, mz)
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

func drain(ch <-chan gcode.Action) {
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
