// cranectl — CLI for the Zhiyun CRANEM2S gimbal over BLE.
//
// Discovery has no target; every other command is "control <this gimbal> by
// doing <this operation>", so the MAC is the first positional parameter.
//
// Usage:
//
//	cranectl scan [--timeout 5s]
//	cranectl <MAC> info [--debug]
//	cranectl <MAC> move <panDeg> <tiltDeg> [--roll N] [--time ms] [--debug]
package main

import (
	"flag"
	"fmt"
	"os"
	"regexp"
	"sort"
	"strconv"
	"strings"
	"text/tabwriter"
	"time"

	"cranectl/internal/ble"
	"cranectl/internal/gimbal"
)

var macRe = regexp.MustCompile(`^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$`)

func main() {
	if len(os.Args) < 2 {
		usage()
		os.Exit(2)
	}
	// Discovery (no target).
	if os.Args[1] == "scan" {
		scanCmd(os.Args[2:])
		return
	}
	if os.Args[1] == "-h" || os.Args[1] == "--help" || os.Args[1] == "help" {
		usage()
		return
	}
	// Everything else: cranectl <MAC> <command> ...
	if !macRe.MatchString(os.Args[1]) {
		fmt.Fprintf(os.Stderr, "first argument must be a MAC (or 'scan'/'help'), got %q\n", os.Args[1])
		usage()
		os.Exit(2)
	}
	mac := os.Args[1]
	if len(os.Args) < 3 {
		fail("missing command after %s", mac)
	}
	switch os.Args[2] {
	case "info":
		infoCmd(mac, os.Args[3:])
	case "move":
		moveCmd(mac, os.Args[3:])
	default:
		fail("unknown command %q (expected 'info' or 'move')", os.Args[2])
	}
}

func usage() {
	fmt.Fprintln(os.Stderr, "usage:")
	fmt.Fprintln(os.Stderr, "  cranectl scan [--timeout 5s]")
	fmt.Fprintln(os.Stderr, "  cranectl <MAC> info [--debug]")
	fmt.Fprintln(os.Stderr, "  cranectl <MAC> move <panDeg> <tiltDeg> [--roll N] [--time ms] [--debug]")
}

func scanCmd(args []string) {
	fs := flag.NewFlagSet("scan", flag.ExitOnError)
	timeout := fs.Duration("timeout", 5*time.Second, "scan duration")
	_ = fs.Parse(args)

	fmt.Printf("scanning for %s...\n", *timeout)
	devs, err := ble.Scan(*timeout)
	if err != nil {
		fail("scan: %v", err)
	}
	if len(devs) == 0 {
		fmt.Println("(none found)")
		return
	}
	sort.Slice(devs, func(i, j int) bool { return devs[i].RSSI > devs[j].RSSI })
	w := tabwriter.NewWriter(os.Stdout, 0, 2, 2, ' ', 0)
	fmt.Fprintln(w, "MAC\tRSSI\tNAME")
	for _, d := range devs {
		name := d.Name
		if name == "" {
			name = "-"
		}
		fmt.Fprintf(w, "%s\t%d\t%s\n", d.MAC, d.RSSI, name)
	}
	_ = w.Flush()
}

func infoCmd(mac string, args []string) {
	fs := flag.NewFlagSet("info", flag.ExitOnError)
	debug := fs.Bool("debug", false, "hex-dump every BLE frame")
	_ = fs.Parse(args)
	if fs.NArg() != 0 {
		fail("usage: cranectl <MAC> info [--debug]")
	}
	dev, err := gimbal.Connect(mac)
	if err != nil {
		fail("connect %s: %v", mac, err)
	}
	defer dev.Close()
	dev.SetDebug(*debug)

	info, warns, err := dev.ReadInfo()
	if err != nil {
		fail("read info: %v", err)
	}
	fmt.Printf("MAC              %s\n", mac)
	fmt.Printf("Device ID        %s\n", info.MakeInfoHex)
	fmt.Printf("MakeInfo A/B/C/D 0x%04X / 0x%04X / 0x%04X / 0x%04X\n",
		info.MakeInfoWords[0], info.MakeInfoWords[1],
		info.MakeInfoWords[2], info.MakeInfoWords[3])
	fmt.Println()
	fmt.Println("Battery:")
	fmt.Printf("  voltage raw    %d\n", info.BatteryVolts)
	fmt.Println()
	fmt.Println("IMU:")
	fmt.Printf("  firmware       0x%04X (%d)\n", info.IMUFirmwareVer, info.IMUFirmwareVer)
	fmt.Printf("  hardware       0x%04X (%d)\n", info.IMUHardwareVer, info.IMUHardwareVer)
	fmt.Printf("  machine        0x%04X (%d)\n", info.IMUMachineVer, info.IMUMachineVer)
	fmt.Printf("  boot           0x%04X (%d)\n", info.IMUBootVer, info.IMUBootVer)
	fmt.Printf("  uuid (hi,lo)   0x%04X:%04X\n", info.IMUUUIDHigh, info.IMUUUIDLow)
	fmt.Println()
	fmt.Println("ICU (main MCU):")
	fmt.Printf("  firmware       0x%04X (%d)\n", info.ICUFirmwareVer, info.ICUFirmwareVer)
	fmt.Printf("  hardware       0x%04X (%d)\n", info.ICUHardwareVer, info.ICUHardwareVer)
	fmt.Printf("  machine        0x%04X (%d)\n", info.ICUMachineVer, info.ICUMachineVer)
	fmt.Printf("  boot           0x%04X (%d)\n", info.ICUBootVer, info.ICUBootVer)
	fmt.Printf("  uuid (hi,lo)   0x%04X:%04X\n", info.ICUUUIDHigh, info.ICUUUIDLow)
	if len(warns) > 0 {
		fmt.Println()
		fmt.Println("Warnings:")
		for _, w := range warns {
			fmt.Printf("  - %s\n", w)
		}
	}
}

func moveCmd(mac string, args []string) {
	// Manual parse: signed angles (-30) would be mistaken for flags by
	// flag.Parse, and we want flags to be usable before or after the
	// positional degrees. Recognised: --roll N, --time N, --debug, and "--".
	var (
		rollStr    string
		timeVal    int64 = int64(gimbal.DefaultMotionTimeMS)
		debug      bool
		positional []string
		noFlags    bool
	)
	for i := 0; i < len(args); i++ {
		a := args[i]
		if noFlags {
			positional = append(positional, a)
			continue
		}
		switch {
		case a == "--":
			noFlags = true
		case a == "--debug" || a == "-debug":
			debug = true
		case a == "--roll" || a == "-roll":
			if i+1 >= len(args) {
				fail("%s requires a value", a)
			}
			i++
			rollStr = args[i]
		case hasPrefixAny(a, "--roll=", "-roll="):
			rollStr = a[strings.IndexByte(a, '=')+1:]
		case a == "--time" || a == "-time":
			if i+1 >= len(args) {
				fail("%s requires a value", a)
			}
			i++
			v, err := strconv.ParseInt(args[i], 10, 64)
			if err != nil || v < 0 {
				fail("bad time %q", args[i])
			}
			timeVal = v
		case hasPrefixAny(a, "--time=", "-time="):
			v, err := strconv.ParseInt(a[strings.IndexByte(a, '=')+1:], 10, 64)
			if err != nil || v < 0 {
				fail("bad time %q", a)
			}
			timeVal = v
		default:
			positional = append(positional, a)
		}
	}
	if len(positional) != 2 {
		fail("usage: cranectl <MAC> move <panDeg> <tiltDeg> [--roll N] [--time ms] [--debug]")
	}
	parseDeg := func(s string) float64 {
		v, err := strconv.ParseFloat(s, 64)
		if err != nil {
			fail("bad angle %q: %v", s, err)
		}
		return v
	}
	pan := parseDeg(positional[0])
	tilt := parseDeg(positional[1])
	var roll *float64
	if rollStr != "" {
		r := parseDeg(rollStr)
		roll = &r
	}

	dev, err := gimbal.Connect(mac)
	if err != nil {
		fail("connect %s: %v", mac, err)
	}
	defer dev.Close()
	dev.SetDebug(debug)

	if err := dev.Move(gimbal.MoveParams{
		Yaw:   &pan,
		Pitch: &tilt,
		Roll:  roll,
		Time:  uint32(timeVal),
	}); err != nil {
		fail("%v", err)
	}
	yaw, pitch, ro, aerr := dev.Angles()
	if aerr != nil {
		if roll != nil {
			fmt.Printf("moved pan=%+.1f° tilt=%+.1f° roll=%+.1f° (%d ms)\n", pan, tilt, *roll, timeVal)
		} else {
			fmt.Printf("moved pan=%+.1f° tilt=%+.1f° (%d ms)\n", pan, tilt, timeVal)
		}
		return
	}
	if roll != nil {
		fmt.Printf("moved to pan=%+.1f° tilt=%+.1f° roll=%+.1f° (%d ms); measured pan=%+.2f° tilt=%+.2f° roll=%+.2f°\n",
			pan, tilt, *roll, timeVal, yaw, pitch, ro)
	} else {
		fmt.Printf("moved to pan=%+.1f° tilt=%+.1f° (%d ms); measured pan=%+.2f° tilt=%+.2f°\n",
			pan, tilt, timeVal, yaw, pitch)
	}
}

func hasPrefixAny(s string, prefixes ...string) bool {
	for _, p := range prefixes {
		if strings.HasPrefix(s, p) {
			return true
		}
	}
	return false
}

func fail(f string, a ...any) {
	if !strings.Contains(f, "\n") && !strings.HasSuffix(f, "\n") {
		f += "\n"
	}
	fmt.Fprintf(os.Stderr, "error: "+f, a...)
	os.Exit(1)
}
