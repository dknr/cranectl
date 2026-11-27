// cranectl — CLI for the Zhiyun CRANEM2S gimbal over BLE.
//
// Usage:
//
//	cranectl scan                    list nearby gimbals
//	cranectl info <MAC>              read model + version info
package main

import (
	"flag"
	"fmt"
	"os"
	"sort"
	"text/tabwriter"
	"time"

	"cranectl/internal/ble"
	"cranectl/internal/gimbal"
)

func main() {
	if len(os.Args) < 2 {
		usage()
		os.Exit(2)
	}
	switch os.Args[1] {
	case "scan":
		scanCmd(os.Args[2:])
	case "info":
		infoCmd(os.Args[2:])
	case "-h", "--help", "help":
		usage()
	default:
		fmt.Fprintf(os.Stderr, "unknown command %q\n", os.Args[1])
		usage()
		os.Exit(2)
	}
}

func usage() {
	fmt.Fprintln(os.Stderr, "usage: cranectl <command> [args]")
	fmt.Fprintln(os.Stderr, "commands:")
	fmt.Fprintln(os.Stderr, "  scan [--timeout 5s]        list nearby Zhiyun gimbals")
	fmt.Fprintln(os.Stderr, "  info <MAC>                 read device info")
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

func infoCmd(args []string) {
	fs := flag.NewFlagSet("info", flag.ExitOnError)
	debug := fs.Bool("debug", false, "hex-dump every BLE frame")
	_ = fs.Parse(args)
	if fs.NArg() < 1 {
		fmt.Fprintln(os.Stderr, "usage: cranectl info [--debug] <MAC>")
		os.Exit(2)
	}
	mac := fs.Arg(0)

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

func fail(f string, a ...any) {
	fmt.Fprintf(os.Stderr, "error: "+f+"\n", a...)
	os.Exit(1)
}
