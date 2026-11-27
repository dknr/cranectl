// Package gimbal provides a high-level API for the CRANEM2S.
package gimbal

import (
	"encoding/hex"
	"fmt"
	"strings"
	"time"

	"cranectl/internal/ble"
	"cranectl/internal/protocol"
)

// Default per-command reply timeout. Zhiyun's own app uses RxJava with
// generous defaults; 2s is comfortable for a good link.
const DefaultTimeout = 5 * time.Second

// Info holds whatever we could read from the gimbal.
type Info struct {
	MakeInfoHex       string
	MakeInfoWords     [4]uint16 // A, B, C, D
	BatteryVolts      uint16
	IMUFirmwareVer    uint16
	IMUHardwareVer    uint16
	IMUMachineVer     uint16
	IMUBootVer        uint16
	IMUUUIDHigh       uint16
	IMUUUIDLow        uint16
	ICUFirmwareVer    uint16
	ICUHardwareVer    uint16
	ICUMachineVer     uint16
	ICUBootVer        uint16
	ICUUUIDHigh       uint16
	ICUUUIDLow        uint16
}

// Device is a connected gimbal session.
type Device struct {
	conn    *ble.Conn
	timeout time.Duration
}

// Connect opens a session with the given MAC address (colon hex).
func Connect(mac string) (*Device, error) {
	c, err := ble.Connect(mac)
	if err != nil {
		return nil, err
	}
	return &Device{conn: c, timeout: DefaultTimeout}, nil
}

// Close releases the BLE connection.
func (d *Device) Close() error { return d.conn.Close() }

// SetDebug enables hex dumps of every frame written and received.
func (d *Device) SetDebug(on bool) { d.conn.SetDebug(on) }

// read is a small helper with the session default timeout.
func (d *Device) read(opcode uint16) (uint16, error) {
	return d.conn.Read(opcode, d.timeout)
}

// ReadInfo probes the gimbal for identity and firmware versions.
// Fields that time out or error are left zero and reported via Warns.
func (d *Device) ReadInfo() (Info, []string, error) {
	var info Info
	var warns []string

	// Make info (A..D) is the identity — if this fails we bail.
	words := []struct {
		opcode uint16
		name   string
		target *uint16
	}{
		{protocol.OpcodeMakeInfoA, "MAKE_INFO_A", &info.MakeInfoWords[0]},
		{protocol.OpcodeMakeInfoB, "MAKE_INFO_B", &info.MakeInfoWords[1]},
		{protocol.OpcodeMakeInfoC, "MAKE_INFO_C", &info.MakeInfoWords[2]},
		{protocol.OpcodeMakeInfoD, "MAKE_INFO_D", &info.MakeInfoWords[3]},
	}
	for _, w := range words {
		v, err := d.read(w.opcode)
		if err != nil {
			return info, warns, fmt.Errorf("read %s: %w", w.name, err)
		}
		*w.target = v
	}
	// Assemble per cc/l0.java: bytes = BE(D) || BE(C) || BE(B) || BE(A),
	// hex-formatted with a leading zero stripped.
	raw := make([]byte, 0, 8)
	for _, i := range []int{3, 2, 1, 0} {
		raw = append(raw, byte(info.MakeInfoWords[i]>>8), byte(info.MakeInfoWords[i]))
	}
	hexStr := strings.ToUpper(hex.EncodeToString(raw))
	info.MakeInfoHex = strings.TrimLeft(hexStr, "0")

	// Best-effort extras — record per-field failures rather than abort.
	extras := []struct {
		opcode uint16
		name   string
		target *uint16
	}{
		{protocol.OpcodeBattery, "Battery", &info.BatteryVolts},
		{protocol.OpcodeIMUFirmwareVer, "IMU.Firmware", &info.IMUFirmwareVer},
		{protocol.OpcodeIMUHardwareVer, "IMU.Hardware", &info.IMUHardwareVer},
		{protocol.OpcodeIMUMachineVersion, "IMU.Machine", &info.IMUMachineVer},
		{protocol.OpcodeIMUBootVersion, "IMU.Boot", &info.IMUBootVer},
		{protocol.OpcodeIMUUUIDHigh, "IMU.UUIDHigh", &info.IMUUUIDHigh},
		{protocol.OpcodeIMUUUIDLow, "IMU.UUIDLow", &info.IMUUUIDLow},
		{protocol.OpcodeICUFirmwareVer, "ICU.Firmware", &info.ICUFirmwareVer},
		{protocol.OpcodeICUHardwareVer, "ICU.Hardware", &info.ICUHardwareVer},
		{protocol.OpcodeICUMachineVersion, "ICU.Machine", &info.ICUMachineVer},
		{protocol.OpcodeICUBootVersion, "ICU.Boot", &info.ICUBootVer},
		{protocol.OpcodeICUUUIDHigh, "ICU.UUIDHigh", &info.ICUUUIDHigh},
		{protocol.OpcodeICUUUIDLow, "ICU.UUIDLow", &info.ICUUUIDLow},
	}
	for _, e := range extras {
		v, err := d.read(e.opcode)
		if err != nil {
			warns = append(warns, fmt.Sprintf("%s: %v", e.name, err))
			continue
		}
		*e.target = v
	}
	return info, warns, nil
}
