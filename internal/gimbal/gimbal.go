// Package gimbal provides a high-level API for the CRANEM2S.
package gimbal

import (
	"encoding/hex"
	"errors"
	"fmt"
	"math"
	"strings"
	"time"

	"cranectl/internal/ble"
	"cranectl/internal/protocol"
)

// DefaultTimeout is the per-command reply timeout.
const DefaultTimeout = 2 * time.Second

// DefaultMotionTimeMS is the point-to-position motion envelope.
const DefaultMotionTimeMS uint32 = 1500

// Info holds whatever we could read from the gimbal.
type Info struct {
	MakeInfoHex    string
	MakeInfoWords  [4]uint16 // A, B, C, D
	BatteryVolts   uint16
	IMUFirmwareVer uint16
	IMUHardwareVer uint16
	IMUMachineVer  uint16
	IMUBootVer     uint16
	IMUUUIDHigh    uint16
	IMUUUIDLow     uint16
	ICUFirmwareVer uint16
	ICUHardwareVer uint16
	ICUMachineVer  uint16
	ICUBootVer     uint16
	ICUUUIDHigh    uint16
	ICUUUIDLow     uint16
}

// MoveParams describes one point-to-position waypoint. Nil axis means
// "leave this axis alone" — mirrors cc/q1.java's 65535.0f sentinel.
// Time is the motion envelope in milliseconds (0 → DefaultMotionTimeMS).
type MoveParams struct {
	Yaw   *float64
	Roll  *float64
	Pitch *float64
	Time  uint32
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

// ReadInfo probes the gimbal for identity and firmware versions.
// Fields that fail are left zero and reported via warns.
func (d *Device) ReadInfo() (Info, []string, error) {
	var info Info
	var warns []string

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
		v, err := d.conn.Read(w.opcode, d.timeout)
		if err != nil {
			return info, warns, fmt.Errorf("read %s: %w", w.name, err)
		}
		*w.target = v
	}
	// Assemble per cc/l0.java: bytes = BE(D) || BE(C) || BE(B) || BE(A),
	// hex-formatted with any leading zeros stripped.
	raw := make([]byte, 0, 8)
	for _, i := range []int{3, 2, 1, 0} {
		raw = append(raw, byte(info.MakeInfoWords[i]>>8), byte(info.MakeInfoWords[i]))
	}
	info.MakeInfoHex = strings.TrimLeft(strings.ToUpper(hex.EncodeToString(raw)), "0")

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
		v, err := d.conn.Read(e.opcode, d.timeout)
		if err != nil {
			warns = append(warns, fmt.Sprintf("%s: %v", e.name, err))
			continue
		}
		*e.target = v
	}
	return info, warns, nil
}

// Move executes a single point-to-position transition. The full sequence
// mirrors com/zhiyun/device/a.java:93 → d(a, q1):
//
//	SET_BUTTON_EVENT(0x18)
//	LOCATION_SET_POINT_POWERED(1)
//	LOCATION_POINT_CONTROLLER(0)
//	[PITCH|ROLL|YAW]_ROTATE_ANGLE(deg × 100)   — per non-nil axis
//	POINT_MOTION_TIME_LOW_BIT / _HIGH_BIT
//	LOCATION_POINT_CONTROLLER(1)               — GO trigger
//	poll LOCATION_POINT_STATE_REGISTER bit 2
//	LOCATION_SET_POINT_POWERED(0)
func (d *Device) Move(mp MoveParams) error {
	if mp.Yaw == nil && mp.Roll == nil && mp.Pitch == nil {
		return errors.New("move: no axis specified")
	}
	t := mp.Time
	if t == 0 {
		t = DefaultMotionTimeMS
	}

	w := func(opcode uint16, param uint16, label string) error {
		if err := d.conn.Write(opcode, param, d.timeout); err != nil {
			return fmt.Errorf("%s: %w", label, err)
		}
		return nil
	}

	if err := w(protocol.OpcodeSetButtonEvent, 0x18, "set-button-event"); err != nil {
		return err
	}
	if err := w(protocol.OpcodeLocationSetPointPowered, 1, "set-point-power-on"); err != nil {
		return err
	}
	if err := w(protocol.OpcodeLocationPointController, 0, "controller-arm"); err != nil {
		return err
	}

	if mp.Pitch != nil {
		if err := w(protocol.OpcodePitchRotateAngle, degToParam(*mp.Pitch), "pitch"); err != nil {
			return err
		}
	}
	if mp.Roll != nil {
		if err := w(protocol.OpcodeRollRotateAngle, degToParam(*mp.Roll), "roll"); err != nil {
			return err
		}
	}
	if mp.Yaw != nil {
		if err := w(protocol.OpcodeYawRotateAngle, degToParam(*mp.Yaw), "yaw"); err != nil {
			return err
		}
	}

	if err := w(protocol.OpcodeMotionTimeLow, uint16(t&0xFFFF), "motion-time-low"); err != nil {
		return err
	}
	if err := w(protocol.OpcodeMotionTimeHigh, uint16((t>>16)&0xFF), "motion-time-high"); err != nil {
		return err
	}
	if err := w(protocol.OpcodeLocationPointController, 1, "controller-go"); err != nil {
		return err
	}

	if err := d.waitMoveDone(mp); err != nil {
		_ = w(protocol.OpcodeLocationSetPointPowered, 0, "set-point-power-off")
		return err
	}
	return w(protocol.OpcodeLocationSetPointPowered, 0, "set-point-power-off")
}

// Closed-loop settle parameters. The gimbal is an active stabilizer, so it
// holds a commanded attitude with small jitter; we require the live angle to
// stay within SettleTolerance of the target for SettleSamples consecutive
// polls before declaring the move complete.
const (
	SettleTolerance = 1.0 // degrees
	SettleSamples   = 3   // consecutive in-window reads
	SettlePollEvery = 80 * time.Millisecond
	SettleGrace     = 3 * time.Second // added to motion envelope as a cap
)

// waitMoveDone is the definitive completion gate. It polls the live attitude
// (YAW/PITCH/ROLL_ANGLE_R) and returns once every commanded axis is within
// SettleTolerance of its target for SettleSamples consecutive samples. If the
// gimbal has not settled by motionTime + SettleGrace it returns an error
// carrying the last observed angles, so "done" always means physically at
// target — never just "the envelope elapsed".
func (d *Device) waitMoveDone(mp MoveParams) error {
	motion := mp.Time
	if motion == 0 {
		motion = DefaultMotionTimeMS
	}
	deadline := time.Now().Add(time.Duration(motion)*time.Millisecond + SettleGrace)

	type axis struct {
		target   float64
		opcode   uint16
		inWindow int
	}
	var axes []axis
	if mp.Yaw != nil {
		axes = append(axes, axis{target: *mp.Yaw, opcode: protocol.OpcodeYawAngleR})
	}
	if mp.Pitch != nil {
		axes = append(axes, axis{target: *mp.Pitch, opcode: protocol.OpcodePitchAngleR})
	}
	if mp.Roll != nil {
		axes = append(axes, axis{target: *mp.Roll, opcode: protocol.OpcodeRollAngleR})
	}

	last := make([]float64, len(axes))
	var lastErr error
	for {
		allSettled := true
		for i := range axes {
			v, err := d.conn.Read(axes[i].opcode, d.timeout)
			if err != nil {
				lastErr = err
				allSettled = false
				axes[i].inWindow = 0
				continue
			}
			lastErr = nil
			deg := float64(int16(v)) / 100.0
			last[i] = deg
			if math.Abs(deg-axes[i].target) <= SettleTolerance {
				axes[i].inWindow++
			} else {
				axes[i].inWindow = 0
			}
			if axes[i].inWindow < SettleSamples {
				allSettled = false
			}
		}
		if allSettled {
			return nil
		}
		if time.Now().After(deadline) {
			if lastErr != nil {
				return fmt.Errorf("settle: %w", lastErr)
			}
			msg := "settle timeout, last angles:"
			for i := range axes {
				msg += fmt.Sprintf(" %+.2f°(want %+.2f°)", last[i], axes[i].target)
			}
			return errors.New(msg)
		}
		time.Sleep(SettlePollEvery)
	}
}

// Angles reads the live pan/tilt/roll attitude in degrees.
func (d *Device) Angles() (yaw, pitch, roll float64, err error) {
	if yaw, err = d.readAngle(protocol.OpcodeYawAngleR); err != nil {
		return
	}
	if pitch, err = d.readAngle(protocol.OpcodePitchAngleR); err != nil {
		return
	}
	roll, err = d.readAngle(protocol.OpcodeRollAngleR)
	return
}

func (d *Device) readAngle(opcode uint16) (float64, error) {
	v, err := d.conn.Read(opcode, d.timeout)
	if err != nil {
		return 0, err
	}
	return float64(int16(v)) / 100.0, nil
}

// degToParam converts signed degrees to the wire uint16 the gimbal expects
// (Stabilizer.p8 at line 2420 does Math.round(deg * 100.0f)).
func degToParam(deg float64) uint16 {
	return uint16(int16(math.Round(deg * 100.0)))
}
