package protocol

// Opcodes used by cranectl. Values are the 16-bit encodings per
// SPEC-ble-frame.md. Group nibble is stripped on the wire; these constants
// preserve it so the name is easy to match against InteractCode.java.

// — Info reads (used by `cranectl info`) —
var (
	OpcodeMakeInfoA = Opcode(0, 1, 124)
	OpcodeMakeInfoB = Opcode(0, 1, 125)
	OpcodeMakeInfoC = Opcode(0, 1, 126)
	OpcodeMakeInfoD = Opcode(0, 1, 127)

	OpcodeBattery = Opcode(0, 1, 6)

	OpcodeIMUBootAdr        = Opcode(192, 1, 16)
	OpcodeIMUMachineVersion = Opcode(192, 1, 17)
	OpcodeIMUBootVersion    = Opcode(192, 1, 18)
	OpcodeIMUHardwareVer    = Opcode(192, 1, 19)
	OpcodeIMUUUIDLow        = Opcode(192, 1, 20)
	OpcodeIMUUUIDHigh       = Opcode(192, 1, 21)
	OpcodeIMUFirmwareVer    = Opcode(192, 1, 22)

	OpcodeICUBootAdr        = Opcode(192, 4, 16)
	OpcodeICUMachineVersion = Opcode(192, 4, 17)
	OpcodeICUBootVersion    = Opcode(192, 4, 18)
	OpcodeICUHardwareVer    = Opcode(192, 4, 19)
	OpcodeICUUUIDLow        = Opcode(192, 4, 20)
	OpcodeICUUUIDHigh       = Opcode(192, 4, 21)
	OpcodeICUFirmwareVer    = Opcode(192, 4, 22)
)

// — Movement (used by `cranectl <MAC> move`) —
// The Zhiyun point-to-position protocol is a state machine. Rotation
// opcodes only take effect between LOCATION_POINT_CONTROLLER(0) and
// LOCATION_POINT_CONTROLLER(1), and the whole sequence must run with
// LOCATION_SET_POINT_POWERED(1) asserted. See com/zhiyun/device/a.java:93.
var (
	OpcodeLocationSetPointPowered    = Opcode(192, 1, 48) // 0xC130
	OpcodeLocationPointController    = Opcode(192, 1, 49) // 0xC131
	OpcodeLocationPointStateRegister = Opcode(192, 1, 50) // 0xC132
	OpcodePitchRotateAngle           = Opcode(192, 1, 51) // 0xC133
	OpcodeRollRotateAngle            = Opcode(192, 1, 52) // 0xC134
	OpcodeYawRotateAngle             = Opcode(192, 1, 53) // 0xC135
	OpcodeMotionTimeLow              = Opcode(192, 1, 54) // 0xC136
	OpcodeMotionTimeHigh             = Opcode(192, 1, 55) // 0xC137
	OpcodeSetButtonEvent             = Opcode(192, 1, 32) // 0xC120
)
