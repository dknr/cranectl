package protocol

// Opcodes for commands used by the info feature.
// Values are the 16-bit encodings per SPEC-ble-frame.md.
// Group nibble is stripped on the wire; these constants preserve it so the
// name is easy to match against InteractCode.java.
var (
	// Manufacturer/device identity — read as four uint16 words, concatenated
	// (D,C,B,A) big-endian into an 8-byte device identifier.
	// Stabilizer.java:1543 (W8).
	OpcodeMakeInfoA = Opcode(0, 1, 124)
	OpcodeMakeInfoB = Opcode(0, 1, 125)
	OpcodeMakeInfoC = Opcode(0, 1, 126)
	OpcodeMakeInfoD = Opcode(0, 1, 127)

	// GET_BATTERY_VOLTAGE = h(0, 1, 6).
	OpcodeBattery = Opcode(0, 1, 6)

	// IMU (type=0xC0) firmware/hardware/machine version reads.
	OpcodeIMUBootAdr        = Opcode(192, 1, 16)
	OpcodeIMUMachineVersion = Opcode(192, 1, 17)
	OpcodeIMUBootVersion    = Opcode(192, 1, 18)
	OpcodeIMUHardwareVer    = Opcode(192, 1, 19)
	OpcodeIMUUUIDLow        = Opcode(192, 1, 20)
	OpcodeIMUUUIDHigh       = Opcode(192, 1, 21)
	OpcodeIMUFirmwareVer    = Opcode(192, 1, 22)

	// ICU (main MCU) — same encodings as Z-axis motor per InteractCode.
	OpcodeICUBootAdr        = Opcode(192, 4, 16)
	OpcodeICUMachineVersion = Opcode(192, 4, 17)
	OpcodeICUBootVersion    = Opcode(192, 4, 18)
	OpcodeICUHardwareVer    = Opcode(192, 4, 19)
	OpcodeICUUUIDLow        = Opcode(192, 4, 20)
	OpcodeICUUUIDHigh       = Opcode(192, 4, 21)
	OpcodeICUFirmwareVer    = Opcode(192, 4, 22)

	// PowerOn (h(192,1,3)) — handy smoke test; do NOT send from `info`.
	OpcodePowerOn = Opcode(192, 1, 3)
)
