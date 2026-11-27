// Package protocol implements the CRANEM2S BLE wire format.
//
// Spec: ~/Projects/zhitun-crane-m3s-re/SPEC-ble-frame.md
package protocol

// Frame is a decoded request/response on the CRANEM2S BLE link.
type Frame struct {
	Seq    uint8
	Opcode uint16
	Param  uint16
}

// Opcode encodes (type, group, op) into the 16-bit wire opcode.
// Mirrors di/h.java:154.
func Opcode(typ, group, op uint8) uint16 {
	return (uint16(typ>>4) << 12) | (uint16(group&0xF) << 8) | uint16(op)
}

// crc16 computes CRC-16/XMODEM (poly 0x1021, init 0, MSB-first, no reflection,
// no final XOR) — matches di/d.java.
func crc16(data []byte) uint16 {
	var c uint16
	for _, b := range data {
		c = (c << 8) ^ crcTable[int(b)^int(c>>8)]
	}
	return c
}

var crcTable = func() [256]uint16 {
	var t [256]uint16
	for i := 0; i < 256; i++ {
		c := uint16(i) << 8
		for k := 0; k < 8; k++ {
			if c&0x8000 != 0 {
				c = (c << 1) ^ 0x1021
			} else {
				c <<= 1
			}
		}
		t[i] = c
	}
	return t
}()

const (
	magicLo    byte = 0x24 // request magic low byte  (0x3C24 LE, kh/b.java f34433a=15396)
	magicHi    byte = 0x3C // request magic high byte
	// BLE replies from the gimbal use magic 0x3E24 in some message classes
	// (kh/b.java also defines f34432j=15908=0x3E24). The Zhiyun app's BLE
	// parseFrom path never validates magic, so we accept both here.
	hdrFlags   byte = 0x18 // di.b.n(8,1,true,false)
	hdrVer     byte = 0x12 // jh/e.java:10 does m().o(18)  →  hdr[5] = 18
	bodyFlags  byte = 0x01 // mh/a.java:20 -> di.b.n(1,0,true,false)
	hdrLen           = 6
	bodyLen          = 6
	crcLen           = 2
	frameLen         = hdrLen + bodyLen + crcLen
)

// Build serialises f into a full 14-byte frame ready to hand to the
// write characteristic (D44BC439-...-9601).
func Build(f Frame) []byte {
	opcode := f.Opcode
	out := make([]byte, 0, frameLen)
	// header
	total := uint16(bodyLen + crcLen) // body + crc, matches jh/a.java:36
	out = append(out, magicLo, magicHi)
	out = append(out, byte(total), byte(total>>8))
	out = append(out, hdrFlags, hdrVer)
	// body
	out = append(out,
		f.Seq,
		bodyFlags,
		byte(opcode&0xFF),           // opLow
		byte((opcode&0xF0FF)>>8),    // opHigh (group nibble stripped)
		byte(f.Param), byte(f.Param>>8),
	)
	// CRC over last 2 header bytes + body (see jh/p.java:22)
	c := crc16(out[4:])
	out = append(out, byte(c), byte(c>>8))
	return out
}

// Parse decodes a full 14-byte frame from the notify characteristic
// (D44BC439-...-9610). It validates the magic, length, and CRC.
// The Zhiyun app skips CRC validation on receive; we don't.
func Parse(b []byte) (Frame, error) {
	if len(b) < frameLen {
		return Frame{}, errShortFrame
	}
	// Magic: the Zhiyun BLE parseFrom path does not validate magic bytes;
	// accept anything so we don't reject frames the app would accept.
	if b[1] != magicHi && b[1] != 0x3E {
		return Frame{}, errBadMagic
	}
	if uint16(b[2])|uint16(b[3])<<8 != uint16(bodyLen+crcLen) {
		return Frame{}, errBadLength
	}
	got := uint16(b[12]) | uint16(b[13])<<8
	if want := crc16(b[4:12]); got != want {
		return Frame{}, errBadCRC
	}
	// Opcode is stored as [opLow, opHigh] where opHigh carries the type byte
	// in bits 4..7 (shifted by (type>>4)<<4 on send). Reconstruct the way
	// mh/a.java:47 does: h(opHigh, 1, opLow), which promotes opHigh's top
	// nibble into bits 12..15 and forces group=1.
	opLow := uint16(b[8])
	opHigh := uint16(b[9])
	opcode := (opHigh&0xF0)<<8 | (1 << 8) | opLow
	return Frame{
		Seq:    b[6],
		Opcode: opcode,
		Param:  uint16(b[10]) | uint16(b[11])<<8,
	}, nil
}

// FrameLen is the size of a serialised frame.
const FrameLen = frameLen
