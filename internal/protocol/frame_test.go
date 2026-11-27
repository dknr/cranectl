package protocol

import (
	"encoding/hex"
	"strings"
	"testing"
)

// Worked examples with hdrVer=0x12 (the value jh/e.java's constructor sets
// via m().o(18)). Each hex string is the exact 14 bytes produced by the
// Java byte-builders on the Zhiyun side.
func TestBuild_MatchesSpecVectors(t *testing.T) {
	cases := []struct {
		name string
		f    Frame
		hex  string
	}{
		{"YAW_CONTROL(30)",
			Frame{Seq: 1, Opcode: Opcode(16, 0, 3), Param: 30},
			"243c0800181201010310 1e00 c463"},
		{"PITCH_CONTROL(30)",
			Frame{Seq: 1, Opcode: Opcode(16, 0, 1), Param: 30},
			"243c0800181201010110 1e00 ac8e"},
		{"ZOOM_CONTROL(100)",
			Frame{Seq: 1, Opcode: Opcode(16, 0, 5), Param: 100},
			"243c0800181201010510 6400 cfa3"},
		{"MSC_YAW_R(0)",
			Frame{Seq: 1, Opcode: Opcode(64, 1, 114), Param: 0},
			"243c080018120101 7240 0000 b729"},
		{"MSC_YAW_W(25)",
			Frame{Seq: 1, Opcode: Opcode(192, 1, 114), Param: 25},
			"243c080018120101 72c0 1900 06ab"},
		{"PHONE_CONTROL_MODE(1)",
			Frame{Seq: 1, Opcode: Opcode(192, 1, 56), Param: 1},
			"243c080018120101 38c0 0100 eb27"},
		{"PowerOn",
			Frame{Seq: 1, Opcode: Opcode(192, 1, 3), Param: 0},
			"243c080018120101 03c0 0000 2c26"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			want := strings.ReplaceAll(tc.hex, " ", "")
			got := Build(tc.f)
			if hex.EncodeToString(got) != want {
				t.Fatalf("Build() = %x\nwant     %s", got, want)
			}
		})
	}
}

func TestParse_RoundTripsBuiltFrame(t *testing.T) {
	in := Frame{Seq: 42, Opcode: Opcode(16, 0, 3), Param: 12345}
	b := Build(in)
	out, err := Parse(b)
	if err != nil {
		t.Fatalf("Parse: %v", err)
	}
	if out.Seq != in.Seq || out.Param != in.Param {
		t.Fatalf("roundtrip: got %+v want %+v", out, in)
	}
	// Opcode survives with group forced to 1 (matches mh/a.java:47).
	want := (in.Opcode & 0xF0FF) | (1 << 8)
	if out.Opcode != want {
		t.Fatalf("opcode: got 0x%04X want 0x%04X", out.Opcode, want)
	}
}

func TestParse_RejectsBadCRC(t *testing.T) {
	b := Build(Frame{Seq: 1, Opcode: Opcode(16, 0, 3), Param: 1})
	b[12] ^= 0xFF
	if _, err := Parse(b); err != errBadCRC {
		t.Fatalf("expected bad CRC, got %v", err)
	}
}

func TestParse_RejectsBadMagic(t *testing.T) {
	b := Build(Frame{Seq: 1, Opcode: Opcode(16, 0, 3), Param: 1})
	b[1] = 0x00 // neither 0x3C (TX) nor 0x3E (RX)
	if _, err := Parse(b); err != errBadMagic {
		t.Fatalf("expected bad magic, got %v", err)
	}
}

func TestCRC16_Table(t *testing.T) {
	// Spot-check a couple of table entries against the ones extracted from
	// di/d.java's hand-coded table.
	if crcTable[0] != 0 {
		t.Fatalf("table[0]=%d", crcTable[0])
	}
	if crcTable[1] != 0x1021 {
		t.Fatalf("table[1]=0x%04X", crcTable[1])
	}
	if crcTable[16] != 0x1231 {
		t.Fatalf("table[16]=0x%04X want 0x1231", crcTable[16])
	}
	if crcTable[17] != 528 {
		t.Fatalf("table[17]=%d want 528 (was obfuscated in decompile)", crcTable[17])
	}
	if crcTable[207] != 10403 {
		t.Fatalf("table[207]=%d want 10403", crcTable[207])
	}
	// CRC-16/XMODEM of "123456789" = 0x31C3 (published reference vector).
	if got := crc16([]byte("123456789")); got != 0x31C3 {
		t.Fatalf("crc16(123456789)=0x%04X want 0x31C3", got)
	}
}
