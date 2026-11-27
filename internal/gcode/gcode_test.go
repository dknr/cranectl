package gcode

import "testing"

func newCtl() (Lexer, *Controller) {
	return Lexer{}, NewController(Limits{Pan: 180, Tilt: 90, Roll: 45}, 1500, 60, 60000)
}

func plan(t *testing.T, lx Lexer, c *Controller, line string) Plan {
	t.Helper()
	p := c.Plan(lx.Parse(line))
	if p.Err != nil {
		t.Fatalf("plan %q: unexpected err %v", line, p.Err)
	}
	return p
}

func TestAbsoluteMovePreHome(t *testing.T) {
	lx, c := newCtl()
	p := plan(t, lx, c, "G1 X30 Y20 F60") // origin=0 => firmware == user
	if p.Kind != KindMotion || p.X != 30 || p.Y != 20 {
		t.Fatalf("got %+v", p)
	}
	if p.TimeMS != 30000 { // 60000*30/60
		t.Fatalf("time %d", p.TimeMS)
	}
}

func TestHomeDefinesOriginThenX0IsHome(t *testing.T) {
	lx, c := newCtl()
	if p := plan(t, lx, c, "G28"); p.Kind != KindHome {
		t.Fatalf("G28 kind %d", p.Kind)
	}
	// Daemon, after home settles, reports measured pose:
	c.SetOriginFromMeasured(-18, 0, 0)
	p := plan(t, lx, c, "G1 X0 Y0")
	if p.X != -18 || p.Y != 0 {
		t.Fatalf("X0Y0 after home => fw (%.2f,%.2f), want (-18,0)", p.X, p.Y)
	}
}

func TestUserFrameOffsetAfterHome(t *testing.T) {
	lx, c := newCtl()
	plan(t, lx, c, "G28")
	c.SetOriginFromMeasured(-18, 0, 0)
	p := plan(t, lx, c, "G1 X30 Y30")
	if p.X != 12 || p.Y != 30 { // -18+30, 0+30
		t.Fatalf("got (%.2f,%.2f) want (12,30)", p.X, p.Y)
	}
}

func TestRapidMinTime(t *testing.T) {
	lx, c := newCtl()
	p := plan(t, lx, c, "G0 X45")
	if p.TimeMS != 60 {
		t.Fatalf("rapid time %d want 60", p.TimeMS)
	}
}

func TestExplicitTime(t *testing.T) {
	lx, c := newCtl()
	p := plan(t, lx, c, "G1 X10 T250")
	if p.TimeMS != 250 {
		t.Fatalf("time %d want 250", p.TimeMS)
	}
}

func TestRelativeAccumulates(t *testing.T) {
	lx, c := newCtl()
	plan(t, lx, c, "G90 G1 X0 Y0")
	plan(t, lx, c, "G91")
	p := plan(t, lx, c, "G1 X10")
	if p.X != 10 {
		t.Fatalf("first X=%v want 10", p.X)
	}
	p = plan(t, lx, c, "G1 X10")
	if p.X != 20 {
		t.Fatalf("second X=%v want 20", p.X)
	}
}

func TestOutOfRangeRejectedOnFirmwareTarget(t *testing.T) {
	lx, c := newCtl()
	plan(t, lx, c, "G28")
	c.SetOriginFromMeasured(0, 0, 0)
	a := lx.Parse("G1 Y999")
	if p := c.Plan(a); p.Err == nil {
		t.Fatalf("expected range error, got %+v", p)
	}
}

func TestG53MachineCoords(t *testing.T) {
	lx, c := newCtl()
	plan(t, lx, c, "G28")
	c.SetOriginFromMeasured(-18, 0, 0)
	p := plan(t, lx, c, "G53 X0 Y0") // raw firmware coords, ignore origin
	if p.X != 0 || p.Y != 0 {
		t.Fatalf("G53 gave (%.2f,%.2f) want (0,0)", p.X, p.Y)
	}
}

func TestDwell(t *testing.T) {
	lx, c := newCtl()
	p := plan(t, lx, c, "G4 P250")
	if p.Kind != KindDwell || p.DwellMS != 250 {
		t.Fatalf("got %+v", p)
	}
}

func TestComments(t *testing.T) {
	lx, c := newCtl()
	if p := plan(t, lx, c, "; nothing"); p.Kind != KindIgnore {
		t.Fatalf("comment kind %d", p.Kind)
	}
	p := plan(t, lx, c, "G1 X5 (comment) ; trailing")
	if p.Kind != KindMotion || p.X != 5 {
		t.Fatalf("inline comment got %+v", p)
	}
}

func TestG92ShiftsOrigin(t *testing.T) {
	lx, c := newCtl()
	plan(t, lx, c, "G1 X10") // user pos (10,0,0), fw (10,0,0)
	plan(t, lx, c, "G92 X0") // redefine current as X0 (shift origin)
	x, _, _ := c.Position()
	if x != 0 {
		t.Fatalf("pos x=%v want 0", x)
	}
	ox, _, _ := c.Origin()
	if ox != 10 {
		t.Fatalf("origin x=%v want 10 (shifted)", ox)
	}
	p := plan(t, lx, c, "G1 X5")
	if p.X != 15 { // origin(10) + 5
		t.Fatalf("X5 after G92 => fw %v want 15", p.X)
	}
}

func TestProgramEnd(t *testing.T) {
	lx, c := newCtl()
	if p := plan(t, lx, c, "M2"); p.Kind != KindEnd {
		t.Fatalf("M2 kind %d", p.Kind)
	}
}

func TestUnknownWordError(t *testing.T) {
	lx, c := newCtl()
	if p := c.Plan(lx.Parse("G1 X10 Q5")); p.Err == nil {
		t.Fatalf("expected error")
	}
}

func TestRecordSetSemantics(t *testing.T) {
	lx, c := newCtl()
	if p := plan(t, lx, c, "M3"); p.Kind != KindRecord || p.RecOp != RecOn {
		t.Fatalf("first M3: %+v", p)
	}
	if !c.Recording() {
		t.Fatal("expected recording=true after M3")
	}
	// Idempotent: M3 while already recording must not fire.
	if p := plan(t, lx, c, "M3"); p.Kind != KindIgnore {
		t.Fatalf("second M3 should be no-op, got %+v", p)
	}
	// M5 stops.
	if p := plan(t, lx, c, "M5"); p.Kind != KindRecord || p.RecOp != RecOff {
		t.Fatalf("M5: %+v", p)
	}
	if c.Recording() {
		t.Fatal("expected recording=false after M5")
	}
	if p := plan(t, lx, c, "M5"); p.Kind != KindIgnore {
		t.Fatalf("second M5 should be no-op, got %+v", p)
	}
	// M240 (photo) always fires.
	for i := 0; i < 2; i++ {
		if p := plan(t, lx, c, "M240"); p.Kind != KindRecord || p.RecOp != RecPhoto {
			t.Fatalf("M240 #%d: %+v", i, p)
		}
	}
}

func TestRecordLexerAcceptsCameraCodes(t *testing.T) {
	lx, _ := newCtl()
	for _, m := range []string{"M3", "M5", "M240"} {
		if a := lx.Parse(m); a.Kind != KindRecord {
			t.Fatalf("%s lexed to kind %d, want KindRecord", m, a.Kind)
		}
	}
	if a := lx.Parse("M7"); a.Err == nil {
		t.Fatal("M7 should be rejected")
	}
}
