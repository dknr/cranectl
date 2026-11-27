package gcode

import "testing"

func newp() *Parser {
	return NewParser(Limits{Pan: 180, Tilt: 90, Roll: 45}, 1500, 60, 60000)
}

func TestAbsoluteMoveFeed(t *testing.T) {
	p := newp()
	c := p.Parse("G1 X30 Y20 F60") // dist=30 deg, feed=60deg/min -> 60*30/60=30000ms? 60000*30/60=30000
	if c.Kind != KindMove {
		t.Fatalf("kind=%v err=%v", c.Kind, c.Err)
	}
	if c.X != 30 || c.Y != 20 {
		t.Fatalf("target got %v %v", c.X, c.Y)
	}
	if c.TimeMS != 30000 {
		t.Fatalf("time got %d want 30000", c.TimeMS)
	}
}

func TestRapidUsesMinTime(t *testing.T) {
	p := newp()
	c := p.Parse("G0 X45")
	if !c.Rapid {
		t.Fatalf("expected rapid")
	}
	if c.TimeMS != 60 { // G0 -> min time floor
		t.Fatalf("time got %d want 60", c.TimeMS)
	}
}

func TestExplicitTimeOverrides(t *testing.T) {
	p := newp()
	c := p.Parse("G1 X10 T250")
	if c.TimeMS != 250 {
		t.Fatalf("time got %d want 250", c.TimeMS)
	}
}

func TestRelativeAccumulates(t *testing.T) {
	p := newp()
	p.Parse("G90 G1 X0 Y0")
	p.Parse("G91")
	c := p.Parse("G1 X10")
	if c.X != 10 {
		t.Fatalf("got %v want 10", c.X)
	}
	c = p.Parse("G1 X10")
	if c.X != 20 {
		t.Fatalf("got %v want 20", c.X)
	}
}

func TestOutOfRangeRejected(t *testing.T) {
	p := newp()
	c := p.Parse("G1 X999")
	if c.Kind != KindError {
		t.Fatalf("expected error, got %v", c.Kind)
	}
}

func TestDwell(t *testing.T) {
	p := newp()
	c := p.Parse("G4 P250")
	if c.Kind != KindDwell || c.DwellMS != 250 {
		t.Fatalf("dwell got %+v", c)
	}
}

func TestCommentsAndProgramEnd(t *testing.T) {
	p := newp()
	if c := p.Parse("; just a comment"); c.Kind != KindIgnore {
		t.Fatalf("comment got %v", c.Kind)
	}
	if c := p.Parse("G1 X5 (move pan) ; trailing"); c.Kind != KindMove || c.X != 5 {
		t.Fatalf("inline comment got %+v", c)
	}
	if c := p.Parse("M2"); c.Kind != KindProgramEnd {
		t.Fatalf("M2 got %v", c.Kind)
	}
}

func TestHome(t *testing.T) {
	p := newp()
	p.Parse("G1 X30 Y20")
	c := p.Parse("G28")
	if c.Kind != KindMove || c.X != 0 || c.Y != 0 {
		t.Fatalf("home got %+v", c)
	}
}

func TestUnknownWordError(t *testing.T) {
	p := newp()
	if c := p.Parse("G1 X10 Q5"); c.Kind != KindError {
		t.Fatalf("expected error, got %v", c.Kind)
	}
}

func TestG92SetsDatum(t *testing.T) {
	p := newp()
	p.Parse("G92 X100")
	x, _, _ := p.Position()
	if x != 100 {
		t.Fatalf("datum x=%v want 100", x)
	}
}
