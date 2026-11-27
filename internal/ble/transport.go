// Package ble implements the CRANEM2S BLE transport on top of
// tinygo.org/x/bluetooth.
package ble

import (
	"fmt"
	"io"
	"sync"
	"time"

	"cranectl/internal/protocol"

	tinygo_ble "tinygo.org/x/bluetooth"
)

// Zhiyun GATT layout — the app's variable names in tc/e0.java are misleading.
// BlueZ reports the ground truth for the M2S:
//
//	…-9600  Flags: [write-without-response]   app → gimbal
//	…-9601  Flags: [notify]                   gimbal → app
//
// Both live under the Quintic Corp. primary service (0000fee9-…).
// The …-9610 UUID that appears in the APK is absent on M2S.
const (
	WriteCharUUID  = "d44bc439-abfd-45a2-b575-925416129600"
	NotifyCharUUID = "d44bc439-abfd-45a2-b575-925416129601"
)

// Device is a scan hit — enough info to connect.
type Device struct {
	Name    string
	MAC     string
	RSSI    int16
	Address tinygo_ble.Address
}

// Scan runs a BLE scan for `timeout` and returns gimbal candidates.
// A candidate is any device advertising one of the Zhiyun characteristic
// UUIDs, or whose local name starts with a known Zhiyun model prefix.
func Scan(timeout time.Duration) ([]Device, error) {
	var (
		mu   sync.Mutex
		seen = map[string]Device{}
	)

	adapter := tinygo_ble.DefaultAdapter
	if err := adapter.Enable(); err != nil {
		return nil, fmt.Errorf("enable adapter: %w", err)
	}

	scanErr := make(chan error, 1)
	go func() {
		scanErr <- adapter.Scan(func(a *tinygo_ble.Adapter, r tinygo_ble.ScanResult) {
			name := r.AdvertisementPayload.LocalName()
			match := hasZhiyunName(name)
			if !match {
				for _, u := range []string{WriteCharUUID, NotifyCharUUID} {
					if pu, err := tinygo_ble.ParseUUID(u); err == nil && r.AdvertisementPayload.HasServiceUUID(pu) {
						match = true
						break
					}
				}
			}
			if !match {
				return
			}
			mac := r.Address.MAC.String()
			mu.Lock()
			defer mu.Unlock()
			if _, ok := seen[mac]; ok {
				return
			}
			seen[mac] = Device{
				Name:    name,
				MAC:     mac,
				RSSI:    r.RSSI,
				Address: r.Address,
			}
		})
	}()

	select {
	case <-time.After(timeout):
	}
	_ = adapter.StopScan()
	select {
	case <-scanErr:
	case <-time.After(500 * time.Millisecond):
	}

	mu.Lock()
	defer mu.Unlock()
	out := make([]Device, 0, len(seen))
	for _, d := range seen {
		out = append(out, d)
	}
	return out, nil
}

func hasZhiyunName(n string) bool {
	prefixes := []string{"CRANE", "Weebill", "Smooth", "ZY-", "ZHIYUN", "CR-"}
	for _, p := range prefixes {
		if len(n) >= len(p) && equalFold(n[:len(p)], p) {
			return true
		}
	}
	return false
}

func equalFold(a, b string) bool {
	for i := 0; i < len(a); i++ {
		ca, cb := a[i], b[i]
		if 'A' <= ca && ca <= 'Z' {
			ca += 32
		}
		if 'A' <= cb && cb <= 'Z' {
			cb += 32
		}
		if ca != cb {
			return false
		}
	}
	return true
}

// SetDebug enables hex dumps of every frame written and received.
func (c *Conn) SetDebug(on bool) { c.debug = on }

// Conn is an active BLE connection to a gimbal.
type Conn struct {
	adapter *tinygo_ble.Adapter
	dev     tinygo_ble.Device
	wch     tinygo_ble.DeviceCharacteristic

	debug bool

	mu    sync.Mutex
	seq   uint8 // last used seq
	pends map[uint8]chan protocol.Frame

	closeOnce sync.Once
	done      chan struct{}
}

// Connect dials the given MAC (colon-separated hex) and sets up
// the write / notify characteristics.
func Connect(mac string) (*Conn, error) {
	m, err := tinygo_ble.ParseMAC(mac)
	if err != nil {
		return nil, fmt.Errorf("parse mac %q: %w", mac, err)
	}
	addr := tinygo_ble.Address{MACAddress: tinygo_ble.NewMACAddress(m, false)}

	adapter := tinygo_ble.DefaultAdapter
	if err := adapter.Enable(); err != nil {
		return nil, fmt.Errorf("enable adapter: %w", err)
	}
	dev, err := adapter.Connect(addr, tinygo_ble.ConnectionParams{})
	if err != nil {
		return nil, fmt.Errorf("connect %s: %w", mac, err)
	}

	// Enumerate everything (nil filter), then pick characteristics by UUID.
	// On M2S the Zhiyun characteristics live under the Quintic 0xFEE9
	// primary service; on M3S they may be under a Zhiyun-named service.
	services, err := dev.DiscoverServices(nil)
	if err != nil {
		return nil, fmt.Errorf("discover services: %w", err)
	}

	want := map[string]bool{
		norm(WriteCharUUID):  true,
		norm(NotifyCharUUID): true,
	}
	found := map[string]tinygo_ble.DeviceCharacteristic{}
	for _, s := range services {
		chs, err := s.DiscoverCharacteristics(nil)
		if err != nil {
			continue
		}
		for _, ch := range chs {
			u := norm(ch.UUID().String())
			if want[u] {
				found[u] = ch
			}
		}
	}

	c := &Conn{
		adapter: adapter,
		dev:     dev,
		pends:   map[uint8]chan protocol.Frame{},
		done:    make(chan struct{}),
	}

	wch, ok := found[norm(WriteCharUUID)]
	if !ok {
		return nil, fmt.Errorf("write characteristic %s not found on gimbal", WriteCharUUID)
	}
	c.wch = wch

	nch, ok := found[norm(NotifyCharUUID)]
	if !ok {
		return nil, fmt.Errorf("notify characteristic %s not found on gimbal", NotifyCharUUID)
	}
	if err := nch.EnableNotifications(c.onNotify); err != nil {
		return nil, fmt.Errorf("enable notifications: %w", err)
	}
	return c, nil
}

// norm lowercases and dashes a UUID string so we can compare reliably
// regardless of how BlueZ formats it.
func norm(s string) string {
	out := make([]byte, 0, len(s))
	for i := 0; i < len(s); i++ {
		c := s[i]
		if c == '-' || c == '{' || c == '}' {
			continue
		}
		if 'A' <= c && c <= 'F' {
			c += 32
		}
		out = append(out, c)
	}
	return string(out)
}

// Close stops the local notification dispatcher. It deliberately does NOT
// tear down the BLE link: a clean BlueZ Disconnect over D-Bus blocks ~2.5 s
// and does not even drop the link (the OS/BlueZ keeps the paired device
// connected regardless), so disconnecting on every one-shot command would
// add ~2.5 s of pure overhead and still leave the gimbal connected. When the
// process exits, BlueZ reaps our D-Bus client and its notification watch;
// the link stays up for the next cranectl invocation to reuse (near-instant
// connect). To explicitly release the gimbal, use `bluetoothctl disconnect`.
func (c *Conn) Close() error {
	var err error
	c.closeOnce.Do(func() {
		close(c.done)
	})
	return err
}

// SendFrame writes f and blocks (up to timeout) for a response with
// matching seq. It is safe to call concurrently.
func (c *Conn) SendFrame(f protocol.Frame, timeout time.Duration) (protocol.Frame, error) {
	b := protocol.Build(f)

	ch := make(chan protocol.Frame, 1)
	c.mu.Lock()
	c.pends[f.Seq] = ch
	c.mu.Unlock()
	defer func() {
		c.mu.Lock()
		delete(c.pends, f.Seq)
		c.mu.Unlock()
	}()

	// No retry: this drives a physical machine. A frame that fails to get its
	// ack (dropped packet, stalled peripheral) must surface as an error so the
	// caller faults and halts — never silently re-issue a movement command.
	if c.debug {
		fmt.Printf("TX % x\n", b)
	}
	if _, err := c.wch.WriteWithoutResponse(b); err != nil {
		return protocol.Frame{}, fmt.Errorf("write: %w", err)
	}
	t := time.NewTimer(timeout)
	defer t.Stop()
	select {
	case reply := <-ch:
		return reply, nil
	case <-t.C:
		return protocol.Frame{}, fmt.Errorf("timeout waiting for reply to seq=%d opcode=0x%04X", f.Seq, f.Opcode)
	case <-c.done:
		return protocol.Frame{}, io.ErrClosedPipe
	}
}

// NextSeq returns a fresh sequence byte (1..127 wrapping), matching the
// Zhiyun app's di/e.java:9 counter.
func (c *Conn) NextSeq() uint8 {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.seq == 0 || c.seq >= 127 {
		c.seq = 1
	} else {
		c.seq++
	}
	return c.seq
}

// Read sends a read command (op with param=0) and returns the reply.
func (c *Conn) Read(opcode uint16, timeout time.Duration) (uint16, error) {
	f, err := c.SendFrame(protocol.Frame{
		Seq:    c.NextSeq(),
		Opcode: opcode,
		Param:  0,
	}, timeout)
	if err != nil {
		return 0, err
	}
	return f.Param, nil
}

// Write sends a command with a param, awaits ack (matching seq).
func (c *Conn) Write(opcode, param uint16, timeout time.Duration) error {
	_, err := c.SendFrame(protocol.Frame{
		Seq:    c.NextSeq(),
		Opcode: opcode,
		Param:  param,
	}, timeout)
	return err
}

// onNotify is the BLE notification callback — decode and dispatch.
func (c *Conn) onNotify(buf []byte) {
	if c.debug {
		fmt.Printf("RX % x\n", buf)
	}
	f, err := protocol.Parse(buf)
	if err != nil {
		return // silently drop malformed frames
	}
	c.mu.Lock()
	ch, ok := c.pends[f.Seq]
	c.mu.Unlock()
	if ok {
		select {
		case ch <- f:
		default:
		}
	}
}
