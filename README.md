# cranectl

Command-line control of the **Zhiyun CRANE-M2S** 3-axis gimbal over Bluetooth
Low Energy — pan/tilt moves, live attitude readback, and a G-code motion daemon.

This is an unofficial tool, not affiliated with or endorsed by Zhiyun.

## Requirements

- Linux with a working Bluetooth adapter and **BlueZ** (tested with the device
  paired and auto-connecting).
- **Go** 1.22+.

## Build

```sh
go build -o cranectl  ./cmd/cranectl
go build -o cranectld ./cmd/cranectld
```

## Find and inspect the gimbal

```sh
./cranectl scan                              # list nearby Zhiyun gimbals
./cranectl 53:37:60:0F:2F:30 info            # device id, battery, firmware
```

## Move

```sh
./cranectl <MAC> move <panDeg> <tiltDeg> [--roll N] [--time ms] [--debug]
```

Degrees are signed; `--time` is the motion envelope in milliseconds. A move is
closed-loop: the command sequence is sent, then the live attitude is polled
until every commanded axis is within tolerance of its target for several
consecutive reads — so a successful `move` means the gimbal physically arrived,
not just that the envelope elapsed.

## cranectld — G-code daemon

`cranectld` keeps one persistent BLE connection and drains G-code lines from a
named pipe, mapping each motion block to the move sequence. GRBL-style
real-time commands are handled immediately.

```sh
mkfifo ~/ctl.fifo
./cranectld <MAC> --pipe ~/ctl.fifo --pidfile ~/ctl.pid > ~/ctl.log 2>&1 &

cat shot.g > ~/ctl.fifo        # run a program
printf '?\n'     > ~/ctl.fifo  # report state + measured position
printf '!\n'     > ~/ctl.fifo  # feed hold
printf '~\n'     > ~/ctl.fifo  # resume
printf 'RESET\n' > ~/ctl.fifo  # soft reset (drain queue, clear fault)

kill $(cat ~/ctl.pid)          # release the gimbal
```

Example `shot.g`:

```gcode
G90                 ; absolute positioning
G28                 ; gimbal home (becomes user 0,0)
G1 Y-20 T2000       ; tilt down 20 deg in 2s (from home)
G4 P300             ; dwell 300 ms
G1 X30 Y30 T15000   ; coordinated pan + tilt-up over 15s
M2                  ; program end
```

### Supported G-code

Coordinates are **degrees**: `X` = pan/yaw, `Y` = tilt/pitch, `Z` = roll.

Absolute moves are in a **user frame** relative to a settable origin. Homing
(`G28`) drives the gimbal to its physical home (same as a trigger double-click)
and makes that pose the origin — so right after `G28`, `G1 X0 Y0` *is* home.

| Code | Meaning |
|------|---------|
| `G0 X_ Y_ Z_` | rapid move (minimum motion time) |
| `G1 X_ Y_ Z_ [F_] \| [T_]` | controlled move; `F` = feedrate (deg/min), `T` = explicit time (ms) |
| `G4 P<ms>` | dwell |
| `G28` | gimbal home (`BACK_CENTER`) and define the origin there |
| `G53 X_ Y_ Z_` | move in raw firmware/machine coordinates (ignores the origin) |
| `G90` / `G91` | absolute (default) / relative (user frame) |
| `G92 X_ Y_ Z_` | redefine the current position (shifts the origin) |
| `G20` / `G21` | accepted, ignored (units are always degrees) |
| `M3` / `M5` | start / stop video recording (tracked; idempotent set-semantics) |
| `M240` | take a still photo (momentary) |
| `M2` / `M30` | program end |
| `;` … / `(...)` | comments |
| `N<line>` | line number (ignored) |

Anything else — unknown words, unsupported codes, or out-of-range axes — is
**rejected** with an `error:` and puts the daemon into an `Alarm` state until a
soft reset. Axes are never silently wrapped.

**Camera.** `M3`/`M5` drive the gimbal's record key and `M240` its shutter. The
record key is a hardware *toggle*; the daemon keeps a `recording` bool so `M3`
while already recording (or `M5` while stopped) is a no-op. `?` reports it as
`Rec:0/1`. This tracked bool is authoritative until a definitive
"currently recording?" status flag is wired in.

### Two "centers"

- **Gimbal home** — `G28` (and the physical trigger double-click). Tilt and roll
  home is stable (gravity-referenced); the pan axis has no absolute reference,
  so `G28` re-zeros pan wherever it happens to sit (it can land anywhere). What
  matters is that `G28` captures the resulting pose as the **origin**, so user
  `0,0` is always that home pose. `?` always reports the true measured `Pos:`.
- **Firmware datum** — readback coordinate `0,0`. Reach it explicitly with
  `G53 X0 Y0`. Useful for calibration work.

The `?` real-time command reports both: `MPos:` is the user (origin-relative)
position you programmed, `Pos:` is the measured firmware attitude.

### Example program

`examples/reveal-demo.g` is a short cinematic shot that exercises the whole
set — `G28` home, feedrate moves (`F`), dwell (`G4`), relative moves (`G91`),
rapid (`G0`), machine coords (`G53`), and a datum shift (`G92`):

```sh
cat examples/reveal-demo.g > /path/to/cranectld.fifo
```

## Safety

This drives real motors. Axis limits (`--max-pan/--max-tilt/--max-roll`,
defaults ±180/±90/±45) are enforced as hard rejections, but you are responsible
for what you point at and mount. If a move is interrupted (e.g. the daemon is
killed mid-command) the gimbal can be left holding position in "phone control";
a double-click of the gimbal's trigger button resets that.

## Protocol

14-byte frames with a CRC-16/XMODEM over a Quintic `0xFEE9` GATT service:
`…-9600` (write-without-response) and `…-9601` (notify). Point-to-position
motion runs a small state machine (`LOCATION_SET_POINT_POWERED` /
`LOCATION_POINT_CONTROLLER`) around the per-axis rotate-angle writes; live
attitude is read back from `*_ANGLE_R` (int16, /100 = degrees). See
`internal/protocol` and `internal/gimbal` for the working reference.

## License

ISC — see [LICENSE](LICENSE).
