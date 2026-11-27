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
G28                 ; home to 0/0
G1 Y-20 T2000       ; tilt down 20 deg in 2s
G4 P300             ; dwell 300 ms
G1 X30 Y30 T15000   ; coordinated pan + tilt-up over 15s
M2                  ; program end
```

### Supported G-code

Coordinates are **degrees**: `X` = pan/yaw, `Y` = tilt/pitch, `Z` = roll.

| Code | Meaning |
|------|---------|
| `G0 X_ Y_ Z_` | rapid move (minimum motion time) |
| `G1 X_ Y_ Z_ [F_] \| [T_]` | controlled move; `F` = feedrate (deg/min), `T` = explicit time (ms) |
| `G4 P<ms>` | dwell |
| `G28` | home to 0/0 |
| `G90` / `G91` | absolute (default) / relative |
| `G92 X_ Y_ Z_` | set datum without moving |
| `G20` / `G21` | accepted, ignored (units are always degrees) |
| `M2` / `M30` | program end |
| `;` … / `(...)` | comments |
| `N<line>` | line number (ignored) |

Anything else — unknown words, unsupported codes, or out-of-range axes — is
**rejected** with an `error:` and puts the daemon into an `Alarm` state until a
soft reset. Axes are never silently wrapped.

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
