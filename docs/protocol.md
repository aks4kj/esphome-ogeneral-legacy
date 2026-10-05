# Legacy O General / Fujitsu General wired-remote bus

Reverse-engineered on an **O General ARG25ALC** ducted indoor unit (cooling-only, R22, service manual dated 2003) with its wired weekly-timer wall remote. Everything below was measured on that one unit. Other units of the same generation probably behave the same way, but this has not been checked.

This is **not** the protocol that [`unreality/FujiHeatPump`](https://github.com/unreality/FujiHeatPump) / [`esphome-fujitsu`](https://github.com/FujiHeatPump/esphome-fujitsu) speak. Those projects use 8-byte addressed frames with inverted bytes, and a secondary controller registers in a separate window. This older bus has none of that.

## Physical layer

| | |
|---|---|
| Wires | 3-wire cable between the indoor unit (board connector CN17, terminal block "REMOTE CONTROL") and the wall remote: **1 red = +12 V, 2 white = signal, 3 black = GND** |
| Signal | single-wire, open-collector, idle high (~12 V), LIN-like |
| Interface | any LIN transceiver works (TJA1021 / TJA1020 modules tested). The MCP2025 used by older projects is EOL |
| Bit time | **≈1.985 ms from the remote, ≈2.07 ms from the AC** (≈500 baud; the two ends are not exactly the same) |
| Byte | start bit, 8 data bits **LSB first**, a **9th bit**, stop bit |
| 9th bit | **not parity**: `0` for the two header bytes of a frame, `1` for data bytes, `0` for the trailer bytes `7F`, `81` and `FD` |
| Data | bytes are sent as-is (no XOR / inversion) |

A plain UART set to `500 8O1` reads the bytes correctly. The parity flag is meaningless, but the data is right. For **sending**, the 9th bit must be exactly as above. If it is wrong, the AC silently ignores the frame, which is why every early attempt to send with a UART failed. The bridge therefore sends raw bit strings.

## Roles

**The wall remote is the bus master.** About every 240 ms it polls the indoor unit, and the unit answers:

```
remote → AC   80 D7 xx F7         poll
AC → remote   80 E7 xx B7 7F      status
              xx: E8 = ON, EC = OFF   (data bit 2 of byte 3: 0 = on, 1 = off)
```

The high nibble of byte 2 looks like an address/type: `D_` = remote, `E_` = indoor unit.

The remote owns the state. It re-asserts on/off in every poll, so a second device that merely transmits on the shared wire is overridden within 240 ms. Hence the man-in-the-middle approach below.

## Commands (key press)

A key press makes the remote send a command **instead of** its next poll(s).

**OFF:**
```
remote  80 DB FC
AC      80 EB FC
```

**ON / any state change** (mode, temperature, fan, energy save). The full state is sent every time, 6 frames in about 1.2 s:
```
remote  80 DD FC B4 B5 A0 81
AC      80 ED FC B4 B5 A0 81
remote  80 D7 E8 F5             (poll variant)
AC      80 ED FC B4 B5 A0 81
remote  80 DD EC B4 B5 A0 81
AC      80 ED EC B4 B5 A0 81
remote  80 D7 E8 F5
AC      80 ED EC B4 B5 A0 81
```

### B4: mode, temperature, fan

```
COOL      B4 = ((31 - T) << 3) | fan        T in °C (18..30)
FAN only  B4 = 0xF0 | (fan << 1)
fan:      AUTO = 2, LOW = 3, MID = 5, HIGH = 6
```

Verified examples: COOL 21 °C → `0x50 | fan` (AUTO `52`, LOW `53`, MID `55`, HIGH `56`); 22 °C LOW `4B`; 23 °C LOW `43`; FAN mode AUTO `F4`, LOW `F6`, MID `FA`, HIGH `FC`.

### B5: flags

```
FD = normal
BD = ENERGY SAVE on   (bit 0x40 cleared)
```

The ZONE key was not investigated. MASTER CONTROL only toggles COOL ↔ FAN on this cooling-only unit.

## What did NOT work (save yourself the time)

- **Transmitting as a second device on the shared wire** (correct bytes, correct timing, clean echo): the AC ignores it. The real remote re-asserts its state in every poll.
- **DIP switch on the remote's PCB.** On newer Fujitsu wired remotes, SW1-2 enables "dual remote / slave". On this one, switching it changed nothing: no third bus window appeared.
- **Another input on the indoor PCB.** CN17 is the only control connector. The custom-code jumpers JM1–JM3 are marked "NOT USED" in the service manual for this model.
- **ESP-IDF `esp_timer` with ISR dispatch at 250 µs** hung the ESP32-C6. Use task dispatch.

## How the bridge controls the AC

The white signal wire is cut at the remote. The ESP sits in between with two LIN transceivers and forwards every bit both ways in a GPIO ISR, adding only microseconds of delay.

- **State command from HA:** the next remote poll is **not** forwarded. The bridge sends the recorded `DD FC …` sequence with the new B4/B5 to the AC instead, starting on the same edge, so the AC answers inside the remote's expected window.
- **OFF from HA:** the remote's own `80 DB FC` frame is injected.
- **Keeping both ends consistent:** while an HA override is active, the on/off bit (bit 2 of byte 3) is forced bit by bit.
  - Towards the AC, the poll carries the HA state.
  - Towards the remote, the AC's status carries the remote's own belief. Otherwise the remote shows `EE` (communication error).
  - The forced level must be applied **for the whole bit, starting ~400 µs before the bit edge**: receivers sample earlier than the bit centre.
- Any real key press on the remote cancels the override, so the user always wins.

**Limitation:** the remote's LCD keeps showing its own last state. It does not learn about changes made from Home Assistant.

## Raw data

- `captures/bus_capture_2026-10-01.log` — remote-side log while pressing keys one by one. The `inv=[…]` column is an artefact of an early hypothesis that bytes were inverted; ignore it, `raw=[…]` is correct.
- `captures/key_press_map_2026-10-01.txt` — what was pressed and what changed on the remote's screen.
