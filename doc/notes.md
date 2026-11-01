# servokit notes

Design notes and open work for the library. For what exists today see the
[overview](overview.md) and the [API reference](api.md).

## Current structure

- `core/`: the family-neutral `Servo`, `RegisterAccess` and value types. No I/O.
- `serial/`: `SerialPort` (transport interface), `ServoController` (bounded
  command/response transaction), `ServoPacket` (frame buffer base class),
  `SerialLink`, and the platform-independent timing helpers (`Deadline`,
  `SyncMatcher`).
- `families/stseries/`: the STS protocol frame, register map, unit
  conversions and the `STSServo` driver. Servo documentation lives in
  `doc/STS3215`.
- `platform/`: `ESPSerialPort` for ESP-IDF.
- `pwm/`: reserved; contains no code.

STS frames use sync `FF FF`, little-endian words and a one-byte checksum.
Other protocol families are expected to reuse `ServoController` and
`SerialPort` and supply their own `ServoPacket` subclass, register map and
`Servo` implementation; the conformance suite in `tests/host/conformance` is
the check every family's `Servo` implementation runs.

## Open work

Each item is an open task, not a commitment to an order.

### More servo families

- SCS series (for example SC09, SC15): believed to use the STS frame layout
  with big-endian words and a different memory map. Unverified: needs the
  vendor documentation checked.
- Dynamixel protocol 1.0: same frame idea as STS; memory map differs per
  servo model.
- Dynamixel protocol 2.0: four-byte sync header (`FF FF FD 00`), a two-byte
  length field, and replies that carry both an instruction and an error byte.
  `ServoPacket` and `ServoController::read_response()` assume a one-byte
  length at a fixed index, so they need generalising first.
- LX series (for example LX-16A, LX-224): similar framing to STS with a
  different sync header; memory map and unit conventions to be checked.

### Protocol and register-map separation

- Split the protocol (framing, checksum, endianness, sync header) from the
  per-model memory map more cleanly, so a new model on an existing protocol
  needs only a register table. A sketch is a protocol base parameterised on
  sync header and endianness, with `STSeriesPacket` as one instance.

### PWM backend

- A `PwmOutput` port analogous to `SerialPort`, with platform
  implementations under `platform/`, and a `Servo` implementation on top of
  it. `pwm/README.md` is the placeholder.

### General-purpose operations

Feature flags exist for these, but no `Servo` method or family implements
them yet:

- relative moves, multi-turn targets, continuous rotation and PWM drive modes
  (`MotionMode` has only `ABSOLUTE_POSITION`);
- position gains, dead zone, angle limits and position offset (calibration);
- changing a servo's ID and baud rate;
- bus-level sync write, sync read, deferred write with action, and broadcast
  addressing;
- `clear_alarms()`, `reboot` and factory reset, and a speed readback;
- moving-state helpers such as waiting until a move ends.

### Platform support

- An Arduino platform port: `servokit.h` includes `platform/arduino.h` when
  `ARDUINO` or `SERVOKIT_IMPL_ARDUINO` is defined, but the header does not
  exist yet.
- `ESPSerialPort` is not built or run by the host tests; its timeout
  behaviour is not verified on ESP-IDF hardware.

### Language level

- The library needs C++23 (`std::span`, `std::expected`). Supporting older
  toolchains would need replacements for both (for example Boost's) and a
  check of the remaining C++20 and C++23 features in use.

### Known code issues

- `SerialPort` has no virtual destructor. Implementations must not be deleted
  through a `SerialPort` pointer.
- `STSeriesPacket::set_word()` writes at the start of the buffer, high byte
  first, while `get_word()` reads little endian from the given index. Only
  `set_sync()` uses it, with the symmetric pattern `FFFF`, so nothing is
  affected today; a general word setter would need the index and byte order
  fixed.
- `FakeSerialPort::read_sync()` restarts matching at zero on a mismatch, so it
  does not find a pattern that begins inside a partial match, unlike
  `SyncMatcher` and the ESP port.
- The register ranges of `stseries_defines.h` follow the ST3215 memory table
  where it is explicit; the table's own range column is inconsistent in places
  (for example the speed and the operation-mode ranges). Check against
  hardware before relying on a range.

### Hardware verification

Behaviour that follows the vendor documentation but has not been run on a
servo is listed in the
[hardware-verification list](api.md#hardware-verification-list).
