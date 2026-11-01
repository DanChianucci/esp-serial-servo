# Changelog

## 0.1.0

First tagged release. The API is new; there is no earlier release to be compatible with.

### Added

- Generic `Servo` API with capabilities, plus the `RegisterAccess` and `SerialLink` interfaces.
- STS (Feetech STS series) family: register layer, typed helpers, and a generic `Servo` implementation.
- Explicit EEPROM lock, and `PersistScope` for scoped persistent writes.
- Readback check on register writes.
- Checksum option for serial packets.
- ESP-IDF port with deadlines on every blocking call.
- Host test suite and examples (generic, family, and raw register use).
- Transaction hardening: buffered receive bytes are discarded before every command, output flush is supported, and every wait is bounded by a timeout.
- Documentation: API reference, overview, and notes.

### Not included

- PWM backend.
- General-purpose operations: discovery, configuration of ID and baud rate, other operating modes, calibration, and multi-servo support.
- Servo families other than STS.

### Unverified on hardware

The following are implemented from the register map and vendor documentation, and have not been checked on a physical servo or on ESP-IDF hardware:

1. EEPROM lock semantics: with the lock off, writes are stored; with the lock on, writes are volatile across a power cycle.
2. Readback mask for `REG_PHASE`: bits 3 and 6 are excluded from the readback check based on community documentation.
3. Position mode (`OP_SERVO`) as the operating mode for the generic operations.
4. Protection setters: units and threshold meanings, including overload percent and voltage range, on each servo revision.
5. Telemetry decoding: load and current sign and magnitude encodings, and single-turn position.
6. `MULTI_TURN_POS` and `RETAINS_POSITION_UNPOWERED` are not declared because they are unverified.
7. ESP-IDF port timeouts: every blocking method returns by its deadline, tick conversion against the configured tick rate, and `uart_write_bytes()` hand-off time. The host tests do not build or run `lib/platform/esp_idf.cpp`.

### Breaking

- This is the first tagged release, and the API is new.
