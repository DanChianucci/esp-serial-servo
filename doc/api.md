# servokit API reference

Back to the [overview](overview.md).

Everything is in namespace `servokit`. The **Tier** column uses the access
tiers from the overview: 1 = generic (`Servo&`), 2 = family (`STSServo`),
3 = raw registers, T = transport/infrastructure.

Contents:
[Common contract](#common-contract) |
[Units and conversions](#units-and-conversions) |
[Result](#result) |
[Timeouts and retries](#timeouts-and-retries) |
[Alarms](#alarms) |
[Capabilities and features](#capabilities-and-features) |
[Value types](#value-types) |
[Servo](#servo-generic-interface) |
[Write semantics](#write-semantics) |
[Checksum option](#checksum-option) |
[STSServo](#stsservo) |
[STS units header](#sts-units-header) |
[STS definitions](#sts-definitions) |
[Status codes and helpers](#status-codes-and-helpers) |
[SerialPort](#serialport) |
[ServoController](#servocontroller) |
[Packets](#packets) |
[ESPSerialPort](#espserialport) |
[Hardware-verification list](#hardware-verification-list)

## Common contract

These rules hold for every bus operation unless an entry says otherwise.

- **Blocking.** Calls are synchronous. A call returns when the transaction
  finishes or its time budget expires.
- **Finite time, no retries.** One call is one command/response transaction
  (plus one read-back when write verification is on). The library never
  retries and never loops waiting for a servo.
- **Errors.** Returned as a [`Result`](#result); nothing throws.
- **Thread safety.** Servo objects, controllers, ports and packets are not
  thread-safe, and neither is the bus. Serialize all calls per bus, including
  the checksum accessors, from one task or under a mutex. Pure value types and
  the functions in `sts_units.h` are safe from any thread.
- **Lifetimes.** Pointer and reference arguments are used only during the
  call and never retained. Output values are copies. Output parameters are
  written on `OK` only.
- **Ownership.** A servo borrows its controller (unless built from a
  `unique_ptr`), a controller borrows its port. The borrowed object must
  outlive the borrower.

## Units and conversions

Generic API (`core/servo_types.h`, `core/servo.h`):

| Quantity | Unit |
|---|---|
| angle | millidegrees (mdeg), `int32_t` |
| speed | mdeg/s |
| acceleration | mdeg/s^2 |
| current | mA |
| temperature | degrees C |
| voltage | mV |
| torque, load | permille (1000 = full scale) |
| time | milliseconds |

STS native units (`stseries/sts_units.h`, `STSServo`):

| Quantity | Native unit |
|---|---|
| position | 1 step; 4096 steps per revolution = 360000 mdeg (87.890625 mdeg/step) |
| speed | 1 step/s |
| acceleration | 100 steps/s^2 |
| torque, load | 0.1 % (the same number as permille) |
| current | 6.5 mA |
| voltage | 0.1 V |
| protection time | 10 ms |
| response delay | 2 us |

Conversions to native units round to nearest, halves away from zero, and never
saturate: the caller (or the typed helper) clamps to the register range. The
helpers are listed under [STS units header](#sts-units-header). STS target
positions are limited to +-30719 steps (`STSServo::MAX_TARGET_POSITION`); the
generic angle domain is the same range expressed in mdeg.

## Result

`enum class Result : uint8_t`, tier 1-3. Returned by every bus operation.

| Value | Meaning |
|---|---|
| `OK` | Completed. Output parameters are valid. |
| `TIMEOUT` | No complete response within the transaction budget. Also returned for a write to a servo in response mode `RSP_READ_PING` (the servo does not acknowledge writes; the write took effect). |
| `BAD_RESPONSE` | Framing, checksum, length or device-ID mismatch; the servo reported a non-zero error status (see `STSServo::last_status()`); or a register held a value that is not valid for its type (for example an operating mode above `OP_STEP`). |
| `UNSUPPORTED` | The servo/family does not provide the operation. Default of every optional `Servo` method. |
| `OUT_OF_RANGE` | Argument or register span outside the servo's domain or limits. Nothing is sent. |
| `PARAM` | Invalid argument (null pointer, empty span, broadcast or invalid ID, no controller, caller buffer too small). |
| `BUS` | Transport failure (read/write failure, driver error), not a servo fault. |
| `VERIFY_FAILED` | A verified write read back different from the bytes written. The write took effect. |

Mapping from transport status codes (`to_result()`):

| Status code | Result |
|---|---|
| `SRV_OK` (0) | `OK` |
| `SRV_TIMEOUT` | `TIMEOUT` |
| `SRV_READ_FAIL`, `SRV_WRITE_FAIL` | `BUS` |
| `SRV_BAD_FRAME`, `SRV_BAD_CHECKSUM`, `SRV_BAD_ID` | `BAD_RESPONSE` |
| `SRV_PARAM_ERR`, `SRV_OVERFLOW` | `PARAM` |
| `SRV_FAIL`, any unknown code | `BUS` |

Nothing maps to `UNSUPPORTED` or `OUT_OF_RANGE`; those come from the servo
layer.

## Timeouts and retries

- Each `ServoController` has a **transaction budget**: the total time one
  `command_response()` (send plus receive) may take. It is always finite and
  positive. It defaults to `tx_timeout + rx_timeout` from the constructor
  (saturating); if that sum is not positive it is
  `ServoController::FALLBACK_TRANSACTION_TIMEOUT` (100 ms). Change it with
  `set_transaction_timeout()`, read it with `transaction_timeout()`.
- The send step is clamped to the budget; the response step gets only the time
  left in the budget, so a slow send shortens the receive window.
- When the budget is spent the call returns `TIMEOUT`.
- A transaction that needs a read-back (write verification) or two frames
  (`hold()`, `prepare()`, `set_multiturn_feedback()`) takes one budget per
  frame.
- There are no retries anywhere. To retry, the caller repeats the call a
  count of its own choosing.
- Bytes buffered on the receive side are discarded before every command, so a
  stale reply is never taken for the next response.

## Alarms

`enum class Alarm : uint16_t` (bit values): `OVER_VOLTAGE` (1<<0),
`UNDER_VOLTAGE`, `OVER_TEMPERATURE`, `OVER_CURRENT`, `OVERLOAD`, `STALLED`,
`ENCODER`, `ANGLE_LIMIT`, `COMMUNICATION`, `UNLOADED`, `OTHER` (1<<10).

`AlarmSet` is a bitmask of `Alarm` values (tier 1, value type, thread-safe by
value):

| Member | Contract |
|---|---|
| `bits_t` | Alias of `uint16_t`, the mask type. |
| `AlarmSet()` | Empty set. |
| `AlarmSet(Alarm)` | Implicit; a set with one alarm. |
| `from_bits(bits)` | Static; builds a set from raw bits. |
| `bits()` | Raw `uint16_t` mask. |
| `none()`, `any()` | Empty / non-empty. |
| `has(Alarm)` | True if that alarm is set. |
| `has_all(AlarmSet)`, `has_any(AlarmSet)` | All / at least one of the given alarms are set. |
| `set(Alarm)`, `clear(Alarm)` | Mutate and return `*this`. |
| `operator\|`, `operator&`, `operator\|=`, `operator==` | Bitwise combine/compare. `Alarm \| Alarm` also yields an `AlarmSet`. |
| `name(Alarm)` | Static; upper-case name of one alarm, `"UNKNOWN"` if invalid. Static storage. |
| `to_text(buf, len)` | Writes names joined by `\|`, `"NONE"` for an empty set; NUL-terminated and truncated to fit; returns the length written (0 if `buf` is null or `len` is 0). No heap, non-blocking. |

### STS status to generic alarms

The STS status byte (`REG_STATUS` and the status byte of every reply) has one
bit per fault. `STS::units::alarms_from_native()` maps it through
`STS::units::ALARM_TABLE`:

| STS bit | Constant | Generic alarms |
|---|---|---|
| 0 | `STAT_VOLTAGE` | `OVER_VOLTAGE` and `UNDER_VOLTAGE` (the servo does not say which) |
| 1 | `STAT_SENSOR` | `ENCODER` |
| 2 | `STAT_TEMPERATURE` | `OVER_TEMPERATURE` |
| 3 | `STAT_CURRENT` | `OVER_CURRENT` |
| 4 | `STAT_ANGLE` | `ANGLE_LIMIT` |
| 5 | `STAT_OVERLOAD` | `OVERLOAD` |
| 6, 7 | (no table row) | `OTHER` |

Every set bit is reported; none is dropped. `STALLED`, `COMMUNICATION` and
`UNLOADED` are never reported by STS (see `reportable_alarms()`). In the other
direction, `native_from_alarms()` builds a native mask (used for the unload
mask) and fails if the set contains an alarm without a native bit.

`read_telemetry()` and `read_alarms()` combine the register status with the
status byte of the reply, so a servo that reports a fault in the reply still
returns its data.

## Capabilities and features

`Feature` (`enum class : uint64_t`) names an operation or property a servo
performs itself. `FeatureSet` is the bitmask (`bits_t` is `uint64_t`; `from_bits`, `bits`, `none`,
`has`, `has_all`, `has_any`, `set`, `clear`, `operator|`, `operator&`,
`operator|=`, `operator==`; `Feature | Feature` yields a `FeatureSet`).

**Features are never emulated.** A flag is set only if the servo does the
thing; a method whose feature is absent returns `UNSUPPORTED`. Check
`capabilities().features` before depending on an optional operation.

Features: `ABSOLUTE_POSITION`, `MULTI_TURN_POS`, `RELATIVE_POSITION`,
`CONTINUOUS_ROTATION`, `PWM_DRIVE`, `SET_SPEED`, `SET_ACCEL`, `TORQUE_LIMIT`,
`CURRENT_LIMIT`, `START_TORQUE`, `TORQUE_ENABLE`, `HOLD`, `POSITION_GAINS`,
`DEADZONE`, `ANGLE_LIMITS`, `POSITION_OFFSET`, `READ_POSITION`, `READ_SPEED`,
`READ_CURRENT`, `READ_LOAD`, `READ_VOLTAGE`, `READ_TEMPERATURE`,
`READ_MOVING`, `READ_ALARMS`, `PROTECT_CURRENT`, `PROTECT_OVERLOAD`,
`PROTECT_TEMP`, `PROTECT_VOLTAGE`, `PROTECT_RELEASE_ON_ALARM`, `PING`,
`READ_IDENTITY`, `SET_ID`, `SET_BAUD`, `SYNC_WRITE`, `SYNC_READ`,
`ASYNC_WRITE`, `BROADCAST`, `RAW_REGISTERS`, `RETAINS_POSITION_UNPOWERED`,
`PERSIST_CONFIG` (`set_persist()` controls persistence), `VERIFY_WRITES`
(`set_verify_writes()` is available).

`STSServo::capabilities()` declares: `ABSOLUTE_POSITION`, `SET_SPEED`,
`SET_ACCEL`, `TORQUE_LIMIT`, `START_TORQUE`, `TORQUE_ENABLE`, `HOLD`,
`READ_POSITION`, `READ_CURRENT`, `READ_LOAD`, `READ_VOLTAGE`,
`READ_TEMPERATURE`, `READ_MOVING`, `READ_ALARMS`, `PROTECT_CURRENT`,
`PROTECT_TEMP`, `PROTECT_RELEASE_ON_ALARM`, `PING`, `READ_IDENTITY`,
`RAW_REGISTERS`, `PERSIST_CONFIG`, `VERIFY_WRITES`. It does not declare
`MULTI_TURN_POS` or `RETAINS_POSITION_UNPOWERED` (unverified on hardware), nor
any feature without an implementation.

`Capabilities` holds `features` and `limits`. `Limits` values are valid only
if the matching feature is set, otherwise 0. STS values: angle
-30719..+30719 steps converted to mdeg, `resolution_mdeg` 88 (87.890625
rounded), maximum speed 3400 steps/s and acceleration 254 units converted to
mdeg/s and mdeg/s², `max_torque_permille` 1000, current range +-the largest 15-bit current
feedback in mA (the reportable range; the settable stall threshold is
narrower), `reportable_alarms` from `reportable_alarms()`.

## Value types

All are plain copyable values in `core/servo_types.h`; no heap, no
references; nothing blocks.

| Type | Fields |
|---|---|
| `Limits` | `angle_min_mdeg`, `angle_max_mdeg`, `resolution_mdeg`, `max_speed_mdeg_s`, `max_accel_mdeg_s2`, `max_torque_permille`, `current_min_ma`, `current_max_ma`, `reportable_alarms` |
| `Capabilities` | `features`, `limits` |
| `Identity` | `family[16]` (NUL-terminated, owned, e.g. `"STS"`), `model_major`, `model_minor`, `firmware_major`, `firmware_minor`, `device_id` |
| `Telemetry` | `position_mdeg`, `current_ma`, `temperature_c`, `alarms`; optional `load_valid`/`load_permille`, `voltage_valid`/`voltage_mv`, `moving_valid`/`moving` (read an optional field only when its flag is true) |
| `Protection` | `stall_current_ma`, `stall_time_ms`, `max_temperature_c`, `release_on_alarm` |
| `Profile` | `speed_mdeg_s`, `accel_mdeg_s2`, `torque_permille`, `start_torque_permille` (ignored without `START_TORQUE`) |

## Servo (generic interface)

`class Servo` in `core/servo.h`. Tier 1. Only `move_to` and `capabilities`
are pure; every other method defaults to `UNSUPPORTED` (or the stated default
value). The destructor is virtual. The Servo does not own the bus.

| Method | Contract |
|---|---|
| `move_to(angle_mdeg)` | Absolute move. `OUT_OF_RANGE` outside the limits' angle domain (nothing sent). |
| `capabilities()` | Const; does not touch the bus. Static per model; an implementation may make it depend on a prior `read_identity()`. |
| `ping()` | One transaction, no retries. |
| `read_identity(Identity*)` | Family, model, firmware, device ID. |
| `prepare()` | Puts the servo into absolute-target mode over the full domain. |
| `set_protection(const Protection&)` | Programs servo-side protection thresholds; the servo enforces them itself. |
| `set_profile(const Profile&)` | Speed, acceleration, torque for following moves; clamped to the limits. |
| `hold()` | Stops and holds position (no coast). |
| `set_torque(bool)` | Enables or disables motor torque. |
| `set_persist(bool)` | `true`: configuration writes made afterwards survive a power cycle; `false`: they take effect but are not kept. Applies until changed again; the library keeps no state. |
| `read_telemetry(Telemetry*)` | One atomic sample, filled on `OK` only. |
| `read_alarms(AlarmSet*)` | Active alarms. |
| `clear_alarms()` | Clears latched alarms where supported. |
| `read_raw(addr, data, len)` | Tier 3. Family-defined address, no conversion; `data` is a caller buffer of at least `len` bytes. |
| `write_raw(addr, data, len)` | Tier 3. `data` is read during the call only. |
| `set_verify_checksum(bool)`, `verify_checksum()`, `checksum_errors()` | See [checksum option](#checksum-option). |
| `set_verify_writes(bool)`, `verify_writes()` | See [write semantics](#write-semantics). Defaults: `UNSUPPORTED`, `false`. |

## Write semantics

**Plain writes.** Every write, in every tier, is sent as given. There is no
implicit read-compare-write and no hidden read. A caller that wants to avoid
a write reads first. The only helper that reads before writing is
`STSServo::set_multiturn_feedback()` (it must preserve the other bits of
`REG_PHASE`), and `hold()` reads the position it commands. Each typed setter
performs exactly one register write (plus the read-back when verification is
on). `set_protection()` and `set_profile()` perform several plain writes in a
fixed order and stop at the first failure.

**EEPROM persistence is explicit.** Whether a configuration write survives a
power cycle is decided by `REG_EEPROM_LOCK` on the servo: lock off (0) stores
the write; lock on (1) lets it take effect but it is not kept across a power
cycle (per the register map; see the [hardware list](#hardware-verification-list)).
No helper changes the lock by itself and nothing relocks automatically. The
caller sequences:

```cpp
sts.unlock_eeprom();            // or servo.set_persist(true)
sts.set_overtemp_threshold(70); // one or more configuration writes
sts.lock_eeprom();              // or servo.set_persist(false)
```

A failed write leaves the lock as it was. `STSServo::set_persist(bool)` calls
`unlock_eeprom()` for `true` and `lock_eeprom()` for `false` and keeps no
state. The lock register is read with `get_eeprom_locked()`.

**Write verification** (`set_verify_writes(true)`, off by default). After a
successful `write_register()` of a span, one read of the same span is sent and
each byte is compared with the byte written through that byte's compare mask.
This happens in `write_register()`, so `write_u8`, `write_u16`, `write_raw`,
every typed helper and every generic operation are covered alike.

- The read is addressed to the written ID, except that a write covering
  `REG_DEVICE_ID` reads from the newly written ID; if that ID is above
  `STS::MAX_ID` (0xFC) nothing is read and the write returns `OK`.
- Exactly one extra read frame per write that has at least one compared byte;
  none for a span of only volatile or unlisted bytes; none when verification is
  off.
- A write error is returned as is and nothing is read. A failed read-back
  returns that read's own error (the write took effect). A difference in a
  compared bit returns `VERIFY_FAILED` (the write took effect and the servo
  holds a different value).

Register classes (`STSServo::verify_rule(addr)`, a pure function; a register
absent from the table, such as every read-only register, a gap or an address
of 256 or more, is `VOLATILE` with mask 0 and is never compared). "Volatile"
here means the servo may change the value on its own, not "lost at power-off".

| Address | Register | Class |
|---|---|---|
| 0x05 | `REG_DEVICE_ID` | STABLE |
| 0x06 | `REG_BAUD_RATE` | VOLATILE (the link rate may change at once) |
| 0x07 | `REG_RESPONSE_DELAY` | STABLE |
| 0x08 | `REG_RESPONSE_MODE` | STABLE |
| 0x09, 0x0B | `REG_MIN_ANGLE_THRESH`, `REG_MAX_ANGLE_THRESH` (2 bytes each) | STABLE |
| 0x0D, 0x0E, 0x0F | `REG_OVERTEMP_THRESH`, `REG_OVERVOLT_THRESH`, `REG_UNDERVOLT_THRESH` | STABLE |
| 0x10 | `REG_CONTROL_TORQUE_NV` (2) | STABLE |
| 0x12 | `REG_PHASE` | STABLE_MASKED with `PHASE_VERIFY_MASK` (bits 3 and 6 excluded) |
| 0x13, 0x14 | `REG_UNLOAD_ALARM_MASK`, `REG_LED_ALARM_MASK` | STABLE |
| 0x15, 0x16, 0x17 | `REG_POS_COEF_P`, `_D`, `_I` | STABLE |
| 0x18 | `REG_START_TORQUE` (2) | STABLE |
| 0x1A, 0x1B | `REG_CW_DEADZONE`, `REG_CCW_DEADZONE` | STABLE |
| 0x1C | `REG_OVERCURR_THRESH` (2) | STABLE |
| 0x1E | `REG_ANGLE_RESOLUTION` | STABLE |
| 0x1F | `REG_POS_OFFSET` (2) | STABLE |
| 0x21..0x27 | `REG_OPERATION_MODE`, `REG_UNLOAD_CTRL_TORQUE`, `REG_OVERLOAD_TIME`, `REG_OVERLOAD_THRESH`, `REG_SPEED_COEF_P`, `REG_OVERCURR_TIME`, `REG_SPEED_COEF_I` | STABLE |
| 0x28 | `REG_SERVO_EN` | VOLATILE (a protection trip can clear it; the calibration value is a command). Torque-enable writes are never read back. |
| 0x29, 0x2A, 0x2C, 0x2E, 0x30 | `REG_CTRL_ACCEL`, `REG_CTRL_POS`, `REG_CTRL_PWM`, `REG_CTRL_SPEED`, `REG_CTRL_TORQUE` | VOLATILE |
| 0x37 | `REG_EEPROM_LOCK` | STABLE |

`VerifyClass` has the values `STABLE` (compared in full), `STABLE_MASKED`
(compared in `VerifyRule::mask` only) and `VOLATILE` (never compared).
`VerifyRule` holds `kind` and `mask` (0xFF, a subset, or 0).

## Checksum option

STS frames end with `~(id + length + instruction/status + params/data)`, low
byte. Transmitted frames always carry a correct checksum. Verification of
**received** frames is on by default and can be switched:

- `Servo::set_verify_checksum(bool)` returns `UNSUPPORTED` if the protocol has
  no checksum; `STSServo` forwards to its controller (`PARAM` if it has none).
- With verification off, a reply with a wrong checksum is still accepted if its
  frame (sync and length) and device ID validate.
- `checksum_errors()` counts replies with a bad checksum, whether or not
  verification is on. `Servo` default: 0. `verify_checksum()` default: `true`.
- The accessors are not thread-safe (see the common contract).

## STSServo

`class STSServo : public Servo` in `stseries/sts_servo.h`. Not thread-safe;
every call is one blocking transaction as described in the
[common contract](#common-contract).

STS-specific rules:

- **Target.** Methods without an `id` parameter act on `device_id()`. A
  broadcast (0xFE) or higher ID makes every method that uses the bus return
  `PARAM`; a null controller does too.
- **Words** are little endian (low byte at the lower register address).
- **Status.** A reply whose status byte is non-zero makes the call return
  `BAD_RESPONSE`; the byte is available from `last_status()`. The exceptions
  are `read_telemetry()`, `read_alarms()` and the position read of `hold()`,
  which accept it and report the alarm.
- **Response mode.** A servo whose `REG_RESPONSE_MODE` is `RSP_READ_PING` does
  not answer writes; write calls then return `TIMEOUT` although the write took
  effect.
- **Ranges.** Typed helpers enforce only hardware ranges. An out-of-range
  argument returns `OUT_OF_RANGE` and sends nothing.

### Construction and identity

| Member | Tier | Contract |
|---|---|---|
| `STSServo(ServoController*, id = 1)` | 2 | Borrows the controller (must outlive the servo). A null controller makes every bus call return `PARAM`. |
| `STSServo(std::unique_ptr<ServoController>, id = 1)` | 2 | Takes ownership of the controller; its `SerialPort` must outlive the servo. |
| `device_id()` | 2 | ID used by methods without an `id` parameter. No bus access. |
| `set_device_id(id)` | 2 | Changes only the ID this object addresses. Sends nothing; it does not reprogram the servo. |
| `last_status()` | 2 | Status byte of the most recent valid reply (bitmask of `err_code_t`, 0 when healthy). Reset at the start of every call that uses the bus; stays 0 if no valid reply arrived. |

### Constants

| Constant | Value / meaning |
|---|---|
| `MAX_TARGET_POSITION` | 30719; target magnitude, steps |
| `MAX_SPEED` | 3400; steps/s |
| `MAX_ACCELERATION` | 254; units of 100 steps/s^2 |
| `MAX_TORQUE` | 1000; units of 0.1 % |
| `MAX_RESPONSE_DELAY` | 254; units of 2 us |
| `MAX_TEMPERATURE_LIMIT` | 100; degrees C |
| `MAX_VOLTAGE_LIMIT` | 254; units of 0.1 V |
| `MAX_CURRENT_LIMIT` | 511; units of 6.5 mA |
| `MAX_PROTECTION_TIME` | 254; units of 10 ms |
| `MAX_OVERLOAD_LIMIT` | 100; percent of maximum torque |
| `ALARM_MASK_BITS` | 0x3F; defined `err_code_t` bits |
| `PHASE_MULTITURN_BIT` | 1 << 4; `REG_PHASE` multi-turn feedback bit |
| `PHASE_VERIFY_MASK` | `REG_PHASE` bits compared by write verification (all except bits 3 and 6) |
| `REGISTER_SPACE` | 256; the control table is addressed by one byte |
| `MAX_SPAN` | 64; longest register span of one read or write |

### Register access (tier 3)

| Method | Contract |
|---|---|
| `ping(id)` | Unicast ID 0..0xFD. 0xFE (broadcast) and above returns `PARAM`. |
| `read_register(id, addr, data, len)` | `len` 1..`MAX_SPAN` and `addr + len <= REGISTER_SPACE`. `PARAM` for a null buffer or `len == 0`; `OUT_OF_RANGE` for a longer span or a span past the table. `data` is untouched on failure. |
| `write_register(id, addr, data, len)` | Same limits. Written as given, no EEPROM handling; verified when `verify_writes()` is on. |
| `read_u8`, `read_u16` | One byte / one little-endian word; `PARAM` for a null output. |
| `write_u8`, `write_u16` | One byte / one little-endian word; as `write_register`. |
| `verify_rule(addr)` | Static, pure, no bus. See [write semantics](#write-semantics). |

### Typed helpers (tier 2)

Each does one register write (or read) on `device_id()`; "SRAM" registers
reset at power-up, "EEPROM" registers follow the lock.

| Method | Contract |
|---|---|
| `get_operating_mode(op_mode_t*)` | `BAD_RESPONSE` if the stored value is not a mode. |
| `set_operating_mode(op_mode_t)` | `OP_SERVO`..`OP_STEP`; others `OUT_OF_RANGE`. Decides how `REG_CTRL_*` are read. |
| `get_response_mode(rsp_mode_t*)`, `set_response_mode(rsp_mode_t)` | `RSP_READ_PING` stops write acknowledgements. |
| `get_response_delay(uint8_t*)`, `set_response_delay(delay_2us)` | Units of 2 us, 0..254. |
| `get_servo_enabled(bool*)`, `set_servo_enabled(bool)` | Torque output on/off (SRAM). Never writes the mid-point calibration value. `get_servo_enabled` returns `BAD_RESPONSE` if the register holds neither 0 nor 1. |
| `set_acceleration(accel)` | 0..254, units of 100 steps/s^2 (SRAM). |
| `set_speed(steps_per_s)` | 0..3400, unsigned magnitude for position moves (SRAM). |
| `set_signed_speed(steps_per_s)` | -3400..3400, bit 15 = reverse, for constant-speed mode (SRAM). |
| `set_torque_limit(torque)` | 0..1000, 0.1 % (SRAM); reloaded from the stored maximum at power-up. |
| `set_start_torque(torque)` | 0..1000, 0.1 % (EEPROM region). |
| `set_target_position(steps)` | -30719..30719 steps, sign-magnitude, 4096 per revolution (SRAM). |
| `get_eeprom_locked(bool*)` | `BAD_RESPONSE` if neither 0 nor 1. |
| `unlock_eeprom()` | Writes lock = 0: EEPROM-region writes are stored. |
| `lock_eeprom()` | Writes lock = 1: EEPROM-region writes take effect but are not saved across a power cycle. |
| `get_multiturn_feedback(bool*)` | Reads the `REG_PHASE` multi-turn bit. |
| `set_multiturn_feedback(bool)` | Reads `REG_PHASE`, then writes it with only that bit changed (two frames); a failed read sends no write. |
| `set_overtemp_threshold(celsius)` | 0..100 C. |
| `set_overvoltage_threshold(decivolts)`, `set_undervoltage_threshold(decivolts)` | 0..254, 0.1 V. |
| `set_overcurrent_threshold(units)` | 0..511, 6.5 mA (16-bit register). |
| `set_overcurrent_time(units_10ms)`, `set_overload_time(units_10ms)` | 0..254, 10 ms. |
| `set_overload_threshold(percent)` | 0..100 percent of maximum torque. |
| `set_led_alarm_mask(mask)`, `set_unload_alarm_mask(mask)` | Bitmask of `err_code_t`; bits above `ALARM_MASK_BITS` are `OUT_OF_RANGE`. |

### Servo overrides (tier 1 on STS)

| Method | Contract |
|---|---|
| `ping()` | `ping(device_id())`. |
| `read_raw`, `write_raw` | `read_register` / `write_register` on `device_id()`. |
| `set_persist(bool)` | `unlock_eeprom()` / `lock_eeprom()`; keeps no state. |
| `set_verify_checksum`, `verify_checksum`, `checksum_errors` | Forward to the controller. |
| `set_verify_writes(bool)` | Always `OK`; touches no bus. `verify_writes()` returns the setting. |
| `move_to(angle_mdeg)` | Converts to steps (nearest) and writes `REG_CTRL_POS` once. `OUT_OF_RANGE` outside the limits' domain (nothing sent). |
| `capabilities()` | Fixed STS description; no bus access; independent of `read_identity()`. |
| `read_identity(Identity*)` | One read of the six bytes `0x00..0x05`; the device ID is the value stored in the servo. |
| `prepare()` | Writes `REG_RESPONSE_MODE = RSP_ALL`, then `REG_OPERATION_MODE = OP_SERVO`. Two plain writes, no reads. From `RSP_READ_PING` the first write gets no reply: it takes effect but `TIMEOUT` is returned and the second write is not sent; call again. |
| `set_protection(const Protection&)` | Writes overcurrent threshold (6.5 mA units), overcurrent time (10 ms units), over-temperature threshold and the unload alarm mask, in that order; stops at the first failure. All values are validated first: a value that does not fit its register, or an alarm without a native bit (`STALLED`, `COMMUNICATION`, `UNLOADED`, `OTHER`), returns `OUT_OF_RANGE` and sends nothing. Values are never clamped. `OVER_VOLTAGE` and `UNDER_VOLTAGE` share one native bit, so requesting either releases torque on both. Overload and voltage thresholds are not written. |
| `set_profile(const Profile&)` | Writes torque limit, start torque, acceleration, speed, in that order, one plain write each; values are clamped to `capabilities().limits` and converted. Start torque is an EEPROM-region register and follows the lock. Stops at the first failure. |
| `hold()` | Reads `REG_POSITION`, then writes it as the target (two transactions). Torque must already be enabled. A position outside the domain returns `OUT_OF_RANGE` and writes nothing. A servo reporting an alarm still yields its position. Never reports `OK` for a write that was not acknowledged. |
| `hold_failed_step()` | Which sub-operation failed in the latest `hold()`: `HoldStep::NONE`, `READ_POSITION`, `POSITION_OUT_OF_DOMAIN` or `WRITE_TARGET`. |
| `set_torque(bool)` | `set_servo_enabled`. |
| `read_telemetry(Telemetry*)` | One 15-byte read of `REG_POSITION..REG_CURRENT`. Fails as a whole on a transport error, short reply, or an undecodable field (load outside +-1000 permille, moving flag other than 0/1) with `BAD_RESPONSE`. Position is single-turn. Load, voltage and moving are always valid on STS. |
| `read_alarms(AlarmSet*)` | Reads `REG_STATUS` and combines it with the reply status byte through `alarms_from_native()`. |

Not provided by STS (default `UNSUPPORTED`): `clear_alarms()`.

## STS units header

`stseries/sts_units.h`, namespace `STS::units`. Header-only pure functions
(any thread). Conversions round to nearest, halves away from zero, and do not
saturate.

| Name | Contract |
|---|---|
| `STEPS_PER_REV`, `MDEG_PER_REV` | 4096 and 360000. |
| `CURRENT_UNIT_X2_MA`, `VOLTAGE_UNIT_MV`, `TIME_UNIT_MS` | 13 (6.5 mA in half milliamps), 100, 10. |
| `CURRENT_FEEDBACK_MAX_UNITS` | 0x7FFF; largest current feedback magnitude. |
| `round_div(num, den)` | Signed division, nearest, halves away from zero; `den > 0`. |
| `steps_to_mdeg`, `mdeg_to_steps` | Position. The result of `mdeg_to_steps` may lie outside any register range. |
| `mdeg_s_to_steps_s`, `steps_s_to_mdeg_s` | Speed. |
| `mdeg_s2_to_accel_units`, `accel_units_to_mdeg_s2` | Acceleration, 100 steps/s^2 units. |
| `current_units_to_ma`, `ma_to_current_units` | 6.5 mA units. |
| `voltage_units_to_mv` | 0.1 V units to mV. |
| `ms_to_time_units` | Milliseconds to 10 ms units. |
| `decode_sign_magnitude(raw, sign_bit)` | Sign bit selects the sign, lower bits the magnitude; negative zero is 0; bits above the sign bit are the caller's to check. |
| `decode_position_steps`, `decode_current_units` | Bit 15 sign. |
| `decode_load_permille(raw, int16_t*)` | Bit 10 sign, magnitude 0..1000. Returns false (output untouched) if a bit above bit 10 is set or the magnitude exceeds 1000. |
| `AlarmMapRow`, `ALARM_TABLE`, `MAPPED_STATUS_BITS` | Native bit to generic alarms; 0x3F. See [Alarms](#alarms). |
| `alarms_from_native(status)` | Maps a status byte; unmapped set bits report `OTHER`. |
| `reportable_alarms()` | Every alarm with a table row, plus `OTHER`. |
| `native_from_alarms(AlarmSet, uint8_t*)` | Native mask; false if an alarm has no native bit (output untouched). |

## STS definitions

`stseries/stseries_defines.h`, namespace `STS`.

- Protocol constants: `LIB_FW_VERSION`, `SYNC_PATTERN` (0xFFFF), `MAX_ID`
  (0xFC, the highest usable device ID), `BROADCAST_ID` (0xFE, no response),
  frame index constants `SYNC_IDX`, `DEVID_IDX`, `LENGTH_IDX`, `CODE_IDX`,
  `DATA_IDX`, size constants `NUM_SYNC_BYTES`, `NUM_DEVID_BYTES`,
  `NUM_LENGTH_BYTES`, `NUM_CODE_BYTES`, `NUM_CHKSUM_BYTES`, `NUM_META_BYTES`.
- `cmd_e` instructions: `CMD_PING`, `CMD_READ`, `CMD_WRITE`, `CMD_REG_WRITE`,
  `CMD_ACTION`, `CMD_RESET`, `CMD_SYNC_READ`, `CMD_SYNC_WRITE`. `STSServo`
  uses ping, read and write only.
- `err_code_t` (alias `sts_err_code_t`): `STAT_OK`, `STAT_VOLTAGE`,
  `STAT_SENSOR`, `STAT_TEMPERATURE`, `STAT_CURRENT`, `STAT_ANGLE`,
  `STAT_OVERLOAD`; also the layout of `REG_LED_ALARM_MASK` and
  `REG_UNLOAD_ALARM_MASK`.
- `rsp_mode_t`: `RSP_READ_PING`, `RSP_ALL`.
- `op_mode_t`: `OP_SERVO`, `OP_SPEED`, `OP_PWM`, `OP_STEP`.
- `baud_rate_e`: `BAUD_1000000`, `BAUD_500000`, `BAUD_250000`, `BAUD_128000`,
  `BAUD_115200`, `BAUD_76800`, `BAUD_57600`, `BAUD_38400`.
- `servo_en_t`: `SERVO_DISABLE`, `SERVO_ENABLE`. `eeprom_lock_t`:
  `EEPROM_UNLOCKED`, `EEPROM_LOCKED`. `move_flag_t`: `SERVO_STOPPED`,
  `SERVO_MOVING`.
- `reg_addr_e` register map (address, size, access, memory in the header
  comments): `REG_VER_MAJ`, `REG_VER_SUB`, `REG_MODEL_MAJ`, `REG_MODEL_SUB`,
  `REG_DEVICE_ID`, `REG_BAUD_RATE`, `REG_RESPONSE_DELAY`, `REG_RESPONSE_MODE`,
  `REG_CONTROL_TORQUE_NV`, `REG_PHASE`, `REG_CW_DEADZONE`, `REG_CCW_DEADZONE`,
  `REG_ANGLE_RESOLUTION`, `REG_POS_OFFSET`, `REG_POS_COEF_P`, `REG_POS_COEF_D`,
  `REG_POS_COEF_I`, `REG_SPEED_COEF_P`, `REG_SPEED_COEF_I`,
  `REG_UNLOAD_ALARM_MASK`, `REG_LED_ALARM_MASK`, `REG_MIN_ANGLE_THRESH`,
  `REG_MAX_ANGLE_THRESH`, `REG_OVERTEMP_THRESH`, `REG_OVERVOLT_THRESH`,
  `REG_UNDERVOLT_THRESH`, `REG_OVERCURR_THRESH`, `REG_OVERCURR_TIME`,
  `REG_OVERLOAD_THRESH`, `REG_OVERLOAD_TIME`, `REG_START_TORQUE`,
  `REG_OPERATION_MODE`, `REG_UNLOAD_CTRL_TORQUE`, `REG_SERVO_EN`,
  `REG_CTRL_ACCEL`, `REG_CTRL_POS`, `REG_CTRL_PWM`, `REG_CTRL_SPEED`,
  `REG_CTRL_TORQUE`, `REG_EEPROM_LOCK`, `REG_POSITION`, `REG_SPEED`,
  `REG_LOAD`, `REG_VOLTAGE`, `REG_TEMPERATURE`, `REG_ASYNC_WRITE_FLAG`,
  `REG_STATUS`, `REG_MOVE_FLAG`, `REG_CURRENT`.

## Status codes and helpers

`core/servo_utils.h`. Transport-level types used by `SerialPort`,
`ServoController` and the packets. Pure, non-blocking, thread-safe.

| Name | Contract |
|---|---|
| `buffer_t`, `buffer_const_t` | `std::span<uint8_t>` / `std::span<const uint8_t>`; non-owning views. |
| `timeout_ctrl_t`, `timeout_duration_t` | `steady_clock` time point and duration. |
| `srv_stat_t` | `std::unexpected<int>`: a status code. |
| `SRV_OK` (0), `SRV_FAIL` (-1), `SRV_TIMEOUT` (-2), `SRV_OVERFLOW` (-3), `SRV_PARAM_ERR` (-4), `SRV_READ_FAIL` (-5), `SRV_WRITE_FAIL` (-6), `SRV_BAD_FRAME` (-7), `SRV_BAD_CHECKSUM` (-8), `SRV_BAD_ID` (-9) | Status constants. Map to `Result` as in the [Result](#result) table. `SRV_OK` is the zero status; compare with `get_status()`. |
| `srv_result_t` | `std::expected<int, int>`: a value (for example a byte count) or a status code. |
| `is_value(r)`, `is_status(r)` | Holds a value / holds a status. |
| `get_status(r)` | `SRV_OK` if `r` holds a value, otherwise its status. |
| `get_value(r, default = 0)` | The value, or `default` if `r` holds a status. |
| `to_result(int)`, `to_result(srv_stat_t)`, `to_result(srv_result_t)` | Map to `Result` per the table above (declared in `servo_controller.h`). A held value maps to `OK`. |

## SerialPort

`class SerialPort` in `serial/serial_port.h`: the transport interface a
platform implements. Tier T. All methods are pure virtual; every one blocks
for at most the given timeout (durations are `std::chrono`, not ticks).
`LOG_TAG` is a constant string `"SerialPort"`. Implementations are not
thread-safe.

| Method | Contract |
|---|---|
| `enable_rx(bool)` | Enables/disables the receiver. `SRV_OK`, else `SRV_PARAM_ERR`. |
| `discard_input()` | Drops all buffered receive data. `SRV_OK`, else `SRV_PARAM_ERR`. |
| `flush_output(timeout)` | Blocks until all transmit data is sent. `SRV_OK` or `SRV_TIMEOUT`. |
| `read_sync(pattern, timeout)` | Consumes bytes up to and including `pattern`. `SRV_OK`, or `SRV_TIMEOUT` if not found within `timeout`. |
| `read_bytes(data, timeout)` | Fills `data` until full or `timeout`. Returns the byte count (may be short), or `SRV_READ_FAIL`. |
| `write_bytes(data, timeout)` | Writes `data`; `timeout` bounds the wait for transmission (0 = do not wait). Returns the byte count, or `SRV_WRITE_FAIL`, or `SRV_TIMEOUT`. Default timeout is the maximum duration. |

A scripted implementation for host use is `tests/host/fake_serial_port.h`
(`servokit_test::FakeSerialPort`): each write consumes one scripted reply,
reads never block and return the bytes buffered.

## ServoController

`class ServoController` in `serial/servo_controller.h`. Tier T. Runs one
command/response transaction over a `SerialPort` with a bounded time. Not
thread-safe. Constants: `DEF_TIMEOUT` (zero: "use the configured value"),
`LOG_TAG`, `FALLBACK_TRANSACTION_TIMEOUT` (100 ms).

| Member | Contract |
|---|---|
| `ServoController(bus, tx_timeout, rx_timeout, auto_disable_rx = true)` | Borrows `bus` (must outlive the controller). `tx_timeout` bounds sending, `rx_timeout` bounds receiving. With `auto_disable_rx` the receiver is disabled while transmitting (half-duplex buses) and always re-enabled afterwards, even if the write failed. |
| `send_command(ServoPacket*, timeout)`, `send_command(buffer_const_t, timeout)` | Discards buffered input, then writes the bytes. Returns the byte count or a status code. |
| `sync_response(pattern, timeout)` | Waits for the sync pattern. `SRV_TIMEOUT` if not seen. |
| `read_response(ServoPacket*, timeout)` | Reads one frame into the packet within `timeout` (a deadline over the whole frame). `SRV_BAD_FRAME` for an inconsistent frame, `SRV_BAD_CHECKSUM` for a bad checksum when verification is on, `SRV_TIMEOUT` for a missing or truncated frame. |
| `read_response(buffer, len, timeout)` | Raw read of `len` bytes; `SRV_OVERFLOW` if `len` exceeds the buffer. Returns the count (may be short). |
| `command_response(cmd, rsp, cmd_timeout, rsp_timeout)` | Send, then read. The whole call is bounded by `transaction_timeout()`. `SRV_BAD_ID` if the reply carries a different device ID than the command. |
| `flush_rx()` | Discards buffered receive bytes; called before every command. |
| `transaction_timeout()`, `set_transaction_timeout(t)` | Budget, always positive; `SRV_PARAM_ERR` for a non-positive value. |
| `set_verify_checksum(bool)`, `verify_checksum()`, `checksum_errors()` | See [checksum option](#checksum-option). |
| `m_servo_bus`, `m_transmit_timeout`, `m_response_timeout`, `m_auto_disable_rx` | Public configuration members; changing them between calls is allowed, not during one. |

A timeout argument of `DEF_TIMEOUT` (zero) selects the configured value.

## Packets

`ServoPacket` (`serial/servo_packet.h`) is the abstract frame buffer;
`STSeriesPacket` (`stseries/stseries_packet.h`) implements the STS frame
`FF FF id length code data... checksum`. Tier T. Packets are views over a
caller-owned buffer (no heap); the buffer must outlive the packet; not
thread-safe. `STSeriesPacket` needs a buffer of at least `NUM_META_BYTES`
(6) bytes.

`ServoPacket`: `LOG_TAG`; constructor from `buffer_t` (the default
constructor is deleted); pure `sync_buffer()`, `data_buffer()`, `len_index()`,
`is_well_formed()`, `reset()`; `is_frame_valid()` (size, sync and length, not
checksum; defaults to `is_well_formed()`); `is_checksum_valid()` (default
true); `packet_device_id()` (default -1); `resize(size)` (`SRV_OVERFLOW` above
capacity; alters no bytes); `capacity()`, `raw_buffer()`, `size()`,
`as_bytes()`.

`STSeriesPacket`:

| Member | Contract |
|---|---|
| `STSeriesPacket()`, `STSeriesPacket(const buffer_t&)` | The buffer constructor binds a buffer and resets it to the sync header. The default constructor is declared defaulted but cannot be used, because `ServoPacket` has no default constructor. |
| `sync_buffer()`, `data_buffer()`, `len_index()`, `checksum_index()`, `data_size()` | Views and indices; `data_size()` is the payload byte count. |
| `reset()` | Size 0 plus sync header. |
| `is_well_formed()`, `is_frame_valid()`, `is_checksum_valid()`, `packet_device_id()` | Validation (size >= 6, sync, length field, checksum); device ID or -1 when too short. |
| `compute_checksum()`, `compute_length()` | Values the frame should carry. |
| `get_sync()`, `get_device_id()`, `get_length()`, `get_code()`, `get_data()`, `get_data_byte(idx, default)`, `get_data_word(idx, default)`, `get_checksum()` | Read fields; out-of-range reads return the default (0) or an empty view. |
| `set_sync`, `set_device_id`, `set_length`, `set_code`, `set_data`, `set_checksum`, `update_length`, `update_checksum` | Write fields, growing the packet size as needed; `SRV_OVERFLOW` if the buffer is too small. |
| `init(sync, dev_id, length, code, data, checksum)` | Builds a frame from explicit fields. |
| `init(dev_id, code, data)` | Builds a complete frame: sync, length, and a correct checksum. |

## ESPSerialPort

`class ESPSerialPort : public SerialPort` in `platform/esp_idf.h`. Tier T.
Available only when building with ESP-IDF (the umbrella header includes it
under `ESP_PLATFORM` or `SERVOKIT_IMPL_ESP`).

- `ESPSerialPort(uart_num, tx_pin, rx_pin, baud_rate = 115200,
  rx_buffer_size = -1, tx_buffer_size = 0, intr_alloc_flags = 0, tx_od = true,
  tx_pu = true, rx_pu = true)`: stores the configuration only; an
  `rx_buffer_size <= 0` selects the UART hardware FIFO length plus one. Equal
  TX and RX pins select a single-wire half-duplex bus.
- `initialize()`: configures the pins and installs the UART driver. `SRV_OK`,
  `SRV_PARAM_ERR` on configuration errors, `SRV_FAIL` if already initialized or
  the driver is already installed.
- `close()`: removes the driver and resets the pins (also run by the
  destructor). `SRV_OK`, or `SRV_PARAM_ERR`.
- The `SerialPort` methods as above. `write_bytes` copies into the driver and
  waits for transmission only if `timeout` is positive.
- Getters: `get_uart_num()`, `get_tx_pin()`, `get_rx_pin()`, `get_baud_rate()`,
  `get_rx_buffer_size()`, `get_tx_buffer_size()`, `get_intr_alloc_flags()`,
  `get_tx_od()`, `get_tx_pu()`, `get_rx_pu()`. No bus access.

Known limitation: `ESPSerialPort::read_sync()` re-arms the full timeout on
every read and only counts time down when the build defines
`DISABLE_SRV_TIMEOUTS`. In a default build, a bus that stays silent does not
make `read_sync()` return `SRV_TIMEOUT`, so a transaction on an unresponsive
servo is not bounded by the transaction budget on ESP-IDF. The host `SerialPort`
fake does not show this.

## Hardware-verification list

The code marks these as not verified on a physical servo. They follow the
register map or vendor documentation.

1. `REG_EEPROM_LOCK` semantics: lock off stores writes, lock on makes them
   volatile across a power cycle.
2. `PHASE_VERIFY_MASK`: the meaning of `REG_PHASE` bits is not documented; bits
   3 and 6 are excluded from write verification on community documentation.
3. `prepare()`: that `OP_SERVO` position mode is the right mode for the generic
   operations.
4. `set_protection()` and the typed protection setters: the units and the
   meaning of the thresholds on a given servo revision.
5. `read_telemetry()`: the load encoding (bit 10 sign, 0..1000), the current
   encoding (bit 15 sign, 6.5 mA) and that position is single-turn. The same
   encodings in `sts_units.h` (position bit 15 sign, load bit 10 sign, current
   bit 15 sign).
6. `MULTI_TURN_POS` and `RETAINS_POSITION_UNPOWERED` are not declared because
   they are unverified.
