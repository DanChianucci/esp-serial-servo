# servokit API reference

Back to the [overview](overview.md).

Everything is in namespace `servokit`. The **Tier** column uses the access
tiers from the overview: 1 = generic (`Servo&`), 2 = family (`STSServo`),
3 = register access, T = transport/infrastructure.

Contents:
[Common contract](#common-contract) |
[Units and conversions](#units-and-conversions) |
[Result](#result) |
[Timeouts and retries](#timeouts-and-retries) |
[Alarms](#alarms) |
[Capabilities and features](#capabilities-and-features) |
[Value types](#value-types) |
[Servo](#servo-generic-interface) |
[RegisterAccess](#registeraccess) |
[SerialLink](#seriallink) |
[Write semantics](#write-semantics) |
[Checksum option](#checksum-option) |
[STSServo](#stsservo) |
[STS units header](#sts-units-header) |
[STS definitions](#sts-definitions) |
[Status codes and helpers](#status-codes-and-helpers) |
[SerialPort](#serialport) |
[ServoController](#servocontroller) |
[Packets](#packets) |
[Timing helpers](#timing-helpers) |
[SyncMatcher](#syncmatcher) |
[Internal macros](#internal-macros) |
[ESPSerialPort](#espserialport) |
[Hardware-verification list](#hardware-verification-list)

## Common contract

These rules hold for every bus operation unless an entry says otherwise.

- **Blocking.** Calls are synchronous. A call returns when the transaction
  finishes or its time budget expires.
- **Finite time, no retries.** One call is one command/response transaction
  (plus one read-back when the readback check is on). The library never
  retries and never loops waiting for a servo.
- **Errors.** Returned as a [`Result`](#result); nothing throws.
- **Thread safety.** Servo objects, controllers, ports and packets are not
  thread-safe, and neither is the bus. Serialize all calls per bus, including
  calls through `get_registers()` and `get_serial_link()`, from one task or under a mutex. Pure value types and
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

STS native units (`families/stseries/sts_units.h`, `STSServo`):

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
positions are limited to +-30719 steps (`STSServo::MAX_TARGET_POSITION`; the
memory table lists +-32766); the generic angle domain is the same range
expressed in mdeg.

## Result

`enum class Result : uint8_t`, tier 1-3. Returned by every bus operation.

| Value | Meaning |
|---|---|
| `OK` | Completed. Output parameters are valid. |
| `TIMEOUT` | No complete response within the transaction budget. Also returned for a write to a servo in response mode `RSP_READ_PING` (the servo does not acknowledge writes; the write took effect). |
| `BAD_RESPONSE` | Framing, checksum, length or device-ID mismatch; the servo reported a non-zero error status (see `STSServo::get_last_status()`); or a register held a value that is not valid for its type (for example an operating mode above `OP_STEP`). |
| `UNSUPPORTED` | The servo/family does not provide the operation. Default of every optional `Servo` method. |
| `OUT_OF_RANGE` | Argument or register span outside the servo's domain or limits. Nothing is sent. |
| `PARAM` | Invalid argument (null pointer, empty span, broadcast or invalid ID, no controller, caller buffer too small). |
| `BUS` | Transport failure (read/write failure, driver error), not a servo fault. |
| `VERIFY_FAILED` | With the readback check on, a write read back different from the bytes written. The write took effect. |

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
  `set_transaction_timeout()`, read it with `get_transaction_timeout()`.
- The send step is clamped to the budget; the response step gets only the time
  left in the budget, so a slow send shortens the receive window.
- When the budget is spent the call returns `TIMEOUT`.
- A transaction that needs a read-back (readback check) or two frames
  (`cancel_motion()` takes three frames; `set_multiturn_feedback()` two) takes one budget per
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
`get_capabilities().features` before depending on an optional operation.

Features: `ABSOLUTE_POSITION`, `MULTI_TURN_POS`, `RELATIVE_POSITION`,
`CONTINUOUS_ROTATION`, `PWM_DRIVE`, `SET_SPEED`, `SET_ACCEL`, `TORQUE_LIMIT`,
`CURRENT_LIMIT`, `START_TORQUE`, `TORQUE_ENABLE`, `CANCEL_MOTION`, `POSITION_GAINS`,
`DEADZONE`, `ANGLE_LIMITS`, `POSITION_OFFSET`, `READ_POSITION`, `READ_SPEED`,
`READ_CURRENT`, `READ_LOAD`, `READ_VOLTAGE`, `READ_TEMPERATURE`,
`READ_MOVING`, `READ_ALARMS`, `PROTECT_CURRENT`, `PROTECT_OVERLOAD`,
`PROTECT_TEMP`, `PROTECT_VOLTAGE`, `PROTECT_RELEASE_ON_ALARM`, `PROBE`,
`IDENTIFY`, `SET_ID`, `SET_BAUD`, `SYNC_WRITE`, `SYNC_READ`,
`ASYNC_WRITE`, `BROADCAST`, `REGISTER_ACCESS` (`Servo::get_registers()` is non-null exactly when it is set),
`RETAINS_POSITION_UNPOWERED`, `PERSIST_CONFIG` (`set_persist()` controls
persistence), `READBACK_CHECK` (`RegisterAccess::set_readback_check()` is
available).

`STSServo::get_capabilities()` declares: `ABSOLUTE_POSITION`, `SET_SPEED`,
`SET_ACCEL`, `TORQUE_LIMIT`, `START_TORQUE`, `TORQUE_ENABLE`, `CANCEL_MOTION`,
`READ_POSITION`, `READ_CURRENT`, `READ_LOAD`, `READ_VOLTAGE`,
`READ_TEMPERATURE`, `READ_MOVING`, `READ_ALARMS`, `PROTECT_CURRENT`,
`PROTECT_OVERLOAD`, `PROTECT_TEMP`, `PROTECT_VOLTAGE`, `PROTECT_RELEASE_ON_ALARM`, `PROBE`, `IDENTIFY`,
`REGISTER_ACCESS`, `PERSIST_CONFIG`, `READBACK_CHECK`. It does not declare
`MULTI_TURN_POS` or `RETAINS_POSITION_UNPOWERED` (unverified on hardware), nor
any feature without an implementation.

`Capabilities` holds `features` and `limits`. `Limits` values are valid only
if the matching feature is set, otherwise 0. STS values: angle
-30719..+30719 steps converted to mdeg, `resolution_mdeg` 88 (87.890625
rounded), maximum speed 3400 steps/s and acceleration 254 units converted to
mdeg/s and mdeg/s², `max_torque_permille` 1000, `max_torque_mnm` 1912
(the ST3215 stall torque, 19.5 kgf*cm, rotor locked at a 7.4 V supply per the
manufacturer datasheet; lower at lower voltages, 16.5 kgf*cm at 6 V;
informational only, commands stay in permille), current range +-the largest 15-bit current
feedback in mA (the reportable range; the settable stall threshold is
narrower), `reportable_alarms` from `reportable_alarms()`.

## Value types

All are plain copyable values in `core/servo_types.h`; no heap, no
references; nothing blocks.

| Type | Fields |
|---|---|
| `Limits` | `angle_min_mdeg`, `angle_max_mdeg`, `resolution_mdeg`, `max_speed_mdeg_s`, `max_accel_mdeg_s2`, `max_torque_permille`, `max_torque_mnm` (informational, mN*m, 0 = unknown), `current_min_ma`, `current_max_ma`, `reportable_alarms` |
| `Capabilities` | `features`, `limits` |
| `Identity` | `family[16]` (NUL-terminated, owned, e.g. `"STS"`), `model_major`, `model_minor`, `firmware_major`, `firmware_minor`, `device_id` |
| `Telemetry` | `position_mdeg`, `current_ma`, `temperature_c`, `alarms`; optional `load_valid`/`load_permille`, `voltage_valid`/`voltage_mv`, `moving_valid`/`moving` (read an optional field only when its flag is true) |
| `MotionMode` | `ABSOLUTE_POSITION` |

## Servo (generic interface)

`class Servo` in `core/servo.h`. Tier 1. Only `move_to` and `get_capabilities`
are pure; every other method defaults to `UNSUPPORTED` (or the stated default
value). The destructor is virtual. The Servo does not own the bus.

| Method | Contract |
|---|---|
| `move_to(angle_mdeg)` | Absolute move. `OUT_OF_RANGE` outside the limits' angle domain (nothing sent). |
| `get_capabilities()` | Const; does not touch the bus. Static per model; an implementation may make it depend on a prior `identify()`. |
| `probe()` | One transaction, no retries. |
| `identify(Identity*)` | Family, model, firmware, device ID. |
| `set_mode(MotionMode)` | Selects the motion mode (`MotionMode::ABSOLUTE_POSITION`: absolute target over the full domain). A value that is not a `MotionMode` returns `OUT_OF_RANGE`. |
| `set_protect_current(ma, time_ms)` | Current protection (`PROTECT_CURRENT`): threshold in mA and time above it in ms. |
| `set_protect_overload(permille, time_ms)` | Overload protection (`PROTECT_OVERLOAD`): load threshold in permille and time above it in ms. |
| `set_protect_temperature(c)` | Temperature trip threshold in degrees C (`PROTECT_TEMP`). |
| `set_protect_voltage(min_mv, max_mv)` | Supply voltage range in mV (`PROTECT_VOLTAGE`); `min_mv` above `max_mv` is `OUT_OF_RANGE`. |
| `set_protect_release_alarms(AlarmSet)` | Alarms that make the servo release torque (`PROTECT_RELEASE_ON_ALARM`). An alarm the servo has no setting for is `OUT_OF_RANGE`. |
| `set_speed(mdeg_s)`, `set_acceleration(mdeg_s2)` | Speed and acceleration for following moves (`SET_SPEED`, `SET_ACCEL`). |
| `set_torque_limit(permille)`, `set_start_torque(permille)` | Torque limit and starting torque (`TORQUE_LIMIT`, `START_TORQUE`). |
| `cancel_motion()` | Cancels a move in progress (`CANCEL_MOTION`). On `OK` the servo is stationary, its target equals its position and torque is disabled. The servo is unpowered between the torque-off and the end of the sequence; nothing restores torque. |
| `set_torque_enabled(bool)`, `read_torque_enabled(bool*)` | Enables or disables motor torque; reads the state (`TORQUE_ENABLE`). `read_torque_enabled` fills its output on `OK` only. |
| `set_persist(bool)` | `true`: configuration writes made afterwards survive a power cycle; `false`: they take effect but are not kept. Applies until changed again; the library keeps no state. |
| `read_telemetry(Telemetry*)` | One atomic sample, filled on `OK` only. |
| `read_alarms(AlarmSet*)` | Active alarms. |
| `clear_alarms()` | Clears latched alarms where supported. |
| `get_registers()` | Non-owning `RegisterAccess*`, valid for the lifetime of the `Servo`; `nullptr` by default. Non-null exactly when `Feature::REGISTER_ACCESS` is set. See [RegisterAccess](#registeraccess). |
| `get_serial_link()` | Non-owning `SerialLink*`, valid for the lifetime of the `Servo`; `nullptr` by default (servos not on a checksummed serial link). See [SerialLink](#seriallink). |

Every protection and motion-parameter setter above is gated by its own
feature and returns `UNSUPPORTED` (nothing sent) when that feature is absent,
independent of the others. Arguments are validated first: an out-of-range value
returns `OUT_OF_RANGE`, nothing is sent and nothing is clamped (callers clamp
using `Limits`). Then plain writes are made. A setter that writes several
registers stops at the first failed write and returns that write's `Result`;
earlier writes of the same call may have applied. Conversions to native units
round to nearest, halves away from zero.

No RTTI is used: the `RegisterAccess` and `SerialLink` interfaces are reached
through `get_registers()` and `get_serial_link()` only.

### PersistScope

`class PersistScope` in `core/persist_scope.h` (header-only). The constructor
takes a `Servo&` (borrowed; must outlive the guard) and calls
`set_persist(true)`. Unlocking is explicit: `ok()` and `result()` report the
unlock. The destructor calls `set_persist(false)` only if the unlock succeeded,
so every exit path of the scope relocks; a relock failure in the destructor
cannot be reported. `close()` relocks early, returns the relock `Result` and
makes the destructor do nothing (it sends nothing and returns `OK` when the
unlock failed or `close()` already ran). Not copyable, not movable, not
thread-safe; blocking and timeouts are those of `set_persist()`.

## RegisterAccess

`class RegisterAccess` in `core/register_access.h`. Tier 3. Register access to
the servo's own device; the implementation holds the device ID. Addresses and
meanings are family-defined, there is no unit conversion, and multi-byte values
are little endian (low byte at the lower address). Every call is one blocking
bus transaction bounded by the bus timeouts, with no retries; not thread-safe
(see the [common contract](#common-contract)). Pointer arguments are used only
during the call; outputs are written on `OK` only. Errors: `PARAM` for a null
pointer, empty span or invalid target, `OUT_OF_RANGE` for a span outside the
table, bus errors as usual.

| Method | Contract |
|---|---|
| `read_register(addr, data, len)` | Reads `len` bytes from `addr` into the caller buffer `data` (at least `len` bytes). |
| `write_register(addr, data, len)` | Writes `len` bytes, as given. `data` is read during the call only. With the readback check on, followed by one verification read. |
| `read_u8(addr, out)`, `read_u16(addr, out)` | One byte / one little-endian word. |
| `write_u8(addr, value)`, `write_u16(addr, value)` | One byte / one little-endian word; as `write_register`. |
| `set_readback_check(bool)` | Turns the readback check on or off (default off); touches no bus. `UNSUPPORTED` if the implementation has none. See [write semantics](#write-semantics). |
| `get_readback_check()` | Current setting; no bus access. |

## SerialLink

`class SerialLink` in `serial/serial_link.h` (it describes the serial transport,
so it lives with `serial/`; `core/servo.h` only forward-declares it). Tier T.
Link-level settings of servos on a checksummed serial bus; no call touches the
bus. Not thread-safe. The setting belongs to the whole controller behind the
servo, so servos sharing a controller share it. See
[checksum option](#checksum-option).

| Method | Contract |
|---|---|
| `set_verify_checksum(bool)` | Enables or disables response checksum verification (default on). `PARAM` if the servo has no controller. |
| `get_verify_checksum()` | Current setting; `true` when there is no controller. |
| `get_checksum_errors()` | Count of replies with a bad checksum, whether or not verification is on; 0 when there is no controller. |

## Write semantics

**Plain writes.** Every write, in every tier, is sent as given. There is no
implicit read-compare-write and no hidden read. A caller that wants to avoid
a write reads first. The only helper that reads before writing is
`STSServo::set_multiturn_feedback()` (it must preserve the other bits of
`REG_PHASE`), and `cancel_motion()` reads the position it commands. Each typed setter
performs exactly one register write (plus the read-back when the readback
check is on). `set_protect_current()`, `set_protect_overload()` and `set_protect_voltage()`
perform two plain writes in a fixed order and stop at the first failure.

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

**Readback check** (`RegisterAccess::set_readback_check(true)`, off by default). After a
successful `write_register()` of a span, one read of the same span is sent and
each byte is compared with the byte written through that byte's compare mask.
This happens in `write_register()`, so `write_u8`, `write_u16`, the
`RegisterAccess` calls, every typed helper and every generic operation are covered alike.

- The read is addressed to the written ID, except that a write covering
  `REG_DEVICE_ID` reads from the newly written ID; if that ID is above
  `STS::MAX_ID` (0xFC) nothing is read and the write returns `OK`.
- Exactly one extra read frame per write that has at least one compared byte;
  none for a span of only volatile or unlisted bytes; none when the readback
  check is off.
- A write error is returned as is and nothing is read. A failed read-back
  returns that read's own error (the write took effect). A difference in a
  compared bit returns `VERIFY_FAILED` (the write took effect and the servo
  holds a different value).

Register classes (`STSServo::readback_rule(addr)`, a pure function; a register
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
| 0x12 | `REG_PHASE` | STABLE_MASKED with `PHASE_READBACK_MASK` (bits 3 and 6 excluded) |
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

`ReadbackClass` has the values `STABLE` (compared in full), `STABLE_MASKED`
(compared in `ReadbackRule::mask` only) and `VOLATILE` (never compared).
`ReadbackRule` holds `kind` and `mask` (0xFF, a subset, or 0).

## Checksum option

STS frames end with `~(id + length + instruction/status + params/data)`, low
byte. Transmitted frames always carry a correct checksum. Verification of
**received** frames is on by default and can be switched:

- `SerialLink::set_verify_checksum(bool)` on `STSServo` forwards to its
  controller (`PARAM` if it has none). Servos without a serial link return
  `nullptr` from `get_serial_link()`.
- With verification off, a reply with a wrong checksum is still accepted if its
  frame (sync and length) and device ID validate.
- `get_checksum_errors()` counts replies with a bad checksum, whether or not
  verification is on; 0 without a controller. `get_verify_checksum()` is
  `true` without a controller.
- The accessors are not thread-safe (see the common contract).

## STSServo

`class STSServo : public Servo, public RegisterAccess, public SerialLink` in
`families/stseries/sts_servo.h`. Not thread-safe;
every call is one blocking transaction as described in the
[common contract](#common-contract).

STS-specific rules:

- **Target.** Methods without an `id` parameter act on `get_device_id()`. A
  broadcast (0xFE) or higher ID makes every method that uses the bus return
  `PARAM`; a null controller does too.
- **Words** are little endian (low byte at the lower register address).
- **Status.** A reply whose status byte is non-zero makes the call return
  `BAD_RESPONSE`; the byte is available from `get_last_status()`. The exceptions
  are `read_telemetry()`, `read_alarms()` and the position read of `cancel_motion()`,
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
| copy and move operations | 2 | Deleted: `STSServo` is neither copyable nor movable. An object that owns its controller would otherwise leave a moved-from object with a dangling controller pointer. Construct it in place, or hold it behind a `std::unique_ptr`. |
| `get_device_id()` | 2 | ID used by methods without an `id` parameter. No bus access. |
| `set_device_id(id)` | 2 | Changes only the ID this object addresses. Sends nothing; it does not reprogram the servo. |
| `get_last_status()` | 2 | Status byte of the most recent valid reply (bitmask of `err_code_t`, 0 when healthy). Reset at the start of every call that uses the bus; stays 0 if no valid reply arrived. |

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
| `STALL_TORQUE_MNM` | 1912; ST3215 stall torque in mN*m (19.5 kgf*cm, rotor locked, 7.4 V supply); informational |
| `ALARM_MASK_BITS` | 0x3F; defined `err_code_t` bits |
| `PHASE_MULTITURN_BIT` | 1 << 4; `REG_PHASE` multi-turn feedback bit |
| `PHASE_READBACK_MASK` | `REG_PHASE` bits compared by the readback check (all except bits 3 and 6) |
| `REGISTER_SPACE` | 256; the control table is addressed by one byte |
| `MAX_SPAN` | 64; longest register span of one read or write |

### Explicit-ID register access (tier 3)

These take the servo ID per call, so one object can address any servo on the
bus. The `RegisterAccess` overloads without an `id` act on `get_device_id()`.

| Method | Contract |
|---|---|
| `probe(id)` | Checks that the servo with `id` answers. Unicast ID 0..0xFD. 0xFE (broadcast) and above returns `PARAM`. |
| `read_register(id, addr, data, len)` | `len` 1..`MAX_SPAN` and `addr + len <= REGISTER_SPACE`. `PARAM` for a null buffer or `len == 0`; `OUT_OF_RANGE` for a longer span or a span past the table. `data` is untouched on failure. |
| `write_register(id, addr, data, len)` | Same limits. Written as given, no EEPROM handling; followed by a readback check when `get_readback_check()` is on. |
| `read_u8`, `read_u16` | One byte / one little-endian word; `PARAM` for a null output. |
| `write_u8`, `write_u16` | One byte / one little-endian word; as `write_register`. |
| `readback_rule(addr)` | Static, pure, no bus. See [write semantics](#write-semantics). |

### Typed helpers (tier 2)

Each does one register write (or read) on `get_device_id()`; "SRAM" registers
reset at power-up, "EEPROM" registers follow the lock.

| Method | Contract |
|---|---|
| `get_operating_mode(op_mode_t*)` | `BAD_RESPONSE` if the stored value is not a mode. |
| `set_operating_mode(op_mode_t)` | `OP_SERVO`..`OP_STEP`; others `OUT_OF_RANGE`. Decides how `REG_CTRL_*` are read. |
| `get_response_mode(rsp_mode_t*)`, `set_response_mode(rsp_mode_t)` | `RSP_READ_PING` stops write acknowledgements. An application using the generic interface must call `set_response_mode(STS::RSP_ALL)` once after power-up so that writes are acknowledged; no generic call does it. |
| `get_response_delay(uint8_t*)`, `set_response_delay(delay_2us)` | Units of 2 us, 0..254. |
| `get_servo_enabled(bool*)`, `set_servo_enabled(bool)` | Torque output on/off (SRAM). Never writes the mid-point calibration value. `get_servo_enabled` returns `BAD_RESPONSE` if the register holds neither 0 nor 1. |
| `set_acceleration_native(accel)` | 0..254, units of 100 steps/s^2 (SRAM). |
| `set_speed_native(steps_per_s)` | 0..3400, unsigned magnitude for position moves (SRAM). |
| `set_signed_speed(steps_per_s)` | -3400..3400, bit 15 = reverse, for constant-speed mode (SRAM). |
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
| `probe()` | `probe(get_device_id())`. |
| `get_registers()`, `get_serial_link()` | Both return `this`. |
| `set_persist(bool)` | `unlock_eeprom()` / `lock_eeprom()`; keeps no state. |
| `read_register`, `write_register`, `read_u8`, `read_u16`, `write_u8`, `write_u16` without an `id` | The explicit-ID call with `get_device_id()` (the `RegisterAccess` interface). |
| `set_verify_checksum`, `get_verify_checksum`, `get_checksum_errors` | Forward to the controller (the `SerialLink` interface). |
| `set_readback_check(bool)` | Always `OK`; touches no bus. `get_readback_check()` returns the setting. |
| `move_to(angle_mdeg)` | Converts to steps (nearest) and writes `REG_CTRL_POS` once. `OUT_OF_RANGE` outside the limits' domain (nothing sent). |
| `get_capabilities()` | Fixed STS description; no bus access; independent of `identify()`. |
| `identify(Identity*)` | One read of the six bytes `0x00..0x05`; the device ID is the value stored in the servo. |
| `set_mode(MotionMode)` | `ABSOLUTE_POSITION` writes `REG_OPERATION_MODE = OP_SERVO`: one plain write, no reads. That `OP_SERVO` is the right mode for the generic operations is not verified on hardware. Other values return `OUT_OF_RANGE`, nothing sent. |
| `set_protect_current(ma, time_ms)` | Writes overcurrent threshold (`ma` to 6.5 mA units, at most 511 = 3322 mA) then overcurrent time (`time_ms` to 10 ms units, at most 254 = 2540 ms). |
| `set_protect_overload(permille, time_ms)` | Writes overload threshold (`permille` to percent of maximum torque, rounded, at most 1000 permille; below 5 permille writes 0, whose meaning is unverified) then overload time (10 ms units, at most 254). The register units come from the register map; how the threshold relates to the load feedback is unverified on hardware. |
| `set_protect_temperature(c)` | Writes `REG_OVERTEMP_THRESH`, 0..100 C. |
| `set_protect_voltage(min_mv, max_mv)` | Writes `REG_UNDERVOLT_THRESH` (`min_mv`) then `REG_OVERVOLT_THRESH` (`max_mv`), each converted to 0.1 V units rounded to nearest, at most 254 (25.4 V). `min_mv` above `max_mv` is `OUT_OF_RANGE`. Unit semantics unverified on hardware. |
| `set_protect_release_alarms(AlarmSet)` | Writes `REG_UNLOAD_ALARM_MASK`. An alarm without a native bit (`STALLED`, `COMMUNICATION`, `UNLOADED`, `OTHER`) returns `OUT_OF_RANGE` and writes nothing. `OVER_VOLTAGE` and `UNDER_VOLTAGE` share one native bit, so requesting either releases torque on both. |
| `set_speed(mdeg_s)` | Converts to steps/s (nearest) and writes `REG_CTRL_SPEED`; more than 3400 steps/s is `OUT_OF_RANGE`. |
| `set_acceleration(mdeg_s2)` | Converts to units of 100 steps/s^2 (nearest) and writes `REG_CTRL_ACCEL`; more than 254 is `OUT_OF_RANGE`. |
| `set_torque_limit(permille)`, `set_start_torque(permille)` | 0..1000 (the register unit 0.1 % equals permille); `REG_CTRL_TORQUE` (SRAM) and `REG_START_TORQUE` (EEPROM region, follows the lock). |
| `cancel_motion()` | Writes `REG_SERVO_EN = 0`, reads `REG_POSITION`, then writes that position as the target (three transactions). Torque need not be on beforehand. On `OK` the servo is stationary, target equals position and torque is disabled; it is unpowered between the torque-off and the end of the sequence and nothing restores torque. If a later step fails torque stays disabled; if the torque-off fails the torque state is unknown. A position outside the domain returns `OUT_OF_RANGE` and writes no target. A servo reporting an alarm still yields its position. Never reports `OK` for a write that was not acknowledged. |
| `get_cancel_failed_step()` | Step that failed in the latest `cancel_motion()`: `CancelStep::NONE`, `DISABLE_TORQUE`, `READ_POSITION`, `POSITION_OUT_OF_DOMAIN` or `WRITE_TARGET`. |
| `set_torque_enabled(bool)`, `read_torque_enabled(bool*)` | `set_servo_enabled`, `get_servo_enabled`. |
| `read_telemetry(Telemetry*)` | One 15-byte read of `REG_POSITION..REG_CURRENT`. Fails as a whole on a transport error, short reply, or an undecodable field (load outside +-1000 permille, moving flag other than 0/1) with `BAD_RESPONSE`. Position is single-turn. Load, voltage and moving are always valid on STS. |
| `read_alarms(AlarmSet*)` | Reads `REG_STATUS` and combines it with the reply status byte through `alarms_from_native()`. |

Not provided by STS (default `UNSUPPORTED`): `clear_alarms()`.

## STS units header

`families/stseries/sts_units.h`, namespace `STS::units`. Header-only pure functions
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
| `voltage_units_to_mv`, `mv_to_voltage_units` | 0.1 V units to mV (exact), and mV to 0.1 V units (rounded). |
| `permille_to_percent` | Permille of full scale to percent (1 % units), rounded. |
| `ms_to_time_units` | Milliseconds to 10 ms units. |
| `decode_sign_magnitude(raw, sign_bit)` | Sign bit selects the sign, lower bits the magnitude; negative zero is 0; bits above the sign bit are the caller's to check. |
| `decode_position_steps`, `decode_current_units` | Bit 15 sign. |
| `decode_load_permille(raw, int16_t*)` | Bit 10 sign, magnitude 0..1000. Returns false (output untouched) if a bit above bit 10 is set or the magnitude exceeds 1000. |
| `AlarmMapRow` (`native_bit`, `generic`), `ALARM_TABLE`, `MAPPED_STATUS_BITS` | One row of the native-to-generic alarm table: the `err_code_t` bit and the generic alarms it reports; the table; and the bits that have a row (0x3F). See [Alarms](#alarms). |
| `alarms_from_native(status)` | Maps a status byte; unmapped set bits report `OTHER`. |
| `reportable_alarms()` | Every alarm with a table row, plus `OTHER`. |
| `native_from_alarms(AlarmSet, uint8_t*)` | Native mask; false if an alarm has no native bit (output untouched). |

## STS definitions

`families/stseries/stseries_defines.h`, namespace `STS`.

- Protocol constants: `LIB_FW_VERSION`, `SYNC_PATTERN` (0xFFFF), `MAX_ID`
  (0xFC, the highest device ID the readback check follows after a write to
  `REG_DEVICE_ID`; the memory table allows IDs up to 0xFD), `BROADCAST_ID`
  (0xFE, no response),
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

### Control table

Address, width in bytes, access and memory region of every register in
`reg_addr_e`, from the ST3215 memory table in `doc/STS3215`. Two-byte registers
are little endian. "EEPROM" registers keep their value across a power cycle
unless `REG_EEPROM_LOCK` is 1; "SRAM" registers reset at power-up. Items
marked unverified in the header comments have not been confirmed on hardware
(see the [hardware-verification list](#hardware-verification-list)). Where the
memory table's range column and its text disagree, the header comment says so.

| Register | Address | Bytes | Access | Region | Meaning | Values, unit, notes |
|---|---|---|---|---|---|---|
| `REG_VER_MAJ` | 0x00 (0) | 1 | RO | EEPROM | Firmware Major Version |  |
| `REG_VER_SUB` | 0x01 (1) | 1 | RO | EEPROM | Firmware Minor Version |  |
| `REG_MODEL_MAJ` | 0x03 (3) | 1 | RO | EEPROM | Servo model major (memory table: "main version") |  |
| `REG_MODEL_SUB` | 0x04 (4) | 1 | RO | EEPROM | Servo model minor (memory table: "sub version") |  |
| `REG_DEVICE_ID` | 0x05 (5) | 1 | RW | EEPROM | Device ID, unique on the bus; 0xFE is broadcast | values [0, 0xFD] per the memory table; the library treats IDs above 0xFC (MAX_ID) as not readable back |
| `REG_BAUD_RATE` | 0x06 (6) | 1 | RW | EEPROM | Baud Rate Setting | values `baud_rate_e`; unit enum |
| `REG_RESPONSE_DELAY` | 0x07 (7) | 1 | RW | EEPROM | Response (return) delay before the servo replies | values [0,254]; unit 2us |
| `REG_RESPONSE_MODE` | 0x08 (8) | 1 | RW | EEPROM | Response Mode: which commands are acknowledged | values `rsp_mode_t`; unit enum |
| `REG_MIN_ANGLE_THRESH` | 0x09 (9) | 2 | RW | EEPROM | Min Angle Limit (stroke minimum) | values [0,4094] per the memory table; unit 1step; must be below the maximum; the memory table says 0 for multi-turn absolute control |
| `REG_MAX_ANGLE_THRESH` | 0x0B (11) | 2 | RW | EEPROM | Max Angle Limit (stroke maximum) | values [1,4095] per the memory table; unit 1step; must be above the minimum; the memory table says 0 for multi-turn absolute control |
| `REG_OVERTEMP_THRESH` | 0x0D (13) | 1 | RW | EEPROM | Maximum Temperature for alarm | values [0,100]; unit °C |
| `REG_OVERVOLT_THRESH` | 0x0E (14) | 1 | RW | EEPROM | Maximum Voltage for alarm | values [0,254]; unit 0.1V |
| `REG_UNDERVOLT_THRESH` | 0x0F (15) | 1 | RW | EEPROM | Minimum Voltage for alarm | values [0,254]; unit 0.1V |
| `REG_CONTROL_TORQUE_NV` | 0x10 (16) | 2 | RW | EEPROM | Stored Maximum Control Torque | values [0,1000]; unit 0.1%; value copied to `REG_CTRL_TORQUE` at startup |
| `REG_PHASE` | 0x12 (18) | 1 | RW | EEPROM | SPECIAL FUNCTION BYTE | only write if other register description says to; bit meanings are not in the memory table (bit 4 = multi-turn feedback is used by the library, unverified) |
| `REG_UNLOAD_ALARM_MASK` | 0x13 (19) | 1 | RW | EEPROM | Unloading Condition Alarm Mask | values `err_code_t` bits; unit bitmask (bit set = that fault releases torque) |
| `REG_LED_ALARM_MASK` | 0x14 (20) | 1 | RW | EEPROM | LED Indicator Alarm Mask | values `err_code_t` bits; unit bitmask (bit set = that fault flashes the LED) |
| `REG_POS_COEF_P` | 0x15 (21) | 1 | RW | EEPROM | Position Loop P Coefficient | values [0,254]; unit 1x |
| `REG_POS_COEF_D` | 0x16 (22) | 1 | RW | EEPROM | Position Loop D Coefficient | values [0,254]; unit 1x |
| `REG_POS_COEF_I` | 0x17 (23) | 1 | RW | EEPROM | Position Loop I Coefficient | values [0,254]; unit 1x |
| `REG_START_TORQUE` | 0x18 (24) | 2 | RW | EEPROM | Minimum starting control Torque | values [0,1000]; unit 0.1% |
| `REG_CW_DEADZONE` | 0x1A (26) | 1 | RW | EEPROM | Clockwise Deadzone | values [0,32]; unit 1step |
| `REG_CCW_DEADZONE` | 0x1B (27) | 1 | RW | EEPROM | Counter Clockwise Deadzone | values [0,32]; unit 1step |
| `REG_OVERCURR_THRESH` | 0x1C (28) | 2 | RW | EEPROM | Maximum Current for alarm | values [0,511]; unit 6.5mA |
| `REG_ANGLE_RESOLUTION` | 0x1E (30) | 1 | RW | EEPROM | Angle Resolution Amplification Factor | values [1,100] per the memory table; unit factor on the sensor's minimum resolution angle; larger values extend the number of turns |
| `REG_POS_OFFSET` | 0x1F (31) | 2 | RW | EEPROM | Position Correction Offset | values [+-2047]; unit 1step; bit[11] = direction |
| `REG_OPERATION_MODE` | 0x21 (33) | 1 | RW | EEPROM | Operating Mode Control | values `op_mode_t`; unit enum |
| `REG_UNLOAD_CTRL_TORQUE` | 0x22 (34) | 1 | RW | EEPROM | Maximum Control Torque for Unloaded condition | values [0,100] meaningful (table range [0,254]); unit 1.0% of maximum torque |
| `REG_OVERLOAD_TIME` | 0x23 (35) | 1 | RW | EEPROM | Duration for Overload Torque to cause alarm | values [0,254]; unit 10 ms |
| `REG_OVERLOAD_THRESH` | 0x24 (36) | 1 | RW | EEPROM | Maximum Load for alarm | values [0,100] meaningful (table range [0,254]); unit 1.0% of maximum torque |
| `REG_SPEED_COEF_P` | 0x25 (37) | 1 | RW | EEPROM | Speed Closed Loop P coefficient (speed mode) | values [0,254] per the memory table; unit 1x |
| `REG_OVERCURR_TIME` | 0x26 (38) | 1 | RW | EEPROM | Duration for Protection Current to cause Alarm | values [0,254]; unit 10ms |
| `REG_SPEED_COEF_I` | 0x27 (39) | 1 | RW | EEPROM | Speed Closed Loop I coefficient (speed mode) | values [0,254]; unit none stated in the memory table |
| `REG_SERVO_EN` | 0x28 (40) | 1 | RW | SRAM | Servo Enable / Disable (torque switch) | values `servo_en_t`; unit enum; value 128 calibrates the current position to 2048 (`REG_POS_OFFSET` correction) |
| `REG_CTRL_ACCEL` | 0x29 (41) | 1 | RW | SRAM | Acceleration Control | values [0,254]; unit 100 steps/sec^2 |
| `REG_CTRL_POS` | 0x2A (42) | 2 | RW | SRAM | Position / STEP Control (target) | values [+-32766] per the memory table (the library limits itself to +-30719); unit 1step; bit[15] = direction (sign-magnitude) |
| `REG_CTRL_PWM` | 0x2C (44) | 2 | RW | SRAM | PWM Duty Cycle Control ("running time" in the memory table) | values [0,1000]; unit 0.1%; direction bit: bit[11] per the memory table, unverified |
| `REG_CTRL_SPEED` | 0x2E (46) | 2 | RW | SRAM | Speed Control | values [0,3400] (no-load speed; the memory table range column says 254); unit steps/sec; bit[15] = direction when `REG_OPERATION_MODE` == `OP_SPEED` |
| `REG_CTRL_TORQUE` | 0x30 (48) | 2 | RW | SRAM | Maximum Control Torque | values [0,1000]; unit 0.1%; Takes value from `REG_CONTROL_TORQUE_NV` at startup |
| `REG_EEPROM_LOCK` | 0x37 (55) | 1 | RW | SRAM | EEPROM Lock Flag | values `eeprom_lock_t`; unit enum; semantics unverified on hardware |
| `REG_POSITION` | 0x38 (56) | 2 | RO | SRAM | Sensed Position | unit 1step; bit[15] = sign (unverified) |
| `REG_SPEED` | 0x3A (58) | 2 | RO | SRAM | Sensed Speed | unit steps/sec |
| `REG_LOAD` | 0x3C (60) | 2 | RO | SRAM | Sensed Load (drive duty cycle) | unit 0.1%; bit[10] = sign, magnitude 0..1000 (unverified) |
| `REG_VOLTAGE` | 0x3E (62) | 1 | RO | SRAM | Sensed Voltage | unit 0.1V |
| `REG_TEMPERATURE` | 0x3F (63) | 1 | RO | SRAM | Sensed Temperature | unit °C |
| `REG_ASYNC_WRITE_FLAG` | 0x40 (64) | 1 | RO | SRAM | Flag set when a deferred (async) write is pending | unit N/A |
| `REG_STATUS` | 0x41 (65) | 1 | RO | SRAM | Servo Alarm Status | values `err_code_t` bits; unit bitmask |
| `REG_MOVE_FLAG` | 0x42 (66) | 1 | RO | SRAM | Movement Flag | values `move_flag_t`; unit enum |
| `REG_CURRENT` | 0x45 (69) | 2 | RO | SRAM | Sensed Current | unit 6.5mA; bit[15] = sign (unverified) |

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
| `command_response(cmd, rsp, cmd_timeout, rsp_timeout)` | Send, then read. The whole call is bounded by `get_transaction_timeout()`. `SRV_BAD_ID` if the reply carries a different device ID than the command. |
| `flush_rx()` | Discards buffered receive bytes; called before every command. |
| `get_transaction_timeout()`, `set_transaction_timeout(t)` | Budget, always positive; `SRV_PARAM_ERR` for a non-positive value. |
| `set_verify_checksum(bool)`, `get_verify_checksum()`, `get_checksum_errors()` | See [checksum option](#checksum-option). |
| `m_servo_bus`, `m_transmit_timeout`, `m_response_timeout`, `m_auto_disable_rx` | Public configuration members; changing them between calls is allowed, not during one. |

A timeout argument of `DEF_TIMEOUT` (zero) selects the configured value.

## Packets

`ServoPacket` (`serial/servo_packet.h`) is the abstract frame buffer;
`STSeriesPacket` (`families/stseries/stseries_packet.h`) implements the STS frame
`FF FF id length code data... checksum`. Tier T. Packets are views over a
caller-owned buffer (no heap); the buffer must outlive the packet; not
thread-safe. `STSeriesPacket` needs a buffer of at least `NUM_META_BYTES`
(6) bytes.

`ServoPacket`: `LOG_TAG`; constructor from `buffer_t` (the default
constructor is deleted); pure `sync_buffer()`, `data_buffer()`, `len_index()`,
`is_well_formed()`, `reset()`; `is_frame_valid()` (size, sync and length, not
checksum; defaults to `is_well_formed()`); `is_checksum_valid()` (default
true); `get_packet_device_id()` (default -1); `resize(size)` (`SRV_OVERFLOW` above
capacity; alters no bytes); `get_capacity()`, `get_raw_buffer()`, `get_size()`,
`as_bytes()`.

`STSeriesPacket`:

| Member | Contract |
|---|---|
| `STSeriesPacket()`, `STSeriesPacket(const buffer_t&)` | The buffer constructor binds a buffer and resets it to the sync header. The default constructor is declared defaulted but cannot be used, because `ServoPacket` has no default constructor. |
| `sync_buffer()`, `data_buffer()`, `len_index()`, `checksum_index()`, `data_size()` | Views and indices; `data_size()` is the payload byte count. |
| `reset()` | Size 0 plus sync header. |
| `is_well_formed()`, `is_frame_valid()`, `is_checksum_valid()`, `get_packet_device_id()` | Validation (size >= 6, sync, length field, checksum); device ID or -1 when too short. |
| `compute_checksum()`, `compute_length()` | Values the frame should carry. |
| `get_sync()`, `get_device_id()`, `get_length()`, `get_code()`, `get_data()`, `get_data_byte(idx, default)`, `get_data_word(idx, default)`, `get_checksum()` | Read fields; out-of-range reads return the default (0) or an empty view. |
| `set_sync`, `set_device_id`, `set_length`, `set_code`, `set_data`, `set_checksum`, `update_length`, `update_checksum` | Write fields, growing the packet size as needed; `SRV_OVERFLOW` if the buffer is too small. |
| `init(sync, dev_id, length, code, data, checksum)` | Builds a frame from explicit fields. |
| `init(dev_id, code, data)` | Builds a complete frame: sync, length, and a correct checksum. |

## Timing helpers

`serial/deadline.h`, namespace `servokit::timeout`. Header-only, no platform
dependency, safe from any thread. Used by `ESPSerialPort` and tested on a host
with an injected clock.

| Name | Contract |
|---|---|
| `BasicDeadline<Clock>` | A point in time on `Clock` after which an operation must stop waiting, fixed at construction from a timeout and a start instant (default: now). Arithmetic saturates: a timeout too large to represent never expires. A zero or negative timeout is expired at construction. |
| `BasicDeadline::clock`, `duration`, `time_point` | Type aliases of the clock. |
| `BasicDeadline(timeout, start = now)` | Builds the deadline. |
| `remaining(now = Clock::now())` | Time left, never negative. |
| `expired(now = Clock::now())` | True when `now` is at or past the deadline. |
| `Deadline` | `BasicDeadline<std::chrono::steady_clock>`. |
| `ceil_ms(duration)` | Whole milliseconds, rounded up; negative gives 0; saturates at `UINT32_MAX`. |
| `ceil_ticks(ms, tick_rate_hz, max_ticks)` | `ceil(ms * tick_rate_hz / 1000)` in 64 bits, so a wait never ends early; a zero rate is treated as 1; saturates at `max_ticks`. |

## SyncMatcher

`serial/sync_matcher.h`. Header-only streaming matcher that finds a byte
pattern in a byte stream one byte at a time (Knuth-Morris-Pratt), including a
pattern that starts inside a partial match. Memory is bounded by
`SyncMatcher::MAX_PATTERN` (16). No platform dependencies; not thread-safe.

| Member | Contract |
|---|---|
| `MAX_PATTERN` | 16; longest supported pattern, bytes. |
| `reset(pattern)` | Sets the pattern (borrowed: the storage must stay valid and unchanged while the matcher is used) and restarts matching. An empty pattern is matched from the start. Returns false, leaving the matcher empty, if the pattern is longer than `MAX_PATTERN`. |
| `restart()` | Restarts matching with the current pattern. |
| `feed(byte)` | Consumes one byte; true when it completes the pattern. After a match the next byte continues the search for an overlapping match; call `restart()` to discard the overlap. |
| `complete()` | True when the bytes since the last `restart()` or `reset()` end with the pattern, or the pattern is empty. |

## Internal macros

`serial/servo_internal.h` holds early-return macros used by the library's own
sources: `SRV_RETURN_IF(cond, val)`, `SRV_RETURN_IF_NOT(cond, val)` and
`SRV_RETURN_ON_ERR(expr)` (returns the status of an `srv_stat_t` or
`srv_result_t` expression when it is an error). They are implementation
helpers and not part of the public API; applications should not include the
header.

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
- The `SerialPort` methods as above. `write_bytes` hands the bytes to the
  driver and waits for transmission only if `timeout` is positive.
- Getters: `get_uart_num()`, `get_tx_pin()`, `get_rx_pin()`, `get_baud_rate()`,
  `get_rx_buffer_size()`, `get_tx_buffer_size()`, `get_intr_alloc_flags()`,
  `get_tx_od()`, `get_tx_pu()`, `get_rx_pu()`. No bus access.

Timeouts: every blocking `ESPSerialPort` method takes a deadline from a
monotonic clock when it is called (`serial/deadline.h`) and stops waiting when
it passes. The deadline does not depend on how many reads it takes, so
`read_sync()` returns `SRV_TIMEOUT` on a silent bus after `timeout` in total.
A zero or negative `timeout` still makes one non-blocking attempt. A `timeout`
too large to represent never expires.

- `read_sync`, `read_bytes`: wait on the UART driver for the time left.
  `read_bytes` returns the bytes read so far (possibly fewer than requested,
  or 0) when the deadline passes.
- `write_bytes`: `uart_write_bytes()` has no timeout, so handing the bytes to
  the driver is not bounded by the deadline (with a TX buffer size of 0 it
  returns once the bytes are in the UART FIFO, which can take the duration of
  the frame). The deadline, which starts when the call begins, bounds only the
  wait for transmission to finish, and that wait happens only for a positive
  `timeout`. On `SRV_TIMEOUT` the bytes stay queued and are still sent.
- `read_sync`: finds the pattern with a streaming matcher
  (`serial/sync_matcher.h`), so a pattern that starts inside a partial match
  (for example `FF FF` in `FF FF FF`, or `AA AB` in `AA AA AB`) is found.
  Patterns are at most `SyncMatcher::MAX_PATTERN` (16) bytes; a longer pattern
  returns `SRV_PARAM_ERR`. An empty pattern returns `SRV_OK` at once.
- `flush_output`: `SRV_TIMEOUT` when transmission is not done by the deadline.
- Tick conversion: the driver takes waits in RTOS ticks. The time left is
  rounded up to whole milliseconds, then converted to
  `ceil(ms * configTICK_RATE_HZ / 1000)` ticks in 64-bit arithmetic (exact for
  any tick rate), capped below `portMAX_DELAY`, so a wait never ends before the
  deadline and can overrun it by less than one tick.

Verification status: `lib/platform/esp_idf.cpp` is not built or run by the host
tests, and its timeout behaviour is not verified on ESP-IDF hardware. The host
tests cover the platform-independent parts (`Deadline`, tick arithmetic,
`SyncMatcher`). See the [hardware-verification list](#hardware-verification-list).

## Hardware-verification list

The code marks these as not verified on a physical servo. They follow the
register map or vendor documentation.

1. `REG_EEPROM_LOCK` semantics: lock off stores writes, lock on makes them
   volatile across a power cycle.
2. `PHASE_READBACK_MASK`: the meaning of `REG_PHASE` bits is not documented; bits
   3 and 6 are excluded from the readback check on community documentation.
3. `set_mode()`: that `OP_SERVO` position mode is the right mode for the generic
   operations.
4. `set_protect_*()` and the typed protection setters: the units and the
   meaning of the thresholds on a given servo revision, including the overload
   percent and the voltage range.
5. `read_telemetry()`: the load encoding (bit 10 sign, 0..1000), the current
   encoding (bit 15 sign, 6.5 mA) and that position is single-turn. The same
   encodings in `sts_units.h` (position bit 15 sign, load bit 10 sign, current
   bit 15 sign).
6. `MULTI_TURN_POS` and `RETAINS_POSITION_UNPOWERED` are not declared because
   they are unverified.
7. `ESPSerialPort` timeouts: `lib/platform/esp_idf.cpp` is not built or run by
   the host tests. That every blocking method returns by its deadline, the
   tick conversion against the configured tick rate, and the
   `uart_write_bytes()` hand-off time have not been verified on ESP-IDF
   hardware.
