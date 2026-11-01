# servokit overview

servokit is a C++ library for controlling serial-bus smart servos from a
microcontroller or a host. It provides a family-neutral `Servo` interface, a
transport-independent command/response engine and a driver for the STS series
(for example ST3215). It has no dependency on a platform SDK outside the
`platform/` layer, so everything else builds and runs on a host compiler.

Requirements: a C++23 compiler with `<expected>` and `<span>` (GCC 12+,
Clang 16+, MSVC 2022). Errors are returned as values; nothing throws.

Pages: this overview, and the [API reference](api.md), which lists every
public type, method, constant and free function with its contract.

## Layers

All names live in namespace `servokit`. Headers are under `include/servokit/`;
the umbrella header `servokit.h` includes everything except the platform
layer, which it adds on ESP-IDF builds.

| Layer | Headers | Role |
|---|---|---|
| core | `core/servo.h`, `core/register_access.h`, `core/servo_types.h`, `core/servo_utils.h` | Generic `Servo` interface, the `RegisterAccess` interface, value types (`Result`, `Capabilities`, `Telemetry`, `AlarmSet`, ...), status codes. No I/O. |
| serial | `serial/serial_port.h`, `serial/serial_link.h`, `serial/servo_controller.h`, `serial/servo_packet.h`, `serial/deadline.h` | `SerialPort` transport interface, `SerialLink` (checksum settings), `ServoController` (one command/response transaction with bounded time), packet base class, `Deadline` (monotonic deadline used by the ESP-IDF port). |
| families/stseries | `families/stseries/sts_servo.h`, `families/stseries/sts_units.h`, `families/stseries/stseries_packet.h`, `families/stseries/stseries_defines.h` | STS protocol framing, register map, unit conversions and the `STSServo` driver. |
| pwm | `pwm/` | Reserved for PWM-driven servos. Contains no code yet. |
| platform | `platform/esp_idf.h` | `ESPSerialPort`, a `SerialPort` on an ESP-IDF UART. |

Dependencies point downward: `families/stseries` uses `serial` and `core`; `serial`
uses `core`; `core` uses nothing. `platform` implements `serial`'s
`SerialPort`.

## Three access tiers

| Tier | Entry point | Units | Use it when |
|---|---|---|---|
| 1. Generic | `Servo&` | mdeg, mdeg/s, mA, mV, permille | The application should not care which servo family is attached. |
| 2. Family | `STSServo` typed helpers (`set_speed_steps`, `set_target_position`, `unlock_eeprom`, ...) | The family's native register units | You need an STS feature the generic interface does not model. |
| 3. Registers | `RegisterAccess*` from `Servo::get_registers()` (`read_register`, `write_register`, `read_u8`, `read_u16`, `write_u8`, `write_u16`), or the same calls with an explicit ID on `STSServo` | Register bytes | You need a register no helper covers. |

`STSServo` derives from `Servo`, `RegisterAccess` and `SerialLink`, so the three
tiers can be mixed on one object. From a `Servo&`, `get_registers()` and
`get_serial_link()` return non-owning pointers valid for the servo's lifetime, or
`nullptr` when the servo lacks that interface; `get_registers()` is non-null
exactly when `Feature::REGISTER_ACCESS` is set. No RTTI is used.
Every call that talks to the servo is a synchronous bus transaction that
returns a [`Result`](api.md#result); accessors such as `get_capabilities()` touch no
bus.

## Building the stack

Three objects are needed, each borrowing the one below it:

```cpp
#include "servokit.h"
using namespace servokit;

// 1. A SerialPort for your platform (here: the ESP-IDF UART port).
ESPSerialPort port(UART_NUM_1, tx_pin, rx_pin, 1000000);
port.initialize();

// 2. A controller: 20 ms to send plus 20 ms to receive one reply.
ServoController controller(&port, std::chrono::milliseconds(20),
                           std::chrono::milliseconds(20));

// 3. The servo with bus ID 1.
STSServo sts(&controller, 1);

// From here on, use it through the generic interface.
Servo& servo = sts;
// Once after power-up, so that writes are acknowledged (STS family call).
sts.set_response_mode(STS::RSP_ALL);
servo.set_mode(MotionMode::ABSOLUTE_POSITION);
servo.move_to(45000);  // 45 degrees
```

Ownership and lifetime: `STSServo(ServoController*, id)` and
`ServoController(SerialPort*, ...)` only borrow. The port must outlive the
controller and the controller must outlive the servo. The alternative
constructor `STSServo(std::unique_ptr<ServoController>, id)` takes ownership of
the controller (its port must still outlive the servo).

On a host, replace the platform port with any `SerialPort` implementation.
The runnable programs in `examples/` use the scripted fake in
`tests/host/fake_serial_port.h` and need no hardware:

| Example | Tier |
|---|---|
| `examples/example_generic.cpp` | 1: capabilities, set_mode, PersistScope, set_speed, set_acceleration, set_torque_limit, move_to, read_telemetry, cancel_motion |
| `examples/example_family.cpp` | 2: typed helpers and an explicit unlock/lock persist sequence |
| `examples/example_raw.cpp` | 3: register reads and writes |

They are built and run with the host tests (`tests/host/CMakeLists.txt`).

## Principles

- **Units.** The generic API uses millidegrees, mdeg/s, mdeg/s^2, mA, mV,
  degrees C and permille. The STS API uses native units, stated per helper.
  See [units and conversions](api.md#units-and-conversions).
- **Finite time.** Every call is bounded by the controller's transaction
  budget and the library never retries. A caller that wants retries loops and
  chooses the count. See [timeouts and retries](api.md#timeouts-and-retries).
- **No emulation.** A servo reports what it does in
  `Capabilities::features`; an operation it lacks returns `UNSUPPORTED`. See
  [capabilities](api.md#capabilities-and-features).
- **Writes are written as given.** No read-compare-write and no automatic
  EEPROM lock handling. Reads happen only where documented: `cancel_motion()`, the
  multi-turn setter, and the opt-in readback check. See [write semantics](api.md#write-semantics).
- **Atomic telemetry.** `read_telemetry()` returns every field or fails as a
  whole.
- **Not thread-safe.** Serialize all calls per bus.
- **Hardware status.** Items the code marks as not verified on hardware are
  collected in the [hardware-verification list](api.md#hardware-verification-list).
  This includes `ESPSerialPort`: `lib/platform/esp_idf.cpp` is not built or run
  by the host tests, and its timeout behaviour is not verified on ESP-IDF
  hardware.
