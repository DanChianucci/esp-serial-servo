// Tier 1: family-neutral control through Servo&.
// The application code below (drive()) names no servo family; only main()
// knows that an STS servo sits behind the interface.
#include <chrono>
#include <cstdio>

#include "example_support.h"
#include "servokit.h"

using namespace servokit;
using namespace example;
using std::chrono::milliseconds;

// Works with any Servo. Every call returns a Result; nothing throws.
static Result drive(Servo& servo) {
  // Features are reported by the servo itself and are never emulated: ask
  // before relying on an operation.
  const Capabilities caps = servo.capabilities();
  if (!caps.features.has(Feature::ABSOLUTE_POSITION) || !caps.features.has(Feature::READ_POSITION)) {
    return Result::UNSUPPORTED;
  }

  // Put the servo into absolute-target mode.
  Result r = servo.prepare();
  if (r != Result::OK) return r;

  // Speed, acceleration and torque for the moves that follow. Values are
  // clamped to caps.limits by the implementation.
  Profile profile;
  profile.speed_mdeg_s = 90000;  // 90 deg/s
  profile.accel_mdeg_s2 = 100000;
  profile.torque_permille = 500;
  r = servo.set_profile(profile);
  if (r != Result::OK) return r;

  r = servo.move_to(45000);  // 45 deg
  if (r != Result::OK) return r;

  // One atomic sample: every field is filled or the call fails as a whole.
  Telemetry t;
  r = servo.read_telemetry(&t);
  if (r != Result::OK) return r;
  char alarms[64];
  t.alarms.to_text(alarms, sizeof(alarms));
  std::printf("position %d mdeg, current %d mA, %d C, alarms %s\n", static_cast<int>(t.position_mdeg),
              static_cast<int>(t.current_ma), static_cast<int>(t.temperature_c), alarms);
  if (t.voltage_valid) std::printf("supply %u mV\n", static_cast<unsigned>(t.voltage_mv));

  // Stop where the servo is now (read position, then command it).
  return servo.hold();
}

int main() {
  // A scripted fake stands in for the UART: one reply per command, in order.
  servokit_test::FakeSerialPort port;
  const Bytes ack = reply(1);
  for (int i = 0; i < 2 + 4 + 1; ++i) port.script_reply(ack);  // prepare, set_profile, move_to
  // Telemetry from register 0x38: position 512 steps, speed 0, load 100,
  // 120 (12.0 V), 31 C, no alarm, not moving, current 4 units (26 mA).
  port.script_reply(reply(1, {0x00, 0x02, 0x00, 0x00, 0x64, 0x00, 120, 31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00}));
  port.script_reply(reply(1, {0x00, 0x02}));  // hold: position read
  port.script_reply(ack);                     // hold: target write

  // The controller bounds every transaction: 20 ms to send plus 20 ms to
  // receive a reply. The STSServo borrows the controller, which borrows the port.
  ServoController controller(&port, milliseconds(20), milliseconds(20));
  STSServo sts(&controller, 1);

  Servo& servo = sts;  // from here on the code is family-neutral
  const Result r = drive(servo);
  EXPECT(r == Result::OK);
  EXPECT(port.tx_log().size() == 10);
  return failures() == 0 ? 0 : 1;
}
