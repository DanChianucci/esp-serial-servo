// Tier 3: register access, with no unit conversion. Use it for registers
// the typed helpers do not cover.
#include <chrono>
#include <cstdio>

#include "example_support.h"
#include "servokit.h"

using namespace servokit;
using namespace example;
using std::chrono::milliseconds;

int main() {
  servokit_test::FakeSerialPort port;
  // Firmware 3.7, model 9.3, device ID 1 (six bytes from address 0x00).
  port.script_reply(reply(1, {0x03, 0x07, 0x00, 0x09, 0x03, 0x01}));
  port.script_reply(reply(7));  // write to servo 7
  port.script_reply(reply(1, {0x2A}));
  port.script_reply(reply(1));

  ServoController controller(&port, milliseconds(20), milliseconds(20));
  STSServo sts(&controller, 1);

  // Family API: the servo ID is passed on each call, so one object can
  // address any servo on the bus.
  uint8_t identity[6] = {};
  EXPECT(sts.read_register(1, STS::REG_VER_MAJ, identity, sizeof(identity)) == Result::OK);
  std::printf("firmware %u.%u\n", static_cast<unsigned>(identity[0]), static_cast<unsigned>(identity[1]));

  // 16-bit registers are little endian; write_u16 handles the byte order.
  EXPECT(sts.write_u16(7, STS::REG_CTRL_SPEED, 1000) == Result::OK);

  // Generic API: register access to the servo selected by get_device_id(),
  // through Servo::get_registers(). Addresses and meaning are family-defined.
  Servo& servo = sts;
  RegisterAccess* regs = servo.get_registers();
  EXPECT(regs != nullptr);  // non-null because the servo sets Feature::REGISTER_ACCESS
  uint8_t value = 0;
  EXPECT(regs->read_register(STS::REG_TEMPERATURE, &value, 1) == Result::OK);
  EXPECT(value == 0x2A);
  const uint8_t accel = 10;  // units of 100 steps/s^2
  EXPECT(regs->write_register(STS::REG_CTRL_ACCEL, &accel, 1) == Result::OK);

  // Spans outside the control table are rejected before anything is sent.
  EXPECT(regs->read_register(0xFF, &value, 2) == Result::OUT_OF_RANGE);
  EXPECT(port.tx_log().size() == 4);
  return failures() == 0 ? 0 : 1;
}
