// STSServo register layer: exact transmitted bytes and every error class.
#include <chrono>
#include <cstdio>
#include <memory>

#include "fake_serial_port.h"
#include "servokit/families/stseries/sts_servo.h"

static int g_failures = 0;
#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                               \
    }                                                             \
  } while (0)

using namespace servokit;
using namespace servokit_test;
using std::chrono::milliseconds;

// Builds an STS status frame: FF FF id len status params... checksum.
static Bytes frame(uint8_t id, uint8_t status, Bytes params, bool good_checksum = true, int len_delta = 0) {
  Bytes f = {0xFF, 0xFF, id, (uint8_t)(params.size() + 2 + len_delta), status};
  f.insert(f.end(), params.begin(), params.end());
  unsigned sum = 0;
  for (size_t i = 2; i < f.size(); ++i) sum += f[i];
  const uint8_t cs = (uint8_t)~sum;
  f.push_back(good_checksum ? cs : (uint8_t)(cs ^ 0x5A));
  return f;
}

struct Rig {
  FakeSerialPort port;
  ServoController ctl{&port, milliseconds(10), milliseconds(20)};
  STSServo servo{&ctl, 1};
};

static void test_ping() {
  Rig r;
  r.port.script_reply({0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC});  // manual example
  CHECK(r.servo.probe(1) == Result::OK);
  CHECK(r.port.tx_log().size() == 1);
  CHECK((r.port.tx_log()[0] == Bytes{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}));

  // The Servo overload uses the configured ID.
  r.servo.set_device_id(7);
  r.port.script_reply(frame(7, 0, {}));
  CHECK(r.servo.probe() == Result::OK);
  CHECK((r.port.tx_log()[1] == Bytes{0xFF, 0xFF, 0x07, 0x02, 0x01, 0xF5}));

  // Broadcast and 0xFF are never sent.
  CHECK(r.servo.probe(0xFE) == Result::PARAM);
  CHECK(r.servo.probe(0xFF) == Result::PARAM);
  CHECK(r.port.tx_log().size() == 2);
}

static void test_read() {
  Rig r;
  r.port.script_reply({0xFF, 0xFF, 0x01, 0x04, 0x00, 0x18, 0x05, 0xDD});  // manual example
  uint16_t pos = 0;
  CHECK(r.servo.read_u16(1, STS::REG_POSITION, &pos) == Result::OK);
  CHECK(pos == 0x0518);  // little endian
  CHECK((r.port.tx_log()[0] == Bytes{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE}));

  r.port.script_reply(frame(1, 0, {0x2A}));
  uint8_t v = 0;
  CHECK(r.servo.read_u8(1, STS::REG_TEMPERATURE, &v) == Result::OK && v == 0x2A);
  CHECK((r.port.tx_log()[1] == Bytes{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x3F, 0x01, 0xB8}));

  r.port.script_reply(frame(1, 0, {1, 2, 3}));
  uint8_t buf[3] = {};
  CHECK(r.servo.read_register(1, 0x10, buf, 3) == Result::OK);
  CHECK(buf[0] == 1 && buf[1] == 2 && buf[2] == 3);
  CHECK((r.port.tx_log()[2] == Bytes{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x10, 0x03, 0xE5}));

  // Through the generic interface.
  r.port.script_reply(frame(1, 0, {9, 8}));
  uint8_t raw[2] = {};
  Servo& s = r.servo;
  RegisterAccess* regs = s.get_registers();
  CHECK(regs != nullptr && regs == static_cast<RegisterAccess*>(&r.servo));
  CHECK(regs->read_register(0x38, raw, 2) == Result::OK && raw[0] == 9 && raw[1] == 8);
}

static void test_write() {
  Rig r;
  r.port.script_reply(frame(1, 0, {}));
  CHECK(r.servo.write_u8(1, STS::REG_DEVICE_ID, 2) == Result::OK);
  CHECK((r.port.tx_log()[0] == Bytes{0xFF, 0xFF, 0x01, 0x04, 0x03, 0x05, 0x02, 0xF0}));

  r.port.script_reply(frame(1, 0, {}));
  CHECK(r.servo.write_u16(1, STS::REG_CTRL_POS, 0x0800) == Result::OK);
  CHECK((r.port.tx_log()[1] == Bytes{0xFF, 0xFF, 0x01, 0x05, 0x03, 0x2A, 0x00, 0x08, 0xC4}));

  r.port.script_reply(frame(1, 0, {}));
  const uint8_t data[3] = {1, 2, 3};
  CHECK(r.servo.write_register(1, 0x10, data, 3) == Result::OK);
  CHECK((r.port.tx_log()[2] == Bytes{0xFF, 0xFF, 0x01, 0x06, 0x03, 0x10, 0x01, 0x02, 0x03, 0xDF}));

  r.port.script_reply(frame(1, 0, {}));
  Servo& s = r.servo;
  CHECK(s.get_registers()->write_register(0x28, data, 1) == Result::OK);
  CHECK((r.port.tx_log()[3] == Bytes{0xFF, 0xFF, 0x01, 0x04, 0x03, 0x28, 0x01, 0xCE}));
}

static void test_timeout_and_checksum() {
  uint8_t b[2] = {0xEE, 0xEE};
  {  // timeout
    Rig r;
    r.port.script_timeout();
    CHECK(r.servo.read_register(1, 0x38, b, 2) == Result::TIMEOUT);
    CHECK(b[0] == 0xEE);
  }
  {  // bad checksum: rejected by default and counted; accepted once verification is off
    Rig r;
    r.port.script_reply(frame(1, 0, {1, 2}, false));
    CHECK(r.servo.read_register(1, 0x38, b, 2) == Result::BAD_RESPONSE);
    CHECK(b[0] == 0xEE);
    CHECK(r.servo.get_checksum_errors() == 1);
    SerialLink* s_ptr = static_cast<Servo&>(r.servo).get_serial_link();
    CHECK(s_ptr == static_cast<SerialLink*>(&r.servo));
    SerialLink& s = *s_ptr;
    CHECK(s.get_verify_checksum());
    CHECK(s.set_verify_checksum(false) == Result::OK);
    CHECK(!s.get_verify_checksum() && !r.ctl.get_verify_checksum());
    r.port.script_reply(frame(1, 0, {1, 2}, false));
    CHECK(r.servo.read_register(1, 0x38, b, 2) == Result::OK && b[0] == 1 && b[1] == 2);
    CHECK(s.get_checksum_errors() == 2);
    // Verification off still rejects a bad length.
    r.port.script_reply(frame(1, 0, {1, 2}, false, 1));
    CHECK(r.servo.read_register(1, 0x38, b, 2) == Result::TIMEOUT);
  }
}

static void test_id_and_framing() {
  uint8_t b[2] = {0xEE, 0xEE};
  {  // wrong ID
    Rig r;
    r.port.script_reply(frame(2, 0, {1, 2}));
    CHECK(r.servo.read_register(1, 0x38, b, 2) == Result::BAD_RESPONSE);
    CHECK(b[0] == 0xEE);
  }
  {  // length field larger than the bytes sent: the rest never arrives
    Rig r;
    r.port.script_reply(frame(1, 0, {1, 2}, true, 1));
    CHECK(r.servo.read_register(1, 0x38, b, 2) == Result::TIMEOUT);
  }
  {  // length field smaller than the bytes sent: checksum fails
    Rig r;
    r.port.script_reply(frame(1, 0, {1, 2}, true, -1));
    CHECK(r.servo.read_register(1, 0x38, b, 2) == Result::BAD_RESPONSE);
  }
  {  // valid frame, one data byte short / long
    Rig r;
    r.port.script_reply(frame(1, 0, {1}));
    CHECK(r.servo.read_register(1, 0x38, b, 2) == Result::BAD_RESPONSE);
    r.port.script_reply(frame(1, 0, {1, 2, 3}));
    CHECK(r.servo.read_register(1, 0x38, b, 2) == Result::BAD_RESPONSE);
    CHECK(b[0] == 0xEE);
  }
  {  // write answered with data, ping answered with data
    Rig r;
    r.port.script_reply(frame(1, 0, {1}));
    CHECK(r.servo.write_u8(1, 0x28, 1) == Result::BAD_RESPONSE);
    r.port.script_reply(frame(1, 0, {1}));
    CHECK(r.servo.probe(1) == Result::BAD_RESPONSE);
  }
  {  // truncated header
    Rig r;
    r.port.script_reply({0xFF, 0xFF, 0x01});
    CHECK(r.servo.probe(1) == Result::TIMEOUT);
  }
}

static void test_servo_error_flag() {
  uint8_t b[2] = {0xEE, 0xEE};
  Rig r;
  const uint8_t both = STS::STAT_OVERLOAD | STS::STAT_TEMPERATURE;
  r.port.script_reply(frame(1, both, {1, 2}));
  CHECK(r.servo.read_register(1, 0x38, b, 2) == Result::BAD_RESPONSE);
  CHECK(r.servo.get_last_status() == both);
  CHECK(b[0] == 0xEE);
  r.port.script_reply(frame(1, STS::STAT_VOLTAGE, {}));
  CHECK(r.servo.write_u8(1, 0x28, 1) == Result::BAD_RESPONSE);
  CHECK(r.servo.get_last_status() == STS::STAT_VOLTAGE);
  r.port.script_reply(frame(1, STS::STAT_SENSOR, {}));
  CHECK(r.servo.probe(1) == Result::BAD_RESPONSE);
  // A healthy reply clears the remembered status.
  r.port.script_reply(frame(1, 0, {}));
  CHECK(r.servo.probe(1) == Result::OK && r.servo.get_last_status() == 0);
}

static void test_bus_failure() {
  uint8_t b[2] = {0xEE, 0xEE};
  Rig r;
  r.port.fail_next_write();
  CHECK(r.servo.probe(1) == Result::BUS);
  CHECK(r.port.rx_enabled());
  r.port.script_reply(frame(1, 0, {1, 2}));
  r.port.fail_next_read();
  CHECK(r.servo.read_register(1, 0x38, b, 2) == Result::BUS);
  CHECK(b[0] == 0xEE);
}

static void test_ranges_and_params() {
  uint8_t b[2] = {0xEE, 0xEE};
  Rig r;
  uint8_t big[STSServo::MAX_SPAN + 1] = {};
  CHECK(r.servo.read_register(1, 255, b, 2) == Result::OUT_OF_RANGE);
  CHECK(r.servo.read_register(1, 256, b, 1) == Result::OUT_OF_RANGE);
  CHECK(r.servo.write_register(1, 0xFFFF, big, 1) == Result::OUT_OF_RANGE);
  CHECK(r.servo.read_register(1, 0, big, STSServo::MAX_SPAN + 1) == Result::OUT_OF_RANGE);
  CHECK(r.servo.write_register(1, 0, big, STSServo::MAX_SPAN + 1) == Result::OUT_OF_RANGE);
  CHECK(r.servo.read_u16(1, 255, nullptr) == Result::PARAM);
  CHECK(r.servo.read_register(1, 0, nullptr, 2) == Result::PARAM);
  CHECK(r.servo.read_register(1, 0, b, 0) == Result::PARAM);
  CHECK(r.servo.write_register(1, 0, nullptr, 1) == Result::PARAM);
  CHECK(r.servo.read_register(0xFE, 0, b, 1) == Result::PARAM);
  CHECK(r.servo.write_u8(0xFE, 0, 1) == Result::PARAM);
  CHECK(r.port.tx_log().empty());  // nothing reached the bus

  // The largest span and the last register still work.
  uint8_t span[STSServo::MAX_SPAN] = {};
  r.port.script_reply(frame(1, 0, Bytes(STSServo::MAX_SPAN, 0x5A)));
  CHECK(r.servo.read_register(1, 0, span, STSServo::MAX_SPAN) == Result::OK && span[63] == 0x5A);
  r.port.script_reply(frame(1, 0, {}));
  CHECK(r.servo.write_register(1, 192, span, STSServo::MAX_SPAN) == Result::OK);
  r.port.script_reply(frame(1, 0, {7}));
  uint8_t last = 0;
  CHECK(r.servo.read_u8(1, 255, &last) == Result::OK && last == 7);
}

static void test_no_controller_and_ownership() {
  STSServo none(static_cast<ServoController*>(nullptr), 1);
  CHECK(none.probe(1) == Result::PARAM);
  CHECK(none.set_verify_checksum(false) == Result::PARAM);
  CHECK(none.get_verify_checksum() && none.get_checksum_errors() == 0);

  FakeSerialPort port;
  STSServo owned(std::make_unique<ServoController>(&port, milliseconds(10), milliseconds(20)), 3);
  port.script_reply(frame(3, 0, {}));
  CHECK(owned.probe() == Result::OK);
  CHECK(owned.get_capabilities().features.has(Feature::ABSOLUTE_POSITION));
}

static void test_verify_off_and_default() {
  Rig r;
  CHECK(!r.servo.get_readback_check());
  r.port.script_reply(frame(1, 0, {}));
  CHECK(r.servo.write_u8(1, STS::REG_DEVICE_ID, 2) == Result::OK);
  CHECK(r.port.tx_log().size() == 1);
  CHECK(r.servo.set_readback_check(true) == Result::OK && r.servo.get_readback_check());
  CHECK(r.port.tx_log().size() == 1);  // the setting touches no bus
  CHECK(r.servo.set_readback_check(false) == Result::OK && !r.servo.get_readback_check());
}

static void test_verify_match_and_mismatch() {
  Rig r;
  r.servo.set_readback_check(true);
  r.port.script_reply(frame(1, 0, {}));
  r.port.script_reply(frame(1, 0, {0x2C, 0x01}));
  CHECK(r.servo.write_u16(1, STS::REG_START_TORQUE, 0x012C) == Result::OK);
  CHECK(r.port.tx_log().size() == 2);
  CHECK((r.port.tx_log()[0] == Bytes{0xFF, 0xFF, 0x01, 0x05, 0x03, 0x18, 0x2C, 0x01, 0xB1}));
  CHECK((r.port.tx_log()[1] == Bytes{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x18, 0x02, 0xDE}));

  r.port.script_reply(frame(1, 0, {}));
  r.port.script_reply(frame(1, 0, {0x2D, 0x01}));
  CHECK(r.servo.write_u16(1, STS::REG_START_TORQUE, 0x012C) == Result::VERIFY_FAILED);
  CHECK(r.port.tx_log().size() == 4);

  // Through the RegisterAccess interface.
  Servo& s = r.servo;
  const uint8_t v = 7;
  r.port.script_reply(frame(1, 0, {}));
  r.port.script_reply(frame(1, 0, {7}));
  CHECK(s.get_registers()->get_readback_check());
  CHECK(s.get_registers()->write_register(STS::REG_POS_COEF_P, &v, 1) == Result::OK);
  r.port.script_reply(frame(1, 0, {}));
  r.port.script_reply(frame(1, 0, {8}));
  CHECK(s.get_registers()->write_register(STS::REG_POS_COEF_P, &v, 1) == Result::VERIFY_FAILED);
  CHECK(r.port.tx_log().size() == 8);
}

static void test_verify_failures() {
  Rig r;
  r.servo.set_readback_check(true);
  // Read-back times out: its own error, the write took effect.
  r.port.script_reply(frame(1, 0, {}));
  r.port.script_timeout();
  CHECK(r.servo.write_u8(1, STS::REG_POS_COEF_P, 5) == Result::TIMEOUT);
  CHECK(r.port.tx_log().size() == 2);

  // Read-back reports a servo error status.
  r.port.script_reply(frame(1, 0, {}));
  r.port.script_reply(frame(1, 0x20, {5}));
  CHECK(r.servo.write_u8(1, STS::REG_POS_COEF_P, 5) == Result::BAD_RESPONSE);

  // Read-back transport failure.
  const size_t frames = r.port.tx_log().size();
  r.port.script_reply(frame(1, 0, {}));
  r.port.script_reply(frame(1, 0, {5}));
  r.port.fail_read_after(2);  // the write reply takes two reads; the read-back's first read fails
  CHECK(r.servo.write_u8(1, STS::REG_POS_COEF_P, 5) == Result::BUS);
  CHECK(r.port.tx_log().size() == frames + 2);

  // Write failure: no read frame follows.
  Rig w;
  w.servo.set_readback_check(true);
  w.port.script_timeout();
  CHECK(w.servo.write_u8(1, STS::REG_POS_COEF_P, 5) == Result::TIMEOUT);
  CHECK(w.port.tx_log().size() == 1);
  w.port.fail_next_write();
  CHECK(w.servo.write_u8(1, STS::REG_POS_COEF_P, 5) == Result::BUS);
  CHECK(w.port.tx_log().size() == 1);
}

static void test_verify_masks_and_skips() {
  Rig r;
  r.servo.set_readback_check(true);
  // Bits outside the REG_PHASE mask may differ.
  r.port.script_reply(frame(1, 0, {}));
  r.port.script_reply(frame(1, 0, {(uint8_t)(0x10 ^ 0x08 ^ 0x40)}));
  CHECK(r.servo.write_u8(1, STS::REG_PHASE, 0x10) == Result::OK);
  // A bit inside the mask may not.
  r.port.script_reply(frame(1, 0, {}));
  r.port.script_reply(frame(1, 0, {0x00}));
  CHECK(r.servo.write_u8(1, STS::REG_PHASE, 0x10) == Result::VERIFY_FAILED);
  const size_t before = r.port.tx_log().size();

  // Torque enable (including calibration value 128) is never read back.
  r.port.script_reply(frame(1, 0, {}));
  CHECK(r.servo.write_u8(1, STS::REG_SERVO_EN, 128) == Result::OK);
  r.port.script_reply(frame(1, 0, {}));
  CHECK(r.servo.write_u8(1, STS::REG_SERVO_EN, 1) == Result::OK);

  // Volatile, read-only and unknown registers are written without a read.
  r.port.script_reply(frame(1, 0, {}));
  CHECK(r.servo.write_u16(1, STS::REG_CTRL_POS, 100) == Result::OK);
  r.port.script_reply(frame(1, 0, {}));
  CHECK(r.servo.write_u8(1, STS::REG_CTRL_ACCEL, 5) == Result::OK);
  r.port.script_reply(frame(1, 0, {}));
  CHECK(r.servo.write_u8(1, STS::REG_POSITION, 5) == Result::OK);
  r.port.script_reply(frame(1, 0, {}));
  CHECK(r.servo.write_u8(1, 0xC0, 5) == Result::OK);
  CHECK(r.port.tx_log().size() == before + 6);
}

static void test_verify_span() {
  Rig r;
  r.servo.set_readback_check(true);
  // REG_CTRL_ACCEL and REG_CTRL_POS are volatile: no read.
  const uint8_t vol[3] = {1, 2, 3};
  r.port.script_reply(frame(1, 0, {}));
  CHECK(r.servo.write_register(1, STS::REG_CTRL_ACCEL, vol, 3) == Result::OK);
  CHECK(r.port.tx_log().size() == 1);

  // Four stable bytes from 0x23: one read of the whole span, all compared.
  const uint8_t st[4] = {1, 2, 3, 4};
  r.port.script_reply(frame(1, 0, {}));
  r.port.script_reply(frame(1, 0, {1, 2, 3, 4}));
  CHECK(r.servo.write_register(1, STS::REG_OVERLOAD_TIME, st, 4) == Result::OK);
  CHECK(r.port.tx_log().size() == 3);
  CHECK((r.port.tx_log()[2] == Bytes{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x23, 0x04, 0xD1}));
  r.port.script_reply(frame(1, 0, {}));
  r.port.script_reply(frame(1, 0, {1, 2, 3, 5}));
  CHECK(r.servo.write_register(1, STS::REG_OVERLOAD_TIME, st, 4) == Result::VERIFY_FAILED);

  // 0x27 stable, 0x28 and 0x29 volatile.
  const uint8_t mix[3] = {9, 8, 7};
  r.port.script_reply(frame(1, 0, {}));
  r.port.script_reply(frame(1, 0, {9, 8, 0}));
  CHECK(r.servo.write_register(1, STS::REG_SPEED_COEF_I, mix, 3) == Result::OK);
  r.port.script_reply(frame(1, 0, {}));
  r.port.script_reply(frame(1, 0, {9, 9, 0}));
  CHECK(r.servo.write_register(1, STS::REG_SPEED_COEF_I, mix, 3) == Result::OK);
  r.port.script_reply(frame(1, 0, {}));
  r.port.script_reply(frame(1, 0, {10, 8, 7}));
  CHECK(r.servo.write_register(1, STS::REG_SPEED_COEF_I, mix, 3) == Result::VERIFY_FAILED);
}

static void test_verify_new_device_id() {
  Rig r;
  r.servo.set_readback_check(true);
  r.port.script_reply(frame(1, 0, {}));
  r.port.script_reply(frame(2, 0, {2}));
  CHECK(r.servo.write_u8(1, STS::REG_DEVICE_ID, 2) == Result::OK);
  CHECK((r.port.tx_log()[1] == Bytes{0xFF, 0xFF, 0x02, 0x04, 0x02, 0x05, 0x01, 0xF1}));

  // An ID above STS::MAX_ID cannot be addressed afterwards: no read-back.
  Rig hi;
  hi.servo.set_readback_check(true);
  hi.port.script_reply(frame(1, 0, {}));
  CHECK(hi.servo.write_u8(1, STS::REG_DEVICE_ID, STS::MAX_ID + 1) == Result::OK);
  CHECK(hi.port.tx_log().size() == 1);
  // The highest valid ID is read back.
  hi.port.script_reply(frame(1, 0, {}));
  hi.port.script_reply(frame(STS::MAX_ID, 0, {STS::MAX_ID}));
  CHECK(hi.servo.write_u8(1, STS::REG_DEVICE_ID, STS::MAX_ID) == Result::OK);
  CHECK(hi.port.tx_log().size() == 3);
}

static void test_verify_rule_table() {
  using VC = STSServo::ReadbackClass;
  struct Row {
    uint16_t addr;
    uint8_t bytes;
    VC kind;
    uint8_t mask;
  };
  const Row rows[] = {
      {STS::REG_DEVICE_ID, 1, VC::STABLE, 0xFF},
      {STS::REG_BAUD_RATE, 1, VC::VOLATILE, 0},
      {STS::REG_RESPONSE_DELAY, 1, VC::STABLE, 0xFF},
      {STS::REG_RESPONSE_MODE, 1, VC::STABLE, 0xFF},
      {STS::REG_MIN_ANGLE_THRESH, 2, VC::STABLE, 0xFF},
      {STS::REG_MAX_ANGLE_THRESH, 2, VC::STABLE, 0xFF},
      {STS::REG_OVERTEMP_THRESH, 1, VC::STABLE, 0xFF},
      {STS::REG_OVERVOLT_THRESH, 1, VC::STABLE, 0xFF},
      {STS::REG_UNDERVOLT_THRESH, 1, VC::STABLE, 0xFF},
      {STS::REG_CONTROL_TORQUE_NV, 2, VC::STABLE, 0xFF},
      {STS::REG_PHASE, 1, VC::STABLE_MASKED, STSServo::PHASE_READBACK_MASK},
      {STS::REG_UNLOAD_ALARM_MASK, 1, VC::STABLE, 0xFF},
      {STS::REG_LED_ALARM_MASK, 1, VC::STABLE, 0xFF},
      {STS::REG_POS_COEF_P, 1, VC::STABLE, 0xFF},
      {STS::REG_POS_COEF_D, 1, VC::STABLE, 0xFF},
      {STS::REG_POS_COEF_I, 1, VC::STABLE, 0xFF},
      {STS::REG_START_TORQUE, 2, VC::STABLE, 0xFF},
      {STS::REG_CW_DEADZONE, 1, VC::STABLE, 0xFF},
      {STS::REG_CCW_DEADZONE, 1, VC::STABLE, 0xFF},
      {STS::REG_OVERCURR_THRESH, 2, VC::STABLE, 0xFF},
      {STS::REG_ANGLE_RESOLUTION, 1, VC::STABLE, 0xFF},
      {STS::REG_POS_OFFSET, 2, VC::STABLE, 0xFF},
      {STS::REG_OPERATION_MODE, 1, VC::STABLE, 0xFF},
      {STS::REG_UNLOAD_CTRL_TORQUE, 1, VC::STABLE, 0xFF},
      {STS::REG_OVERLOAD_TIME, 1, VC::STABLE, 0xFF},
      {STS::REG_OVERLOAD_THRESH, 1, VC::STABLE, 0xFF},
      {STS::REG_SPEED_COEF_P, 1, VC::STABLE, 0xFF},
      {STS::REG_OVERCURR_TIME, 1, VC::STABLE, 0xFF},
      {STS::REG_SPEED_COEF_I, 1, VC::STABLE, 0xFF},
      {STS::REG_SERVO_EN, 1, VC::VOLATILE, 0},
      {STS::REG_CTRL_ACCEL, 1, VC::VOLATILE, 0},
      {STS::REG_CTRL_POS, 2, VC::VOLATILE, 0},
      {STS::REG_CTRL_PWM, 2, VC::VOLATILE, 0},
      {STS::REG_CTRL_SPEED, 2, VC::VOLATILE, 0},
      {STS::REG_CTRL_TORQUE, 2, VC::VOLATILE, 0},
      {STS::REG_EEPROM_LOCK, 1, VC::STABLE, 0xFF},
      // Read-only registers and gaps are not compared.
      {STS::REG_VER_MAJ, 1, VC::VOLATILE, 0},
      {STS::REG_MODEL_MAJ, 2, VC::VOLATILE, 0},
      {STS::REG_POSITION, 2, VC::VOLATILE, 0},
      {STS::REG_SPEED, 2, VC::VOLATILE, 0},
      {STS::REG_LOAD, 2, VC::VOLATILE, 0},
      {STS::REG_VOLTAGE, 1, VC::VOLATILE, 0},
      {STS::REG_TEMPERATURE, 1, VC::VOLATILE, 0},
      {STS::REG_STATUS, 1, VC::VOLATILE, 0},
      {STS::REG_MOVE_FLAG, 1, VC::VOLATILE, 0},
      {STS::REG_CURRENT, 2, VC::VOLATILE, 0},
      {0x02, 1, VC::VOLATILE, 0},
      {0x100, 1, VC::VOLATILE, 0},
  };
  for (const Row& row : rows) {
    for (uint8_t i = 0; i < row.bytes; ++i) {
      const STSServo::ReadbackRule v = STSServo::readback_rule(row.addr + i);
      CHECK(v.kind == row.kind && v.mask == row.mask);
    }
  }
  CHECK((STSServo::PHASE_READBACK_MASK & STSServo::PHASE_MULTITURN_BIT) != 0);
}

int main() {
  test_ping();
  test_read();
  test_write();
  test_timeout_and_checksum();
  test_id_and_framing();
  test_servo_error_flag();
  test_bus_failure();
  test_ranges_and_params();
  test_no_controller_and_ownership();
  test_verify_off_and_default();
  test_verify_match_and_mismatch();
  test_verify_failures();
  test_verify_masks_and_skips();
  test_verify_span();
  test_verify_new_device_id();
  test_verify_rule_table();
  if (g_failures) {
    std::printf("%d failure(s)\n", g_failures);
    return 1;
  }
  std::printf("test_sts_servo: all passed\n");
  return 0;
}
