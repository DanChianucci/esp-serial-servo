// STSServo register layer: exact transmitted bytes and every error class.
#include <chrono>
#include <cstdio>
#include <memory>

#include "fake_serial_port.h"
#include "servokit/stseries/sts_servo.h"

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
  CHECK(r.servo.ping(1) == Result::OK);
  CHECK(r.port.tx_log().size() == 1);
  CHECK((r.port.tx_log()[0] == Bytes{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}));

  // The Servo overload uses the configured ID.
  r.servo.set_device_id(7);
  r.port.script_reply(frame(7, 0, {}));
  CHECK(r.servo.ping() == Result::OK);
  CHECK((r.port.tx_log()[1] == Bytes{0xFF, 0xFF, 0x07, 0x02, 0x01, 0xF5}));

  // Broadcast and 0xFF are never sent.
  CHECK(r.servo.ping(0xFE) == Result::PARAM);
  CHECK(r.servo.ping(0xFF) == Result::PARAM);
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
  CHECK(s.read_raw(0x38, raw, 2) == Result::OK && raw[0] == 9 && raw[1] == 8);
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
  CHECK(s.write_raw(0x28, data, 1) == Result::OK);
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
    CHECK(r.servo.checksum_errors() == 1);
    Servo& s = r.servo;
    CHECK(s.verify_checksum());
    CHECK(s.set_verify_checksum(false) == Result::OK);
    CHECK(!s.verify_checksum() && !r.ctl.verify_checksum());
    r.port.script_reply(frame(1, 0, {1, 2}, false));
    CHECK(r.servo.read_register(1, 0x38, b, 2) == Result::OK && b[0] == 1 && b[1] == 2);
    CHECK(s.checksum_errors() == 2);
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
    CHECK(r.servo.ping(1) == Result::BAD_RESPONSE);
  }
  {  // truncated header
    Rig r;
    r.port.script_reply({0xFF, 0xFF, 0x01});
    CHECK(r.servo.ping(1) == Result::TIMEOUT);
  }
}

static void test_servo_error_flag() {
  uint8_t b[2] = {0xEE, 0xEE};
  Rig r;
  const uint8_t both = STS::STAT_OVERLOAD | STS::STAT_TEMPERATURE;
  r.port.script_reply(frame(1, both, {1, 2}));
  CHECK(r.servo.read_register(1, 0x38, b, 2) == Result::BAD_RESPONSE);
  CHECK(r.servo.last_status() == both);
  CHECK(b[0] == 0xEE);
  r.port.script_reply(frame(1, STS::STAT_VOLTAGE, {}));
  CHECK(r.servo.write_u8(1, 0x28, 1) == Result::BAD_RESPONSE);
  CHECK(r.servo.last_status() == STS::STAT_VOLTAGE);
  r.port.script_reply(frame(1, STS::STAT_SENSOR, {}));
  CHECK(r.servo.ping(1) == Result::BAD_RESPONSE);
  // A healthy reply clears the remembered status.
  r.port.script_reply(frame(1, 0, {}));
  CHECK(r.servo.ping(1) == Result::OK && r.servo.last_status() == 0);
}

static void test_bus_failure() {
  uint8_t b[2] = {0xEE, 0xEE};
  Rig r;
  r.port.fail_next_write();
  CHECK(r.servo.ping(1) == Result::BUS);
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
  CHECK(none.ping(1) == Result::PARAM);
  CHECK(none.set_verify_checksum(false) == Result::PARAM);
  CHECK(none.verify_checksum() && none.checksum_errors() == 0);

  FakeSerialPort port;
  STSServo owned(std::make_unique<ServoController>(&port, milliseconds(10), milliseconds(20)), 3);
  port.script_reply(frame(3, 0, {}));
  CHECK(owned.ping() == Result::OK);
  CHECK(owned.capabilities().features.has(Feature::ABSOLUTE_POSITION));
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
  if (g_failures) {
    std::printf("%d failure(s)\n", g_failures);
    return 1;
  }
  std::printf("test_sts_servo: all passed\n");
  return 0;
}
