// STSServo typed helpers: register, width, value bytes, range rejection,
// plain single writes, the explicit EEPROM lock and set_persist().
#include <chrono>
#include <cstdio>

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

static Bytes with_checksum(Bytes f) {
  unsigned sum = 0;
  for (size_t i = 2; i < f.size(); ++i) sum += f[i];
  f.push_back((uint8_t)~sum);
  return f;
}

// Status frame from servo 1.
static Bytes reply(Bytes params = {}, uint8_t status = 0) {
  Bytes f = {0xFF, 0xFF, 0x01, (uint8_t)(params.size() + 2), status};
  f.insert(f.end(), params.begin(), params.end());
  return with_checksum(f);
}

// Command frame to servo 1.
static Bytes write_cmd(uint8_t addr, Bytes data) {
  Bytes f = {0xFF, 0xFF, 0x01, (uint8_t)(data.size() + 3), 0x03, addr};
  f.insert(f.end(), data.begin(), data.end());
  return with_checksum(f);
}
static Bytes read_cmd(uint8_t addr, uint8_t len) { return with_checksum({0xFF, 0xFF, 0x01, 0x04, 0x02, addr, len}); }

struct Rig {
  FakeSerialPort port;
  ServoController ctl{&port, milliseconds(10), milliseconds(20)};
  STSServo servo{&ctl, 1};
  size_t sent() const { return port.tx_log().size(); }
  const Bytes& tx(size_t i) const {
    static const Bytes empty;
    return i < port.tx_log().size() ? port.tx_log()[i] : empty;
  }
  void ack() { port.script_reply(reply()); }
  void value(Bytes v) { port.script_reply(reply(v)); }
};

// A plain-write helper: one frame, exact bytes.
static void expect_single_write(Rig& r, Result got, uint8_t addr, Bytes data) {
  CHECK(got == Result::OK);
  CHECK(r.sent() == 1);
  if (r.sent() == 1) CHECK(r.tx(0) == write_cmd(addr, data));
}

static void test_sram_helpers() {
  {
    Rig r;  // servo enable: REG_SERVO_EN (0x28), 1 byte, 0/1 only
    r.ack();
    expect_single_write(r, r.servo.set_servo_enabled(true), 0x28, {1});
    r.ack();
    CHECK(r.servo.set_servo_enabled(false) == Result::OK);
    CHECK(r.tx(1) == write_cmd(0x28, {0}));
  }
  {
    Rig r;  // acceleration: 0x29, 1 byte, 0..254
    r.ack();
    expect_single_write(r, r.servo.set_acceleration(254), 0x29, {254});
    r.ack();
    CHECK(r.servo.set_acceleration(0) == Result::OK && r.tx(1) == write_cmd(0x29, {0}));
    CHECK(r.servo.set_acceleration(255) == Result::OUT_OF_RANGE);
    CHECK(r.sent() == 2);
  }
  {
    Rig r;  // speed: 0x2E, 2 bytes LE, 0..3400
    r.ack();
    expect_single_write(r, r.servo.set_speed(3400), 0x2E, {0x48, 0x0D});
    CHECK(r.servo.set_speed(3401) == Result::OUT_OF_RANGE);
    CHECK(r.servo.set_speed(0xFFFF) == Result::OUT_OF_RANGE);
    CHECK(r.sent() == 1);
  }
  {
    Rig r;  // signed speed: sign-magnitude, bit 15 = reverse
    r.ack();
    CHECK(r.servo.set_signed_speed(-1000) == Result::OK);
    CHECK(r.tx(0) == write_cmd(0x2E, {0xE8, 0x83}));  // 0x8000 | 1000
    r.ack();
    CHECK(r.servo.set_signed_speed(1000) == Result::OK);
    CHECK(r.tx(1) == write_cmd(0x2E, {0xE8, 0x03}));
    r.ack();
    CHECK(r.servo.set_signed_speed(-3400) == Result::OK);
    CHECK(r.tx(2) == write_cmd(0x2E, {0x48, 0x8D}));
    r.ack();
    CHECK(r.servo.set_signed_speed(0) == Result::OK);
    CHECK(r.tx(3) == write_cmd(0x2E, {0x00, 0x00}));
    CHECK(r.servo.set_signed_speed(3401) == Result::OUT_OF_RANGE);
    CHECK(r.servo.set_signed_speed(-3401) == Result::OUT_OF_RANGE);
    CHECK(r.sent() == 4);
  }
  {
    Rig r;  // torque limit: 0x30, 2 bytes LE, 0..1000
    r.ack();
    expect_single_write(r, r.servo.set_torque_limit(1000), 0x30, {0xE8, 0x03});
    CHECK(r.servo.set_torque_limit(1001) == Result::OUT_OF_RANGE);
    CHECK(r.sent() == 1);
  }
}

static void test_target_position() {
  Rig r;  // REG_CTRL_POS 0x2A, 2 bytes LE, sign-magnitude, +-30719
  r.ack();
  CHECK(r.servo.set_target_position(30719) == Result::OK);
  CHECK(r.tx(0) == write_cmd(0x2A, {0xFF, 0x77}));  // 0x77FF
  r.ack();
  CHECK(r.servo.set_target_position(-30719) == Result::OK);
  CHECK(r.tx(1) == write_cmd(0x2A, {0xFF, 0xF7}));  // 0x8000 | 0x77FF
  r.ack();
  CHECK(r.servo.set_target_position(-1) == Result::OK);
  CHECK(r.tx(2) == write_cmd(0x2A, {0x01, 0x80}));
  r.ack();
  CHECK(r.servo.set_target_position(0) == Result::OK);
  CHECK(r.tx(3) == write_cmd(0x2A, {0x00, 0x00}));
  r.ack();
  CHECK(r.servo.set_target_position(2048) == Result::OK);
  CHECK(r.tx(4) == write_cmd(0x2A, {0x00, 0x08}));
  CHECK(r.sent() == 5);

  CHECK(r.servo.set_target_position(30720) == Result::OUT_OF_RANGE);
  CHECK(r.servo.set_target_position(-30720) == Result::OUT_OF_RANGE);
  CHECK(r.servo.set_target_position(65536) == Result::OUT_OF_RANGE);
  CHECK(r.servo.set_target_position(INT32_MIN) == Result::OUT_OF_RANGE);
  CHECK(r.sent() == 5);
}

static void test_eeprom_lock_and_readers() {
  Rig r;
  r.ack();
  CHECK(r.servo.unlock_eeprom() == Result::OK && r.tx(0) == write_cmd(0x37, {0}));
  r.ack();
  CHECK(r.servo.lock_eeprom() == Result::OK && r.tx(1) == write_cmd(0x37, {1}));

  bool b = false;
  r.value({1});
  CHECK(r.servo.get_eeprom_locked(&b) == Result::OK && b && r.tx(2) == read_cmd(0x37, 1));
  r.value({0});
  CHECK(r.servo.get_eeprom_locked(&b) == Result::OK && !b);
  r.value({2});
  CHECK(r.servo.get_eeprom_locked(&b) == Result::BAD_RESPONSE);
  CHECK(r.servo.get_eeprom_locked(nullptr) == Result::PARAM);

  r.value({1});
  CHECK(r.servo.get_servo_enabled(&b) == Result::OK && b && r.tx(5) == read_cmd(0x28, 1));

  STS::op_mode_t m = STS::OP_SERVO;
  r.value({3});
  CHECK(r.servo.get_operating_mode(&m) == Result::OK && m == STS::OP_STEP && r.tx(6) == read_cmd(0x21, 1));
  r.value({4});
  CHECK(r.servo.get_operating_mode(&m) == Result::BAD_RESPONSE && m == STS::OP_STEP);

  STS::rsp_mode_t rm = STS::RSP_READ_PING;
  r.value({1});
  CHECK(r.servo.get_response_mode(&rm) == Result::OK && rm == STS::RSP_ALL && r.tx(8) == read_cmd(0x08, 1));
  uint8_t d = 0;
  r.value({10});
  CHECK(r.servo.get_response_delay(&d) == Result::OK && d == 10 && r.tx(9) == read_cmd(0x07, 1));

  r.value({0x10});
  CHECK(r.servo.get_multiturn_feedback(&b) == Result::OK && b && r.tx(10) == read_cmd(0x12, 1));
  r.value({0x0C});
  CHECK(r.servo.get_multiturn_feedback(&b) == Result::OK && !b);
}

// EEPROM-region setter: exactly one plain write, no pre-read, lock untouched.
static void test_eeprom_plain_write() {
  Rig r;
  r.ack();
  CHECK(r.servo.set_operating_mode(STS::OP_SPEED) == Result::OK);
  CHECK(r.sent() == 1 && r.tx(0) == write_cmd(0x21, {1}));

  // The same value is written again: no compare is made.
  r.ack();
  CHECK(r.servo.set_operating_mode(STS::OP_SPEED) == Result::OK);
  CHECK(r.sent() == 2 && r.tx(1) == write_cmd(0x21, {1}));

  // Write failure is returned as is; still one frame.
  r.port.script_timeout();
  CHECK(r.servo.set_operating_mode(STS::OP_PWM) == Result::TIMEOUT);
  CHECK(r.sent() == 3);

  // Servo error status on the write.
  r.port.script_reply(reply({}, 0x20));
  CHECK(r.servo.set_operating_mode(STS::OP_PWM) == Result::BAD_RESPONSE);
  CHECK(r.sent() == 4);

  // Out-of-range enum value: rejected before any traffic.
  CHECK(r.servo.set_operating_mode(static_cast<STS::op_mode_t>(4)) == Result::OUT_OF_RANGE);
  CHECK(r.servo.set_response_mode(static_cast<STS::rsp_mode_t>(2)) == Result::OUT_OF_RANGE);
  CHECK(r.sent() == 4);
}

// The caller sequences unlock, writes, lock; setters never touch the lock.
static void test_explicit_lock_sequence() {
  Rig r;
  r.ack();
  r.ack();
  r.ack();
  CHECK(r.servo.unlock_eeprom() == Result::OK);
  CHECK(r.servo.set_operating_mode(STS::OP_PWM) == Result::OK);
  CHECK(r.servo.lock_eeprom() == Result::OK);
  CHECK(r.sent() == 3);
  CHECK(r.tx(0) == write_cmd(0x37, {0}));
  CHECK(r.tx(1) == write_cmd(0x21, {2}));
  CHECK(r.tx(2) == write_cmd(0x37, {1}));
}

// set_persist through the generic Servo interface: plain forward to the lock.
static void test_set_persist() {
  Rig r;
  Servo& generic = r.servo;
  r.ack();
  CHECK(generic.set_persist(true) == Result::OK);
  CHECK(r.sent() == 1 && r.tx(0) == write_cmd(0x37, {0}));
  r.ack();
  CHECK(generic.set_persist(false) == Result::OK);
  CHECK(r.sent() == 2 && r.tx(1) == write_cmd(0x37, {1}));
  r.port.script_timeout();
  CHECK(generic.set_persist(true) == Result::TIMEOUT && r.sent() == 3);
  CHECK(r.servo.set_persist(false) == Result::TIMEOUT && r.sent() == 4);  // nothing scripted
}

// One check per EEPROM-region helper: register, width, bytes, boundaries.
static void test_eeprom_helpers() {
  {
    Rig r;  // response mode 0x08
    r.ack();
    CHECK(r.servo.set_response_mode(STS::RSP_READ_PING) == Result::OK);
    CHECK(r.sent() == 1 && r.tx(0) == write_cmd(0x08, {0}));
  }
  {
    Rig r;  // response delay 0x07, 0..254
    r.ack();
    CHECK(r.servo.set_response_delay(254) == Result::OK);
    CHECK(r.sent() == 1 && r.tx(0) == write_cmd(0x07, {254}));
    CHECK(r.servo.set_response_delay(255) == Result::OUT_OF_RANGE && r.sent() == 1);
  }
  {
    Rig r;  // start torque 0x18, 2 bytes, 0..1000
    r.ack();
    CHECK(r.servo.set_start_torque(1000) == Result::OK);
    CHECK(r.sent() == 1 && r.tx(0) == write_cmd(0x18, {0xE8, 0x03}));
    CHECK(r.servo.set_start_torque(1001) == Result::OUT_OF_RANGE && r.sent() == 1);
  }
  {
    Rig r;  // overtemp 0x0D, 0..100
    r.ack();
    CHECK(r.servo.set_overtemp_threshold(100) == Result::OK && r.tx(0) == write_cmd(0x0D, {100}));
    CHECK(r.servo.set_overtemp_threshold(101) == Result::OUT_OF_RANGE && r.sent() == 1);
  }
  {
    Rig r;  // overvoltage 0x0E, undervoltage 0x0F, 0..254
    r.ack();
    r.ack();
    CHECK(r.servo.set_overvoltage_threshold(254) == Result::OK && r.tx(0) == write_cmd(0x0E, {254}));
    CHECK(r.servo.set_undervoltage_threshold(0) == Result::OK && r.tx(1) == write_cmd(0x0F, {0}));
    CHECK(r.servo.set_overvoltage_threshold(255) == Result::OUT_OF_RANGE);
    CHECK(r.servo.set_undervoltage_threshold(255) == Result::OUT_OF_RANGE);
    CHECK(r.sent() == 2);
  }
  {
    Rig r;  // overcurrent threshold 0x1C 2 bytes 0..511, time 0x26 0..254
    r.ack();
    r.ack();
    CHECK(r.servo.set_overcurrent_threshold(511) == Result::OK && r.tx(0) == write_cmd(0x1C, {0xFF, 0x01}));
    CHECK(r.servo.set_overcurrent_threshold(512) == Result::OUT_OF_RANGE && r.sent() == 1);
    CHECK(r.servo.set_overcurrent_time(254) == Result::OK && r.tx(1) == write_cmd(0x26, {254}));
    CHECK(r.servo.set_overcurrent_time(255) == Result::OUT_OF_RANGE && r.sent() == 2);
  }
  {
    Rig r;  // overload threshold 0x24 0..100, time 0x23 0..254
    r.ack();
    r.ack();
    CHECK(r.servo.set_overload_threshold(100) == Result::OK && r.tx(0) == write_cmd(0x24, {100}));
    CHECK(r.servo.set_overload_threshold(101) == Result::OUT_OF_RANGE);
    CHECK(r.servo.set_overload_time(254) == Result::OK && r.tx(1) == write_cmd(0x23, {254}));
    CHECK(r.servo.set_overload_time(255) == Result::OUT_OF_RANGE);
    CHECK(r.sent() == 2);
  }
  {
    Rig r;  // alarm masks: LED 0x14, unload 0x13, bits 0..5 only
    r.ack();
    r.ack();
    const uint8_t mask = STS::STAT_TEMPERATURE | STS::STAT_OVERLOAD;
    CHECK(r.servo.set_led_alarm_mask(mask) == Result::OK && r.tx(0) == write_cmd(0x14, {0x24}));
    CHECK(r.servo.set_unload_alarm_mask(0x3F) == Result::OK && r.tx(1) == write_cmd(0x13, {0x3F}));
    CHECK(r.servo.set_led_alarm_mask(0x40) == Result::OUT_OF_RANGE);
    CHECK(r.servo.set_unload_alarm_mask(0x80) == Result::OUT_OF_RANGE);
    CHECK(r.sent() == 2);
  }
}

// REG_PHASE: read the byte, change only bit 4, write it back.
static void test_multiturn_bit() {
  Rig r;
  r.value({0x0C});
  r.ack();
  CHECK(r.servo.set_multiturn_feedback(true) == Result::OK);
  CHECK(r.sent() == 2 && r.tx(0) == read_cmd(0x12, 1) && r.tx(1) == write_cmd(0x12, {0x1C}));

  r.value({0xBC});  // bit 4 set among others
  r.ack();
  CHECK(r.servo.set_multiturn_feedback(false) == Result::OK);
  CHECK(r.sent() == 4 && r.tx(3) == write_cmd(0x12, {0xAC}));

  r.port.script_timeout();  // read fails: no write follows
  CHECK(r.servo.set_multiturn_feedback(true) == Result::TIMEOUT && r.sent() == 5);

  r.value({0x0C});  // write fails
  r.port.script_timeout();
  CHECK(r.servo.set_multiturn_feedback(true) == Result::TIMEOUT && r.sent() == 7);
}

static void test_no_controller() {
  STSServo s(static_cast<ServoController*>(nullptr), 1);
  CHECK(s.set_target_position(0) == Result::PARAM);
  CHECK(s.set_operating_mode(STS::OP_SERVO) == Result::PARAM);
}

int main() {
  test_sram_helpers();
  test_target_position();
  test_eeprom_lock_and_readers();
  test_eeprom_plain_write();
  test_explicit_lock_sequence();
  test_set_persist();
  test_eeprom_helpers();
  test_multiturn_bit();
  test_no_controller();
  if (g_failures) {
    std::printf("%d failure(s)\n", g_failures);
    return 1;
  }
  std::printf("test_sts_family: all passed\n");
  return 0;
}
