// Generic Servo implemented on STSServo: capabilities, unit helpers, exact
// frames of every operation (including the absence of pre-reads), telemetry
// decode, alarm mapping, failure handling and the shared conformance suite.
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include "conformance/servo_conformance.h"
#include "fake_serial_port.h"
#include "servokit/families/stseries/sts_servo.h"
#include "servokit/families/stseries/sts_units.h"

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
namespace u = servokit::STS::units;

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
static Bytes write_cmd(uint8_t addr, Bytes data) {
  Bytes f = {0xFF, 0xFF, 0x01, (uint8_t)(data.size() + 3), 0x03, addr};
  f.insert(f.end(), data.begin(), data.end());
  return with_checksum(f);
}
static Bytes read_cmd(uint8_t addr, uint8_t len) { return with_checksum({0xFF, 0xFF, 0x01, 0x04, 0x02, addr, len}); }

// Feedback block REG_POSITION..REG_CURRENT (15 bytes).
struct Feedback {
  uint16_t pos = 0x0800;
  uint16_t speed = 0;
  uint16_t load = 0;
  uint8_t volt = 120;
  uint8_t temp = 30;
  uint8_t status = 0;
  uint8_t move = 0;
  uint16_t cur = 0;
  Bytes bytes() const {
    return {(uint8_t)pos,  (uint8_t)(pos >> 8), (uint8_t)speed, (uint8_t)(speed >> 8), (uint8_t)load,
            (uint8_t)(load >> 8), volt, temp, 0, status, move, 0, 0, (uint8_t)cur, (uint8_t)(cur >> 8)};
  }
};

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
  void acks(int n) {
    for (int i = 0; i < n; ++i) ack();
  }
  void value(Bytes v, uint8_t status = 0) { port.script_reply(reply(v, status)); }
};

static void test_units() {
  CHECK(u::steps_to_mdeg(0) == 0 && u::steps_to_mdeg(1) == 88 && u::steps_to_mdeg(-1) == -88);
  CHECK(u::steps_to_mdeg(4096) == 360000 && u::steps_to_mdeg(2048) == 180000);
  CHECK(u::steps_to_mdeg(30719) == 2699912 && u::steps_to_mdeg(-30719) == -2699912);
  CHECK(u::mdeg_to_steps(88) == 1 && u::mdeg_to_steps(44) == 1 && u::mdeg_to_steps(43) == 0);
  CHECK(u::mdeg_to_steps(-44) == -1 && u::mdeg_to_steps(-43) == 0 && u::mdeg_to_steps(360000) == 4096);
  CHECK(u::mdeg_to_steps(INT32_MAX) > 0 && u::mdeg_to_steps(INT32_MIN) < 0);
  CHECK(u::steps_s_to_mdeg_s(3400) == 298828 && u::mdeg_s_to_steps_s(298828) == 3400);
  CHECK(u::mdeg_s_to_steps_s(0) == 0 && u::mdeg_s_to_steps_s(87891) == 1000);
  CHECK(u::accel_units_to_mdeg_s2(1) == 8789 && u::accel_units_to_mdeg_s2(254) == 2232422);
  CHECK(u::mdeg_s2_to_accel_units(8789) == 1 && u::mdeg_s2_to_accel_units(87891) == 10);
  CHECK(u::mdeg_s2_to_accel_units(4394) == 0 && u::mdeg_s2_to_accel_units(4395) == 1);
  CHECK(u::current_units_to_ma(0) == 0 && u::current_units_to_ma(2) == 13 && u::current_units_to_ma(100) == 650);
  CHECK(u::current_units_to_ma(1) == 7 && u::current_units_to_ma(-1) == -7 && u::current_units_to_ma(500) == 3250);
  CHECK(u::ma_to_current_units(1000) == 154 && u::ma_to_current_units(13) == 2 && u::ma_to_current_units(0) == 0);
  CHECK(u::ma_to_current_units(3322) == 511 && u::ma_to_current_units(3329) == 512);
  CHECK(u::voltage_units_to_mv(0) == 0 && u::voltage_units_to_mv(123) == 12300 && u::voltage_units_to_mv(255) == 25500);
  CHECK(u::ms_to_time_units(500) == 50 && u::ms_to_time_units(4) == 0 && u::ms_to_time_units(5) == 1);

  CHECK(u::decode_position_steps(0) == 0 && u::decode_position_steps(0x8000) == 0);
  CHECK(u::decode_position_steps(0x0800) == 2048 && u::decode_position_steps(0x8001) == -1);
  CHECK(u::decode_position_steps(0x7FFF) == 32767 && u::decode_position_steps(0xFFFF) == -32767);
  CHECK(u::decode_current_units(0x8002) == -2 && u::decode_current_units(0x01F4) == 500);
  int16_t v = 77;
  CHECK(u::decode_load_permille(0, &v) && v == 0);
  CHECK(u::decode_load_permille(0x3E8, &v) && v == 1000);
  CHECK(u::decode_load_permille(0x7E8, &v) && v == -1000);
  CHECK(u::decode_load_permille(0x432, &v) && v == -50);
  CHECK(u::decode_load_permille(0x400, &v) && v == 0);
  v = 77;
  CHECK(!u::decode_load_permille(0x3E9, &v) && !u::decode_load_permille(0x800, &v) && !u::decode_load_permille(0x7FF, &v));
  CHECK(!u::decode_load_permille(0x8000, &v) && v == 77);
}

static void test_alarm_table() {
  const AlarmSet volt = Alarm::OVER_VOLTAGE | Alarm::UNDER_VOLTAGE;
  CHECK(u::alarms_from_native(0).none());
  CHECK(u::alarms_from_native(STS::STAT_VOLTAGE) == volt);
  CHECK(u::alarms_from_native(STS::STAT_SENSOR) == AlarmSet(Alarm::ENCODER));
  CHECK(u::alarms_from_native(STS::STAT_TEMPERATURE) == AlarmSet(Alarm::OVER_TEMPERATURE));
  CHECK(u::alarms_from_native(STS::STAT_CURRENT) == AlarmSet(Alarm::OVER_CURRENT));
  CHECK(u::alarms_from_native(STS::STAT_ANGLE) == AlarmSet(Alarm::ANGLE_LIMIT));
  CHECK(u::alarms_from_native(STS::STAT_OVERLOAD) == AlarmSet(Alarm::OVERLOAD));
  CHECK(u::alarms_from_native(0x40) == AlarmSet(Alarm::OTHER));
  CHECK(u::alarms_from_native(0x80) == AlarmSet(Alarm::OTHER));
  CHECK(u::alarms_from_native(0x24) == (Alarm::OVER_TEMPERATURE | Alarm::OVERLOAD));
  // No set native bit is ever dropped; OTHER appears exactly for bits 6 and 7.
  for (unsigned s = 0; s < 256; ++s) {
    const AlarmSet a = u::alarms_from_native((uint8_t)s);
    CHECK(a.any() == (s != 0));
    CHECK(a.has(Alarm::OTHER) == ((s & 0xC0) != 0));
    for (unsigned bit = 0; bit < 6; ++bit) {
      if (s & (1u << bit)) CHECK(a.has_any(u::ALARM_TABLE[bit].generic));
    }
  }
  CHECK(u::reportable_alarms().has_all(volt | Alarm::ENCODER | Alarm::OVER_TEMPERATURE | Alarm::OVER_CURRENT |
                                       Alarm::ANGLE_LIMIT | Alarm::OVERLOAD | Alarm::OTHER));
  CHECK(!u::reportable_alarms().has_any(Alarm::STALLED | Alarm::COMMUNICATION | Alarm::UNLOADED));

  uint8_t m = 0xAA;
  CHECK(u::native_from_alarms(AlarmSet(), &m) && m == 0);
  CHECK(u::native_from_alarms(Alarm::OVER_VOLTAGE, &m) && m == STS::STAT_VOLTAGE);
  CHECK(u::native_from_alarms(Alarm::UNDER_VOLTAGE, &m) && m == STS::STAT_VOLTAGE);
  CHECK(u::native_from_alarms(volt | Alarm::OVERLOAD | Alarm::ANGLE_LIMIT, &m) && m == 0x31);
  m = 0xAA;
  CHECK(!u::native_from_alarms(Alarm::STALLED, &m) && m == 0xAA);
  CHECK(!u::native_from_alarms(Alarm::OVERLOAD | Alarm::OTHER, &m));
  CHECK(!u::native_from_alarms(Alarm::COMMUNICATION, &m) && !u::native_from_alarms(Alarm::UNLOADED, &m));
}

static void test_capabilities() {
  Rig r;
  const Capabilities c = r.servo.get_capabilities();
  for (Feature f : {Feature::ABSOLUTE_POSITION, Feature::SET_SPEED, Feature::SET_ACCEL, Feature::TORQUE_LIMIT,
                    Feature::START_TORQUE, Feature::TORQUE_ENABLE, Feature::HOLD, Feature::READ_POSITION,
                    Feature::READ_CURRENT, Feature::READ_LOAD, Feature::READ_VOLTAGE, Feature::READ_TEMPERATURE,
                    Feature::READ_MOVING, Feature::READ_ALARMS, Feature::PROTECT_CURRENT, Feature::PROTECT_TEMP,
                    Feature::PROTECT_RELEASE_ON_ALARM, Feature::PING, Feature::READ_IDENTITY,
                    Feature::REGISTER_ACCESS, Feature::PERSIST_CONFIG, Feature::READBACK_CHECK}) {
    CHECK(c.features.has(f));
  }
  // Unverified or not offered through the generic API.
  for (Feature f : {Feature::MULTI_TURN_POS, Feature::RETAINS_POSITION_UNPOWERED, Feature::RELATIVE_POSITION,
                    Feature::CONTINUOUS_ROTATION, Feature::PWM_DRIVE, Feature::CURRENT_LIMIT, Feature::READ_SPEED,
                    Feature::PROTECT_OVERLOAD, Feature::PROTECT_VOLTAGE, Feature::SET_ID, Feature::SET_BAUD,
                    Feature::SYNC_WRITE, Feature::SYNC_READ, Feature::ASYNC_WRITE, Feature::BROADCAST,
                    Feature::POSITION_GAINS, Feature::DEADZONE, Feature::ANGLE_LIMITS, Feature::POSITION_OFFSET}) {
    CHECK(!c.features.has(f));
  }
  CHECK(c.limits.angle_min_mdeg == -2699912 && c.limits.angle_max_mdeg == 2699912);
  CHECK(c.limits.resolution_mdeg == 88 && c.limits.max_speed_mdeg_s == 298828);
  CHECK(c.limits.max_accel_mdeg_s2 == 2232422 && c.limits.max_torque_permille == 1000);
  CHECK(c.limits.max_torque_mnm == STSServo::STALL_TORQUE_MNM && c.limits.max_torque_mnm == 1912);
  CHECK(c.limits.current_min_ma == -212986 && c.limits.current_max_ma == 212986);
  CHECK(c.limits.current_max_ma == u::current_units_to_ma(u::decode_current_units(0x7FFF)));
  CHECK(c.limits.current_min_ma == u::current_units_to_ma(u::decode_current_units(0xFFFF)));
  CHECK(c.limits.reportable_alarms == u::reportable_alarms());
  CHECK(r.sent() == 0);
  // Operations without a feature stay unsupported.
  CHECK(r.servo.clear_alarms() == Result::UNSUPPORTED);
  CHECK(r.sent() == 0);
}

static void test_move_to() {
  Rig r;
  r.ack();
  CHECK(r.servo.move_to(0) == Result::OK && r.tx(0) == write_cmd(0x2A, {0x00, 0x00}));
  r.ack();
  CHECK(r.servo.move_to(87891) == Result::OK && r.tx(1) == write_cmd(0x2A, {0xE8, 0x03}));
  r.ack();
  CHECK(r.servo.move_to(-87891) == Result::OK && r.tx(2) == write_cmd(0x2A, {0xE8, 0x83}));
  r.ack();
  CHECK(r.servo.move_to(44) == Result::OK && r.tx(3) == write_cmd(0x2A, {0x01, 0x00}));
  r.ack();
  CHECK(r.servo.move_to(43) == Result::OK && r.tx(4) == write_cmd(0x2A, {0x00, 0x00}));
  r.ack();
  CHECK(r.servo.move_to(2699912) == Result::OK && r.tx(5) == write_cmd(0x2A, {0xFF, 0x77}));
  r.ack();
  CHECK(r.servo.move_to(-2699912) == Result::OK && r.tx(6) == write_cmd(0x2A, {0xFF, 0xF7}));
  CHECK(r.sent() == 7);
  CHECK(r.servo.move_to(2699913) == Result::OUT_OF_RANGE);
  CHECK(r.servo.move_to(-2699913) == Result::OUT_OF_RANGE);
  CHECK(r.servo.move_to(INT32_MAX) == Result::OUT_OF_RANGE && r.servo.move_to(INT32_MIN) == Result::OUT_OF_RANGE);
  CHECK(r.sent() == 7);
  r.port.script_timeout();
  CHECK(r.servo.move_to(0) == Result::TIMEOUT && r.sent() == 8);
}

static void test_identity() {
  Rig r;
  r.value({3, 7, 0, 9, 3, 1});
  Identity id;
  id.model_major = 99;
  CHECK(r.servo.identify(&id) == Result::OK);
  CHECK(r.sent() == 1 && r.tx(0) == read_cmd(0x00, 6));
  CHECK(std::string(id.family) == "STS");
  CHECK(id.firmware_major == 3 && id.firmware_minor == 7);
  CHECK(id.model_major == 9 && id.model_minor == 3 && id.device_id == 1);
  Identity untouched;
  untouched.model_major = 99;
  r.value({3, 7, 0, 9, 3});  // one byte short
  CHECK(r.servo.identify(&untouched) == Result::BAD_RESPONSE && untouched.model_major == 99);
  r.port.script_timeout();
  CHECK(r.servo.identify(&untouched) == Result::TIMEOUT && untouched.model_major == 99);
  CHECK(r.servo.identify(nullptr) == Result::PARAM && r.sent() == 3);
}

static void test_prepare() {
  {
    Rig r;
    r.acks(2);
    CHECK(r.servo.prepare() == Result::OK);
    CHECK(r.sent() == 2);  // two plain writes, no read first
    CHECK(r.tx(0) == write_cmd(0x08, {1}));  // RSP_ALL
    CHECK(r.tx(1) == write_cmd(0x21, {0}));  // OP_SERVO
  }
  {
    Rig r;  // first write unanswered: reported, second write not sent
    r.port.script_timeout();
    CHECK(r.servo.prepare() == Result::TIMEOUT && r.sent() == 1);
  }
  {
    Rig r;
    r.ack();
    r.port.script_timeout();
    CHECK(r.servo.prepare() == Result::TIMEOUT && r.sent() == 2);
  }
}

static void test_set_protection() {
  Protection p;
  p.stall_current_ma = 1000;
  p.stall_time_ms = 500;
  p.max_temperature_c = 70;
  p.release_on_alarm = Alarm::OVER_TEMPERATURE | Alarm::OVER_CURRENT;
  {
    Rig r;
    r.acks(4);
    CHECK(r.servo.set_protection(p) == Result::OK);
    CHECK(r.sent() == 4);  // four plain writes, no reads
    CHECK(r.tx(0) == write_cmd(0x1C, {154, 0}));
    CHECK(r.tx(1) == write_cmd(0x26, {50}));
    CHECK(r.tx(2) == write_cmd(0x0D, {70}));
    CHECK(r.tx(3) == write_cmd(0x13, {0x0C}));
    // The EEPROM lock register is never written.
    for (const Bytes& f : r.port.tx_log()) CHECK(f[5] != 0x37);
  }
  {
    Rig r;  // boundaries
    r.acks(4);
    p.stall_current_ma = 3322;
    p.stall_time_ms = 2540;
    p.max_temperature_c = 100;
    p.release_on_alarm = Alarm::OVER_VOLTAGE | Alarm::ENCODER | Alarm::OVER_TEMPERATURE | Alarm::OVER_CURRENT |
                         Alarm::ANGLE_LIMIT | Alarm::OVERLOAD;
    CHECK(r.servo.set_protection(p) == Result::OK);
    CHECK(r.tx(0) == write_cmd(0x1C, {0xFF, 0x01}) && r.tx(1) == write_cmd(0x26, {254}));
    CHECK(r.tx(2) == write_cmd(0x0D, {100}) && r.tx(3) == write_cmd(0x13, {0x3F}));
  }
  {
    Rig r;  // zero is written as given; either voltage alarm selects the voltage bit
    r.acks(4);
    p = Protection{};
    p.release_on_alarm = Alarm::UNDER_VOLTAGE;
    CHECK(r.servo.set_protection(p) == Result::OK);
    CHECK(r.tx(0) == write_cmd(0x1C, {0, 0}) && r.tx(1) == write_cmd(0x26, {0}));
    CHECK(r.tx(2) == write_cmd(0x0D, {0}) && r.tx(3) == write_cmd(0x13, {0x01}));
  }
  {
    Rig r;  // out-of-range values send nothing
    Protection bad;
    bad.stall_current_ma = 3329;
    CHECK(r.servo.set_protection(bad) == Result::OUT_OF_RANGE);
    bad = Protection{};
    bad.stall_time_ms = 2600;
    CHECK(r.servo.set_protection(bad) == Result::OUT_OF_RANGE);
    bad = Protection{};
    bad.max_temperature_c = 101;
    CHECK(r.servo.set_protection(bad) == Result::OUT_OF_RANGE);
    bad.max_temperature_c = -1;
    CHECK(r.servo.set_protection(bad) == Result::OUT_OF_RANGE);
    for (Alarm a : {Alarm::STALLED, Alarm::COMMUNICATION, Alarm::UNLOADED, Alarm::OTHER}) {
      bad = Protection{};
      bad.release_on_alarm = Alarm::OVERLOAD | a;
      CHECK(r.servo.set_protection(bad) == Result::OUT_OF_RANGE);
    }
    CHECK(r.sent() == 0);
  }
  {
    Rig r;  // stops at the first failed write
    p = Protection{};
    r.ack();
    r.ack();
    r.port.script_timeout();
    CHECK(r.servo.set_protection(p) == Result::TIMEOUT && r.sent() == 3);
  }
}

static void test_set_profile() {
  {
    Rig r;
    r.acks(4);
    Profile p;
    p.speed_mdeg_s = 87891;
    p.accel_mdeg_s2 = 87891;
    p.torque_permille = 500;
    p.start_torque_permille = 100;
    CHECK(r.servo.set_profile(p) == Result::OK);
    CHECK(r.sent() == 4);  // four plain writes, no reads
    CHECK(r.tx(0) == write_cmd(0x30, {0xF4, 0x01}));
    CHECK(r.tx(1) == write_cmd(0x18, {0x64, 0x00}));
    CHECK(r.tx(2) == write_cmd(0x29, {10}));
    CHECK(r.tx(3) == write_cmd(0x2E, {0xE8, 0x03}));
  }
  {
    Rig r;  // clamped to the family limits
    r.acks(4);
    Profile p;
    p.speed_mdeg_s = UINT32_MAX;
    p.accel_mdeg_s2 = UINT32_MAX;
    p.torque_permille = 65535;
    p.start_torque_permille = 1001;
    CHECK(r.servo.set_profile(p) == Result::OK);
    CHECK(r.tx(0) == write_cmd(0x30, {0xE8, 0x03}) && r.tx(1) == write_cmd(0x18, {0xE8, 0x03}));
    CHECK(r.tx(2) == write_cmd(0x29, {254}) && r.tx(3) == write_cmd(0x2E, {0x48, 0x0D}));
  }
  {
    Rig r;  // zero stays zero
    r.acks(4);
    CHECK(r.servo.set_profile(Profile{}) == Result::OK);
    CHECK(r.tx(0) == write_cmd(0x30, {0, 0}) && r.tx(1) == write_cmd(0x18, {0, 0}));
    CHECK(r.tx(2) == write_cmd(0x29, {0}) && r.tx(3) == write_cmd(0x2E, {0, 0}));
  }
  {
    Rig r;  // stops at the first failed write
    r.ack();
    r.port.script_timeout();
    CHECK(r.servo.set_profile(Profile{}) == Result::TIMEOUT && r.sent() == 2);
  }
}

static void test_torque_and_persist() {
  Rig r;
  r.acks(4);
  CHECK(r.servo.set_torque(true) == Result::OK && r.tx(0) == write_cmd(0x28, {1}));
  CHECK(r.servo.set_torque(false) == Result::OK && r.tx(1) == write_cmd(0x28, {0}));
  CHECK(r.servo.set_persist(true) == Result::OK && r.tx(2) == write_cmd(0x37, {0}));
  CHECK(r.servo.set_persist(false) == Result::OK && r.tx(3) == write_cmd(0x37, {1}));
  CHECK(r.sent() == 4);
}

static void test_hold() {
  {
    Rig r;
    r.value({0x00, 0x08});
    r.ack();
    CHECK(r.servo.hold() == Result::OK);
    CHECK(r.sent() == 2 && r.tx(0) == read_cmd(0x38, 2) && r.tx(1) == write_cmd(0x2A, {0x00, 0x08}));
    CHECK(r.servo.get_hold_failed_step() == STSServo::HoldStep::NONE);
  }
  {
    Rig r;  // negative feedback is re-encoded as sign-magnitude
    r.value({0x64, 0x80});
    r.ack();
    CHECK(r.servo.hold() == Result::OK && r.tx(1) == write_cmd(0x2A, {0x64, 0x80}));
  }
  {
    Rig r;  // an alarm in the status byte does not stop the hold
    r.value({0x10, 0x00}, STS::STAT_OVERLOAD);
    r.ack();
    CHECK(r.servo.hold() == Result::OK && r.tx(1) == write_cmd(0x2A, {0x10, 0x00}));
  }
  {
    Rig r;  // read times out: nothing is commanded
    r.port.script_timeout();
    CHECK(r.servo.hold() == Result::TIMEOUT);
    CHECK(r.sent() == 1 && r.servo.get_hold_failed_step() == STSServo::HoldStep::READ_POSITION);
  }
  {
    Rig r;  // short position reply
    r.value({0x00});
    CHECK(r.servo.hold() == Result::BAD_RESPONSE);
    CHECK(r.sent() == 1 && r.servo.get_hold_failed_step() == STSServo::HoldStep::READ_POSITION);
  }
  {
    Rig r;  // write unanswered: not reported as success
    r.value({0x00, 0x08});
    r.port.script_timeout();
    CHECK(r.servo.hold() == Result::TIMEOUT);
    CHECK(r.sent() == 2 && r.servo.get_hold_failed_step() == STSServo::HoldStep::WRITE_TARGET);
  }
  {
    Rig r;  // write answered with an error status
    r.value({0x00, 0x08});
    r.value({}, STS::STAT_VOLTAGE);
    CHECK(r.servo.hold() == Result::BAD_RESPONSE);
    CHECK(r.servo.get_hold_failed_step() == STSServo::HoldStep::WRITE_TARGET);
  }
  {
    Rig r;  // magnitude beyond the target range: nothing written
    r.value({0xFF, 0x7F});
    CHECK(r.servo.hold() == Result::OUT_OF_RANGE);
    CHECK(r.sent() == 1 && r.servo.get_hold_failed_step() == STSServo::HoldStep::POSITION_OUT_OF_DOMAIN);
    r.value({0x00, 0x08});
    r.ack();
    CHECK(r.servo.hold() == Result::OK && r.servo.get_hold_failed_step() == STSServo::HoldStep::NONE);
  }
}

static void test_telemetry_fields() {
  Rig r;
  Feedback fb;
  fb.pos = 0x0800;
  fb.speed = 0x0123;
  fb.load = 0x0432;  // -50 permille
  fb.volt = 123;
  fb.temp = 45;
  fb.move = 1;
  fb.cur = 100;
  r.value(fb.bytes());
  Telemetry t;
  CHECK(r.servo.read_telemetry(&t) == Result::OK);
  CHECK(r.sent() == 1 && r.tx(0) == read_cmd(0x38, 15));
  CHECK(t.position_mdeg == 180000 && t.current_ma == 650 && t.temperature_c == 45);
  CHECK(t.alarms.none());
  CHECK(t.load_valid && t.load_permille == -50);
  CHECK(t.voltage_valid && t.voltage_mv == 12300);
  CHECK(t.moving_valid && t.moving);

  // Signed and boundary decode.
  struct Row {
    uint16_t pos, load, cur;
    int32_t mdeg, permille, ma;
  };
  const Row rows[] = {
      {0x0000, 0x0000, 0x0000, 0, 0, 0},
      {0x8001, 0x0001, 0x8001, -88, 1, -7},
      {0x0FFF, 0x03E8, 0x01F4, 359912, 1000, 3250},
      {0x7FFF, 0x07E8, 0xFFFF, 2879912, -1000, -212986},
      {0x8000, 0x0400, 0x8000, 0, 0, 0},
  };
  for (const Row& row : rows) {
    Feedback f;
    f.pos = row.pos;
    f.load = row.load;
    f.cur = row.cur;
    r.value(f.bytes());
    Telemetry s;
    CHECK(r.servo.read_telemetry(&s) == Result::OK);
    CHECK(s.position_mdeg == row.mdeg && s.load_permille == row.permille && s.current_ma == row.ma);
  }
  // Temperature and voltage extremes.
  Feedback f;
  f.temp = 255;
  f.volt = 255;
  r.value(f.bytes());
  Telemetry s;
  CHECK(r.servo.read_telemetry(&s) == Result::OK && s.temperature_c == 255 && s.voltage_mv == 25500);
  f.move = 0;
  r.value(f.bytes());
  CHECK(r.servo.read_telemetry(&s) == Result::OK && s.moving_valid && !s.moving);
}

static void test_telemetry_alarms() {
  Rig r;
  const AlarmSet volt = Alarm::OVER_VOLTAGE | Alarm::UNDER_VOLTAGE;
  const AlarmSet expect[8] = {volt,
                              AlarmSet(Alarm::ENCODER),
                              AlarmSet(Alarm::OVER_TEMPERATURE),
                              AlarmSet(Alarm::OVER_CURRENT),
                              AlarmSet(Alarm::ANGLE_LIMIT),
                              AlarmSet(Alarm::OVERLOAD),
                              AlarmSet(Alarm::OTHER),
                              AlarmSet(Alarm::OTHER)};
  for (int bit = 0; bit < 8; ++bit) {
    {  // fault in REG_STATUS
      Feedback f;
      f.status = (uint8_t)(1u << bit);
      r.value(f.bytes());
      Telemetry t;
      CHECK(r.servo.read_telemetry(&t) == Result::OK && t.alarms == expect[bit]);
      r.value({(uint8_t)(1u << bit)});
      AlarmSet a;
      CHECK(r.servo.read_alarms(&a) == Result::OK && a == expect[bit]);
    }
    if (bit < 6) {  // fault flagged only in the reply status byte: sample still delivered
      Feedback f;
      f.pos = 0x0800;
      r.value(f.bytes(), (uint8_t)(1u << bit));
      Telemetry t;
      CHECK(r.servo.read_telemetry(&t) == Result::OK && t.alarms == expect[bit] && t.position_mdeg == 180000);
      CHECK(r.servo.get_last_status() == (1u << bit));
      r.value({0}, (uint8_t)(1u << bit));
      AlarmSet a;
      CHECK(r.servo.read_alarms(&a) == Result::OK && a == expect[bit]);
    }
  }
  {  // both sources combine
    Feedback f;
    f.status = STS::STAT_TEMPERATURE;
    r.value(f.bytes(), STS::STAT_OVERLOAD);
    Telemetry t;
    CHECK(r.servo.read_telemetry(&t) == Result::OK);
    CHECK(t.alarms == (Alarm::OVER_TEMPERATURE | Alarm::OVERLOAD));
    r.value({STS::STAT_CURRENT}, STS::STAT_ANGLE);
    AlarmSet a;
    CHECK(r.servo.read_alarms(&a) == Result::OK && a == (Alarm::OVER_CURRENT | Alarm::ANGLE_LIMIT));
  }
  CHECK(r.tx(1) == read_cmd(0x41, 1));
}

static void test_telemetry_failures() {
  Rig r;
  Telemetry sentinel;
  sentinel.position_mdeg = 4242;
  sentinel.current_ma = -9;
  sentinel.temperature_c = 77;
  sentinel.load_permille = 3;
  const auto untouched = [&](const Telemetry& t) {
    return t.position_mdeg == 4242 && t.current_ma == -9 && t.temperature_c == 77 && t.load_permille == 3 &&
           !t.load_valid && !t.voltage_valid && !t.moving_valid && t.alarms.none();
  };
  Telemetry t = sentinel;
  r.port.script_timeout();
  CHECK(r.servo.read_telemetry(&t) == Result::TIMEOUT && untouched(t));

  Bytes b = Feedback{}.bytes();
  b.pop_back();  // 14 of 15 bytes
  r.value(b);
  CHECK(r.servo.read_telemetry(&t) == Result::BAD_RESPONSE && untouched(t));
  b = Feedback{}.bytes();
  b.push_back(0);  // 16 bytes
  r.value(b);
  CHECK(r.servo.read_telemetry(&t) == Result::BAD_RESPONSE && untouched(t));
  b = Feedback{}.bytes();
  b.resize(8);  // header and part of the payload only
  r.value(b, STS::STAT_OVERLOAD);
  CHECK(r.servo.read_telemetry(&t) == Result::BAD_RESPONSE && untouched(t));

  // Truncated frame on the wire.
  Bytes frame = reply(Feedback{}.bytes());
  frame.resize(frame.size() - 5);
  r.port.script_reply(frame);
  CHECK(r.servo.read_telemetry(&t) == Result::TIMEOUT && untouched(t));

  // Reply from another servo.
  Bytes other = Feedback{}.bytes();
  Bytes wrong = {0xFF, 0xFF, 0x02, 17, 0};
  wrong.insert(wrong.end(), other.begin(), other.end());
  r.port.script_reply(with_checksum(wrong));
  CHECK(r.servo.read_telemetry(&t) == Result::BAD_RESPONSE && untouched(t));

  // Bad checksum.
  Bytes bad = reply(Feedback{}.bytes());
  bad.back() ^= 0x55;
  r.port.script_reply(bad);
  CHECK(r.servo.read_telemetry(&t) == Result::BAD_RESPONSE && untouched(t));

  // Undecodable fields fail the whole sample.
  Feedback f;
  f.load = 0x03E9;  // 1001 permille
  r.value(f.bytes());
  CHECK(r.servo.read_telemetry(&t) == Result::BAD_RESPONSE && untouched(t));
  f.load = 0x0800;  // reserved bit set
  r.value(f.bytes());
  CHECK(r.servo.read_telemetry(&t) == Result::BAD_RESPONSE && untouched(t));
  f = Feedback{};
  f.move = 2;
  r.value(f.bytes());
  CHECK(r.servo.read_telemetry(&t) == Result::BAD_RESPONSE && untouched(t));

  // Transport failure.
  r.value(Feedback{}.bytes());
  r.port.fail_next_read();
  CHECK(r.servo.read_telemetry(&t) == Result::BUS && untouched(t));

  CHECK(r.servo.read_telemetry(nullptr) == Result::PARAM);

  // read_alarms failures leave the output alone.
  AlarmSet a = Alarm::OTHER;
  r.port.script_timeout();
  CHECK(r.servo.read_alarms(&a) == Result::TIMEOUT && a == AlarmSet(Alarm::OTHER));
  r.value({0, 0});
  CHECK(r.servo.read_alarms(&a) == Result::BAD_RESPONSE && a == AlarmSet(Alarm::OTHER));
  CHECK(r.servo.read_alarms(nullptr) == Result::PARAM);
}

// Replies for one call of a generic operation on the STS family.
static void script_sts(Rig& r, ConformanceOp op, int fail_at) {
  std::vector<Bytes> replies;
  switch (op) {
    case ConformanceOp::PING:
    case ConformanceOp::MOVE_TO:
    case ConformanceOp::SET_TORQUE:
    case ConformanceOp::SET_PERSIST:
    case ConformanceOp::REGISTER_WRITE: replies = {reply()}; break;
    case ConformanceOp::READ_IDENTITY: replies = {reply({3, 7, 0, 9, 3, 1})}; break;
    case ConformanceOp::PREPARE: replies = {reply(), reply()}; break;
    case ConformanceOp::SET_PROTECTION:
    case ConformanceOp::SET_PROFILE: replies = {reply(), reply(), reply(), reply()}; break;
    case ConformanceOp::HOLD: replies = {reply({0x00, 0x08}), reply()}; break;
    case ConformanceOp::READ_TELEMETRY: replies = {reply(Feedback{}.bytes())}; break;
    case ConformanceOp::READ_ALARMS: replies = {reply({0})}; break;
    case ConformanceOp::REGISTER_READ: replies = {reply({1, 2})}; break;
  }
  for (size_t i = 0; i < replies.size(); ++i) {
    if (fail_at >= 0 && (int)i == fail_at) {
      r.port.script_timeout();
      return;
    }
    r.port.script_reply(replies[i]);
  }
}

static void test_conformance() {
  {
    Rig r;
    ConformanceHooks h;
    h.reset = [&] { r.port.clear(); };
    h.script = [&](ConformanceOp op, int fail_at) { script_sts(r, op, fail_at); };
    h.frames_sent = [&] { return r.sent(); };
    CHECK(run_servo_conformance(r.servo, h) == 0);
  }
  {  // A servo that offers only move_to conforms with every other operation unsupported.
    class MoveOnly : public Servo {
     public:
      Result move_to(int32_t a) override { return (a < -1000000 || a > 1000000) ? Result::OUT_OF_RANGE : Result::OK; }
      Capabilities get_capabilities() const override {
        Capabilities c;
        c.features = Feature::ABSOLUTE_POSITION;
        c.limits.angle_min_mdeg = -1000000;
        c.limits.angle_max_mdeg = 1000000;
        c.limits.resolution_mdeg = 10;
        return c;
      }
    } minimal;
    ConformanceHooks h;
    h.reset = [] {};
    h.script = [](ConformanceOp, int) {};
    h.frames_sent = [] { return size_t{0}; };
    CHECK(run_servo_conformance(minimal, h) == 0);
  }
  {  // The suite detects a servo whose declared features do not match its operations.
    class Liar : public Servo {
     public:
      explicit Liar(bool claims_hold) : m_claims_hold(claims_hold) {}
      Result move_to(int32_t a) override { return (a < -100 || a > 100) ? Result::OUT_OF_RANGE : Result::OK; }
      Capabilities get_capabilities() const override {
        Capabilities c;
        c.features = m_claims_hold ? (Feature::ABSOLUTE_POSITION | Feature::HOLD) : FeatureSet(Feature::ABSOLUTE_POSITION);
        c.limits.angle_max_mdeg = 100;
        c.limits.angle_min_mdeg = -100;
        c.limits.resolution_mdeg = 1;
        return c;
      }
     private:
      bool m_claims_hold;
    };
    ConformanceHooks h;
    h.reset = [] {};
    h.script = [](ConformanceOp, int) {};
    h.frames_sent = [] { return size_t{0}; };
    Liar honest(false);  // same servo without the HOLD claim: conforms
    Liar liar(true);     // only the HOLD claim is false
    CHECK(run_servo_conformance(honest, h, true) == 0);
    CHECK(run_servo_conformance(liar, h, true) != 0);
    class RegisterLiar : public Liar {
     public:
      RegisterLiar() : Liar(false) {}
      Capabilities get_capabilities() const override {
        Capabilities c = Liar::get_capabilities();
        c.features.set(Feature::REGISTER_ACCESS);  // claimed, but get_registers() stays null
        return c;
      }
    } register_liar;
    CHECK(run_servo_conformance(register_liar, h, true) != 0);
  }
}

int main() {
  test_units();
  test_alarm_table();
  test_capabilities();
  test_move_to();
  test_identity();
  test_prepare();
  test_set_protection();
  test_set_profile();
  test_torque_and_persist();
  test_hold();
  test_telemetry_fields();
  test_telemetry_alarms();
  test_telemetry_failures();
  test_conformance();
  if (g_failures) {
    std::printf("%d failure(s)\n", g_failures);
    return 1;
  }
  std::printf("test_sts_generic: all passed\n");
  return 0;
}
