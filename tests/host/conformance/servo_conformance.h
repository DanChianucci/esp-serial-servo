// Conformance suite for the generic Servo interface. Every family runs it
// through a small set of hooks that script the family's bus replies; the
// checks themselves use only the generic API and the declared capabilities.
//
// What every Servo must satisfy:
//  - an operation whose feature is absent returns UNSUPPORTED and sends nothing;
//  - an operation whose feature is present succeeds when the bus answers;
//  - move_to outside the declared domain returns OUT_OF_RANGE and sends nothing;
//  - each protection and motion-parameter setter (set_protect_*, set_speed,
//    set_acceleration, set_torque_limit, set_start_torque) is UNSUPPORTED exactly
//    when its own feature is absent, independent of the others;
//  - cancel_motion and the torque-enable calls agree with their features;
//    set_mode is either implemented or UNSUPPORTED with nothing sent;
//  - read_telemetry, read_alarms, cancel_motion and the multi-write setters fail
//    when any one of their transactions fails (writes stop at the first failure),
//    and read_telemetry leaves its output untouched;
//  - null output pointers return PARAM;
//  - set_persist agrees with Feature::PERSIST_CONFIG;
//  - get_registers() is non-null exactly when Feature::REGISTER_ACCESS is set,
//    and its set_readback_check/get_readback_check agree with
//    Feature::READBACK_CHECK;
//  - get_serial_link() is stable across calls and, when non-null, reports
//    checksum verification on by default;
//  - limits are populated for the features that use them.
#ifndef SERVO_CONFORMANCE__H
#define SERVO_CONFORMANCE__H

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>

#include "servokit/core/register_access.h"
#include "servokit/core/servo.h"
#include "servokit/serial/serial_link.h"

namespace servokit_test {

enum class ConformanceOp {
  PROBE,
  IDENTIFY,
  SET_MODE,
  SET_PROTECT_CURRENT,
  SET_PROTECT_OVERLOAD,
  SET_PROTECT_TEMPERATURE,
  SET_PROTECT_VOLTAGE,
  SET_PROTECT_RELEASE_ALARMS,
  SET_SPEED,
  SET_ACCELERATION,
  SET_TORQUE_LIMIT,
  SET_START_TORQUE,
  MOVE_TO,
  CANCEL_MOTION,
  SET_TORQUE_ENABLED,
  READ_TORQUE_ENABLED,
  SET_PERSIST,
  READ_TELEMETRY,
  READ_ALARMS,
  REGISTER_READ,
  REGISTER_WRITE
};

struct ConformanceHooks {
  // Clears the sent-frame log and any scripted replies.
  std::function<void()> reset;
  // Scripts the bus replies for one call of op. fail_at < 0: every
  // transaction of the call is answered correctly. fail_at >= 0: the
  // transaction with that zero-based index gets no answer and none follow.
  std::function<void(ConformanceOp op, int fail_at)> script;
  // Number of frames sent since reset().
  std::function<size_t()> frames_sent;
};

// Returns the number of failed checks (0 = conforming).
// quiet suppresses failure messages (for tests of the suite itself).
inline int run_servo_conformance(servokit::Servo& s, const ConformanceHooks& h, bool quiet = false) {
  using namespace servokit;
  int failures = 0;
#define CONF_CHECK(cond)                                                      \
  do {                                                                        \
    if (!(cond)) {                                                            \
      if (!quiet) std::printf("CONFORMANCE FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++failures;                                                             \
    }                                                                         \
  } while (0)

  const Capabilities caps = s.get_capabilities();
  const FeatureSet f = caps.features;
  const Limits& l = caps.limits;

  // Limits are populated for the features that use them.
  CONF_CHECK(f.has(Feature::ABSOLUTE_POSITION));  // move_to is mandatory
  CONF_CHECK(l.angle_max_mdeg > l.angle_min_mdeg);
  CONF_CHECK(l.resolution_mdeg > 0);
  CONF_CHECK(!f.has(Feature::SET_SPEED) || l.max_speed_mdeg_s > 0);
  CONF_CHECK(!f.has(Feature::SET_ACCEL) || l.max_accel_mdeg_s2 > 0);
  CONF_CHECK(!f.has(Feature::TORQUE_LIMIT) || l.max_torque_permille > 0);
  CONF_CHECK(!f.has(Feature::READ_CURRENT) || l.current_max_ma > l.current_min_ma);
  CONF_CHECK(f.has(Feature::READ_ALARMS) == l.reportable_alarms.any());

  const bool has_telemetry =
      f.has_any(Feature::READ_POSITION | Feature::READ_CURRENT | Feature::READ_TEMPERATURE | Feature::READ_ALARMS);

  // Mid-range arguments every family can hold.
  const uint32_t speed = l.max_speed_mdeg_s / 2;
  const uint32_t accel = l.max_accel_mdeg_s2 / 2;
  const uint16_t torque = static_cast<uint16_t>(l.max_torque_permille / 2);
  const uint16_t start_torque = static_cast<uint16_t>(l.max_torque_permille / 10);
  const AlarmSet release = Alarm::OVER_TEMPERATURE | Alarm::OVER_CURRENT;
  bool torque_enabled = false;
  Identity identity;
  Telemetry telemetry;
  AlarmSet alarms;
  uint8_t raw[2] = {0, 0};

  // The register interface exists exactly when the feature is advertised; the
  // pointer is stable. A servo without it gets a stand-in whose calls report
  // UNSUPPORTED, so the table below can stay uniform.
  struct NoRegisters : RegisterAccess {
    Result read_register(uint16_t, uint8_t*, size_t) override { return Result::UNSUPPORTED; }
    Result write_register(uint16_t, const uint8_t*, size_t) override { return Result::UNSUPPORTED; }
    Result read_u8(uint16_t, uint8_t*) override { return Result::UNSUPPORTED; }
    Result read_u16(uint16_t, uint16_t*) override { return Result::UNSUPPORTED; }
    Result write_u8(uint16_t, uint8_t) override { return Result::UNSUPPORTED; }
    Result write_u16(uint16_t, uint16_t) override { return Result::UNSUPPORTED; }
    Result set_readback_check(bool) override { return Result::UNSUPPORTED; }
    bool get_readback_check() const override { return false; }
  } no_registers;
  RegisterAccess* const reg_ptr = s.get_registers();
  CONF_CHECK((reg_ptr != nullptr) == f.has(Feature::REGISTER_ACCESS));
  CONF_CHECK(s.get_registers() == reg_ptr);
  SerialLink* const link = s.get_serial_link();
  CONF_CHECK(s.get_serial_link() == link);
  CONF_CHECK(link == nullptr || link->get_verify_checksum());
  RegisterAccess* const regs = reg_ptr != nullptr ? reg_ptr : &no_registers;

  // set_mode() has no feature flag: a family either implements it or returns
  // UNSUPPORTED without sending anything.
  h.reset();
  h.script(ConformanceOp::SET_MODE, -1);
  const Result moded = s.set_mode(MotionMode::ABSOLUTE_POSITION);
  CONF_CHECK(moded == Result::OK || (moded == Result::UNSUPPORTED && h.frames_sent() == 0));
  const bool has_mode = moded == Result::OK;

  // Each case runs one operation with its scripted bus; supported says whether
  // the capabilities promise it.
  struct Case {
    ConformanceOp op;
    bool supported;
    std::function<Result()> call;
  };
  const Case cases[] = {
      {ConformanceOp::PROBE, f.has(Feature::PROBE), [&] { return s.probe(); }},
      {ConformanceOp::IDENTIFY, f.has(Feature::IDENTIFY), [&] { return s.identify(&identity); }},
      {ConformanceOp::SET_MODE, has_mode, [&] { return s.set_mode(MotionMode::ABSOLUTE_POSITION); }},
      {ConformanceOp::SET_PROTECT_CURRENT, f.has(Feature::PROTECT_CURRENT),
       [&] { return s.set_protect_current(1000, 500); }},
      {ConformanceOp::SET_PROTECT_OVERLOAD, f.has(Feature::PROTECT_OVERLOAD),
       [&] { return s.set_protect_overload(500, 500); }},
      {ConformanceOp::SET_PROTECT_TEMPERATURE, f.has(Feature::PROTECT_TEMP),
       [&] { return s.set_protect_temperature(60); }},
      {ConformanceOp::SET_PROTECT_VOLTAGE, f.has(Feature::PROTECT_VOLTAGE),
       [&] { return s.set_protect_voltage(6000, 8400); }},
      {ConformanceOp::SET_PROTECT_RELEASE_ALARMS, f.has(Feature::PROTECT_RELEASE_ON_ALARM),
       [&] { return s.set_protect_release_alarms(release); }},
      {ConformanceOp::SET_SPEED, f.has(Feature::SET_SPEED), [&] { return s.set_speed(speed); }},
      {ConformanceOp::SET_ACCELERATION, f.has(Feature::SET_ACCEL), [&] { return s.set_acceleration(accel); }},
      {ConformanceOp::SET_TORQUE_LIMIT, f.has(Feature::TORQUE_LIMIT), [&] { return s.set_torque_limit(torque); }},
      {ConformanceOp::SET_START_TORQUE, f.has(Feature::START_TORQUE),
       [&] { return s.set_start_torque(start_torque); }},
      {ConformanceOp::MOVE_TO, true, [&] { return s.move_to(0); }},
      {ConformanceOp::CANCEL_MOTION, f.has(Feature::CANCEL_MOTION), [&] { return s.cancel_motion(); }},
      {ConformanceOp::SET_TORQUE_ENABLED, f.has(Feature::TORQUE_ENABLE), [&] { return s.set_torque_enabled(true); }},
      {ConformanceOp::READ_TORQUE_ENABLED, f.has(Feature::TORQUE_ENABLE),
       [&] { return s.read_torque_enabled(&torque_enabled); }},
      {ConformanceOp::SET_PERSIST, f.has(Feature::PERSIST_CONFIG), [&] { return s.set_persist(true); }},
      {ConformanceOp::READ_TELEMETRY, has_telemetry, [&] { return s.read_telemetry(&telemetry); }},
      {ConformanceOp::READ_ALARMS, f.has(Feature::READ_ALARMS), [&] { return s.read_alarms(&alarms); }},
      {ConformanceOp::REGISTER_READ, f.has(Feature::REGISTER_ACCESS),
       [&] { return regs->read_register(0, raw, 2); }},
      {ConformanceOp::REGISTER_WRITE, f.has(Feature::REGISTER_ACCESS),
       [&] { return regs->write_register(0, raw, 2); }},
  };

  for (const Case& c : cases) {
    h.reset();
    h.script(c.op, -1);
    const Result r = c.call();
    if (c.supported) {
      CONF_CHECK(r == Result::OK);
    } else {
      CONF_CHECK(r == Result::UNSUPPORTED);
      CONF_CHECK(h.frames_sent() == 0);
    }
  }

  // Readback check: off by default, and settable exactly when advertised.
  h.reset();
  CONF_CHECK(!regs->get_readback_check());
  const Result verify_on = regs->set_readback_check(true);
  CONF_CHECK((verify_on == Result::OK) == f.has(Feature::READBACK_CHECK));
  CONF_CHECK(verify_on == Result::OK || verify_on == Result::UNSUPPORTED);
  CONF_CHECK(regs->get_readback_check() == f.has(Feature::READBACK_CHECK));
  CONF_CHECK(regs->set_readback_check(false) == verify_on);
  CONF_CHECK(!regs->get_readback_check());
  CONF_CHECK(h.frames_sent() == 0);

  // set_persist(false) agrees with set_persist(true).
  h.reset();
  h.script(ConformanceOp::SET_PERSIST, -1);
  CONF_CHECK((s.set_persist(false) == Result::OK) == f.has(Feature::PERSIST_CONFIG));

  // move_to bounds: the domain ends are accepted, a full step beyond is
  // rejected without any bus traffic.
  for (int32_t angle : {l.angle_min_mdeg, l.angle_max_mdeg}) {
    h.reset();
    h.script(ConformanceOp::MOVE_TO, -1);
    CONF_CHECK(s.move_to(angle) == Result::OK);
  }
  for (int32_t angle : {l.angle_min_mdeg - static_cast<int32_t>(l.resolution_mdeg),
                        l.angle_max_mdeg + static_cast<int32_t>(l.resolution_mdeg), INT32_MIN, INT32_MAX}) {
    h.reset();
    CONF_CHECK(s.move_to(angle) == Result::OUT_OF_RANGE);
    CONF_CHECK(h.frames_sent() == 0);
  }

  // Multi-transaction reads fail as a whole when any transaction fails.
  const auto fails_at_every_transaction = [&](ConformanceOp op, const std::function<Result()>& call,
                                              const std::function<void()>& check_untouched) {
    h.reset();
    h.script(op, -1);
    CONF_CHECK(call() == Result::OK);
    const int transactions = static_cast<int>(h.frames_sent());
    CONF_CHECK(transactions >= 1);
    for (int k = 0; k < transactions; ++k) {
      h.reset();
      h.script(op, k);
      CONF_CHECK(call() != Result::OK);
      CONF_CHECK(h.frames_sent() == static_cast<size_t>(k + 1));
      check_untouched();
    }
  };

  if (has_telemetry) {
    Telemetry sentinel;
    sentinel.position_mdeg = 123456;
    sentinel.current_ma = -77;
    sentinel.temperature_c = 99;
    sentinel.alarms = Alarm::OTHER;
    sentinel.load_permille = 5;
    telemetry = sentinel;
    fails_at_every_transaction(
        ConformanceOp::READ_TELEMETRY,
        [&] {
          telemetry = sentinel;
          return s.read_telemetry(&telemetry);
        },
        [&] {
          CONF_CHECK(telemetry.position_mdeg == 123456 && telemetry.current_ma == -77);
          CONF_CHECK(telemetry.temperature_c == 99 && telemetry.alarms == AlarmSet(Alarm::OTHER));
          CONF_CHECK(telemetry.load_permille == 5 && !telemetry.load_valid);
        });
    // A successful sample marks exactly the optional fields the features promise.
    h.reset();
    h.script(ConformanceOp::READ_TELEMETRY, -1);
    Telemetry ok;
    CONF_CHECK(s.read_telemetry(&ok) == Result::OK);
    CONF_CHECK(ok.load_valid == f.has(Feature::READ_LOAD));
    CONF_CHECK(ok.voltage_valid == f.has(Feature::READ_VOLTAGE));
    CONF_CHECK(ok.moving_valid == f.has(Feature::READ_MOVING));
    h.reset();
    CONF_CHECK(s.read_telemetry(nullptr) == Result::PARAM);
    CONF_CHECK(h.frames_sent() == 0);
  }
  if (f.has(Feature::CANCEL_MOTION)) {
    fails_at_every_transaction(ConformanceOp::CANCEL_MOTION, [&] { return s.cancel_motion(); }, [] {});
  }
  // Setters made of several writes stop at the first failure.
  for (const Case& c : cases) {
    const bool setter = c.op >= ConformanceOp::SET_PROTECT_CURRENT && c.op <= ConformanceOp::SET_START_TORQUE;
    if (setter && c.supported) fails_at_every_transaction(c.op, c.call, [] {});
  }
  if (f.has(Feature::TORQUE_ENABLE)) {
    h.reset();
    CONF_CHECK(s.read_torque_enabled(nullptr) == Result::PARAM);
    CONF_CHECK(h.frames_sent() == 0);
  }
  if (f.has(Feature::READ_ALARMS)) {
    fails_at_every_transaction(ConformanceOp::READ_ALARMS, [&] { return s.read_alarms(&alarms); }, [] {});
    h.reset();
    CONF_CHECK(s.read_alarms(nullptr) == Result::PARAM);
  }
  if (f.has(Feature::IDENTIFY)) {
    h.reset();
    CONF_CHECK(s.identify(nullptr) == Result::PARAM);
  }

  // A missing bus reply is reported, never turned into OK.
  for (const Case& c : cases) {
    if (!c.supported) continue;
    h.reset();
    h.script(c.op, 0);
    const Result r = c.call();
    CONF_CHECK(r != Result::OK || h.frames_sent() == 0);  // a servo with no bus traffic cannot fail
  }
#undef CONF_CHECK
  return failures;
}

}  // namespace servokit_test
#endif  // SERVO_CONFORMANCE__H
