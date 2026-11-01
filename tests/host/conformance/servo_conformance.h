// Conformance suite for the generic Servo interface. Every family runs it
// through a small set of hooks that script the family's bus replies; the
// checks themselves use only the generic API and the declared capabilities.
//
// What every Servo must satisfy:
//  - an operation whose feature is absent returns UNSUPPORTED and sends nothing;
//  - an operation whose feature is present succeeds when the bus answers;
//  - move_to outside the declared domain returns OUT_OF_RANGE and sends nothing;
//  - read_telemetry, read_alarms, hold, set_protection and set_profile fail
//    when any one of their transactions fails (writes stop at the first failure), and read_telemetry leaves its output untouched;
//  - null output pointers return PARAM;
//  - set_persist agrees with Feature::PERSIST_CONFIG;
//  - set_verify_writes and verify_writes agree with Feature::VERIFY_WRITES;
//  - limits are populated for the features that use them.
#ifndef SERVO_CONFORMANCE__H
#define SERVO_CONFORMANCE__H

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>

#include "servokit/core/servo.h"

namespace servokit_test {

enum class ConformanceOp {
  PING,
  READ_IDENTITY,
  PREPARE,
  SET_PROTECTION,
  SET_PROFILE,
  MOVE_TO,
  HOLD,
  SET_TORQUE,
  SET_PERSIST,
  READ_TELEMETRY,
  READ_ALARMS,
  RAW_READ,
  RAW_WRITE
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

  const Capabilities caps = s.capabilities();
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

  const bool has_protection =
      f.has_any(Feature::PROTECT_CURRENT | Feature::PROTECT_OVERLOAD | Feature::PROTECT_TEMP |
                Feature::PROTECT_VOLTAGE | Feature::PROTECT_RELEASE_ON_ALARM);
  const bool has_profile =
      f.has_any(Feature::SET_SPEED | Feature::SET_ACCEL | Feature::TORQUE_LIMIT | Feature::START_TORQUE);
  const bool has_telemetry =
      f.has_any(Feature::READ_POSITION | Feature::READ_CURRENT | Feature::READ_TEMPERATURE | Feature::READ_ALARMS);

  Profile profile;
  profile.speed_mdeg_s = l.max_speed_mdeg_s / 2;
  profile.accel_mdeg_s2 = l.max_accel_mdeg_s2 / 2;
  profile.torque_permille = l.max_torque_permille / 2;
  profile.start_torque_permille = l.max_torque_permille / 10;
  Protection protection;
  protection.stall_current_ma = 1000;
  protection.stall_time_ms = 500;
  protection.max_temperature_c = 60;
  protection.release_on_alarm = Alarm::OVER_TEMPERATURE | Alarm::OVER_CURRENT;
  Identity identity;
  Telemetry telemetry;
  AlarmSet alarms;
  uint8_t raw[2] = {0, 0};

  // prepare() has no feature flag: a family either implements it or returns
  // UNSUPPORTED without sending anything.
  h.reset();
  h.script(ConformanceOp::PREPARE, -1);
  const Result prepared = s.prepare();
  CONF_CHECK(prepared == Result::OK || (prepared == Result::UNSUPPORTED && h.frames_sent() == 0));
  const bool has_prepare = prepared == Result::OK;

  // Each case runs one operation with its scripted bus; supported says whether
  // the capabilities promise it.
  struct Case {
    ConformanceOp op;
    bool supported;
    std::function<Result()> call;
  };
  const Case cases[] = {
      {ConformanceOp::PING, f.has(Feature::PING), [&] { return s.ping(); }},
      {ConformanceOp::READ_IDENTITY, f.has(Feature::READ_IDENTITY), [&] { return s.read_identity(&identity); }},
      {ConformanceOp::PREPARE, has_prepare, [&] { return s.prepare(); }},
      {ConformanceOp::SET_PROTECTION, has_protection, [&] { return s.set_protection(protection); }},
      {ConformanceOp::SET_PROFILE, has_profile, [&] { return s.set_profile(profile); }},
      {ConformanceOp::MOVE_TO, true, [&] { return s.move_to(0); }},
      {ConformanceOp::HOLD, f.has(Feature::HOLD), [&] { return s.hold(); }},
      {ConformanceOp::SET_TORQUE, f.has(Feature::TORQUE_ENABLE), [&] { return s.set_torque(true); }},
      {ConformanceOp::SET_PERSIST, f.has(Feature::PERSIST_CONFIG), [&] { return s.set_persist(true); }},
      {ConformanceOp::READ_TELEMETRY, has_telemetry, [&] { return s.read_telemetry(&telemetry); }},
      {ConformanceOp::READ_ALARMS, f.has(Feature::READ_ALARMS), [&] { return s.read_alarms(&alarms); }},
      {ConformanceOp::RAW_READ, f.has(Feature::RAW_REGISTERS), [&] { return s.read_raw(0, raw, 2); }},
      {ConformanceOp::RAW_WRITE, f.has(Feature::RAW_REGISTERS), [&] { return s.write_raw(0, raw, 2); }},
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

  // Write verification: off by default, and settable exactly when advertised.
  h.reset();
  CONF_CHECK(!s.verify_writes());
  const Result verify_on = s.set_verify_writes(true);
  CONF_CHECK((verify_on == Result::OK) == f.has(Feature::VERIFY_WRITES));
  CONF_CHECK(verify_on == Result::OK || verify_on == Result::UNSUPPORTED);
  CONF_CHECK(s.verify_writes() == f.has(Feature::VERIFY_WRITES));
  CONF_CHECK(s.set_verify_writes(false) == verify_on);
  CONF_CHECK(!s.verify_writes());
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
  if (f.has(Feature::HOLD)) {
    fails_at_every_transaction(ConformanceOp::HOLD, [&] { return s.hold(); }, [] {});
  }
  if (has_protection) {
    fails_at_every_transaction(ConformanceOp::SET_PROTECTION, [&] { return s.set_protection(protection); }, [] {});
  }
  if (has_profile) {
    fails_at_every_transaction(ConformanceOp::SET_PROFILE, [&] { return s.set_profile(profile); }, [] {});
  }
  if (f.has(Feature::READ_ALARMS)) {
    fails_at_every_transaction(ConformanceOp::READ_ALARMS, [&] { return s.read_alarms(&alarms); }, [] {});
    h.reset();
    CONF_CHECK(s.read_alarms(nullptr) == Result::PARAM);
  }
  if (f.has(Feature::READ_IDENTITY)) {
    h.reset();
    CONF_CHECK(s.read_identity(nullptr) == Result::PARAM);
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
