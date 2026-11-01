#include <algorithm>

#include "servokit/families/stseries/sts_servo.h"

namespace servokit {

namespace {
constexpr size_t IDENTITY_LEN = 6;    // REG_VER_MAJ (0x00) .. REG_DEVICE_ID (0x05)
constexpr size_t TELEMETRY_LEN = 15;  // REG_POSITION (0x38) .. REG_CURRENT high byte (0x46)

// Offsets inside the telemetry span.
constexpr size_t T_POS = STS::REG_POSITION - STS::REG_POSITION;
constexpr size_t T_LOAD = STS::REG_LOAD - STS::REG_POSITION;
constexpr size_t T_VOLTAGE = STS::REG_VOLTAGE - STS::REG_POSITION;
constexpr size_t T_TEMP = STS::REG_TEMPERATURE - STS::REG_POSITION;
constexpr size_t T_STATUS = STS::REG_STATUS - STS::REG_POSITION;
constexpr size_t T_MOVE = STS::REG_MOVE_FLAG - STS::REG_POSITION;
constexpr size_t T_CURRENT = STS::REG_CURRENT - STS::REG_POSITION;
static_assert(T_CURRENT + 2 == TELEMETRY_LEN, "telemetry span must end with the current register");
static_assert(STS::REG_DEVICE_ID + 1 == IDENTITY_LEN, "identity span must end with the device ID register");

uint16_t word(const uint8_t* b, size_t off) { return static_cast<uint16_t>(b[off] | (b[off + 1] << 8)); }
}  // namespace

Capabilities STSServo::get_capabilities() const {
  Capabilities c;
  for (Feature f : {Feature::ABSOLUTE_POSITION, Feature::SET_SPEED, Feature::SET_ACCEL, Feature::TORQUE_LIMIT,
                    Feature::START_TORQUE, Feature::TORQUE_ENABLE, Feature::CANCEL_MOTION, Feature::READ_POSITION,
                    Feature::READ_CURRENT, Feature::READ_LOAD, Feature::READ_VOLTAGE, Feature::READ_TEMPERATURE,
                    Feature::READ_MOVING, Feature::READ_ALARMS, Feature::PROTECT_CURRENT, Feature::PROTECT_OVERLOAD,
                    Feature::PROTECT_TEMP, Feature::PROTECT_VOLTAGE, Feature::PROTECT_RELEASE_ON_ALARM, Feature::PROBE, Feature::IDENTIFY, Feature::REGISTER_ACCESS,
                    Feature::PERSIST_CONFIG, Feature::READBACK_CHECK}) {
    c.features.set(f);
  }
  Limits& l = c.limits;
  l.angle_min_mdeg = STS::units::steps_to_mdeg(-MAX_TARGET_POSITION);
  l.angle_max_mdeg = STS::units::steps_to_mdeg(MAX_TARGET_POSITION);
  l.resolution_mdeg = 88;
  l.max_speed_mdeg_s = STS::units::steps_s_to_mdeg_s(MAX_SPEED);
  l.max_accel_mdeg_s2 = STS::units::accel_units_to_mdeg_s2(MAX_ACCELERATION);
  l.max_torque_permille = MAX_TORQUE;
  l.max_torque_mnm = STALL_TORQUE_MNM;
  // Reportable range: the largest magnitude the current feedback register can encode.
  l.current_min_ma = -STS::units::current_units_to_ma(STS::units::CURRENT_FEEDBACK_MAX_UNITS);
  l.current_max_ma = STS::units::current_units_to_ma(STS::units::CURRENT_FEEDBACK_MAX_UNITS);
  l.reportable_alarms = STS::units::reportable_alarms();
  return c;
}

Result STSServo::move_to(int32_t angle_mdeg) {
  const Limits l = get_capabilities().limits;
  if (angle_mdeg < l.angle_min_mdeg || angle_mdeg > l.angle_max_mdeg) return Result::OUT_OF_RANGE;
  return set_target_position(STS::units::mdeg_to_steps(angle_mdeg));
}

Result STSServo::identify(Identity* out) {
  if (out == nullptr) return Result::PARAM;
  uint8_t b[IDENTITY_LEN];
  const Result r = read_register(m_id, STS::REG_VER_MAJ, b, sizeof(b));
  if (r != Result::OK) return r;
  Identity id;
  const char family[] = "STS";
  std::copy(family, family + sizeof(family), id.family);
  id.firmware_major = b[STS::REG_VER_MAJ];
  id.firmware_minor = b[STS::REG_VER_SUB];
  id.model_major = b[STS::REG_MODEL_MAJ];
  id.model_minor = b[STS::REG_MODEL_SUB];
  id.device_id = b[STS::REG_DEVICE_ID];
  *out = id;
  return Result::OK;
}

Result STSServo::set_mode(MotionMode mode) {
  switch (mode) {
    case MotionMode::ABSOLUTE_POSITION: return set_operating_mode(STS::OP_SERVO);
  }
  return Result::OUT_OF_RANGE;
}

Result STSServo::set_protect_current(uint32_t ma, uint32_t time_ms) {
  namespace u = STS::units;
  if (ma > static_cast<uint32_t>(INT32_MAX)) return Result::OUT_OF_RANGE;
  const int32_t current_units = u::ma_to_current_units(static_cast<int32_t>(ma));
  const uint32_t time_units = u::ms_to_time_units(time_ms);
  if (current_units > MAX_CURRENT_LIMIT || time_units > MAX_PROTECTION_TIME) return Result::OUT_OF_RANGE;
  const Result r = set_overcurrent_threshold(static_cast<uint16_t>(current_units));
  if (r != Result::OK) return r;
  return set_overcurrent_time(static_cast<uint8_t>(time_units));
}

Result STSServo::set_protect_overload(uint16_t permille, uint32_t time_ms) {
  namespace u = STS::units;
  const uint32_t percent = u::permille_to_percent(permille);
  const uint32_t time_units = u::ms_to_time_units(time_ms);
  if (permille > MAX_TORQUE || percent > MAX_OVERLOAD_LIMIT || time_units > MAX_PROTECTION_TIME) {
    return Result::OUT_OF_RANGE;
  }
  const Result r = set_overload_threshold(static_cast<uint8_t>(percent));
  if (r != Result::OK) return r;
  return set_overload_time(static_cast<uint8_t>(time_units));
}

Result STSServo::set_protect_temperature(int16_t c) {
  if (c < 0 || c > MAX_TEMPERATURE_LIMIT) return Result::OUT_OF_RANGE;
  return set_overtemp_threshold(static_cast<uint8_t>(c));
}

Result STSServo::set_protect_voltage(uint32_t min_mv, uint32_t max_mv) {
  namespace u = STS::units;
  if (min_mv > max_mv) return Result::OUT_OF_RANGE;
  const uint32_t min_units = u::mv_to_voltage_units(min_mv);
  const uint32_t max_units = u::mv_to_voltage_units(max_mv);
  if (max_units > MAX_VOLTAGE_LIMIT) return Result::OUT_OF_RANGE;
  const Result r = set_undervoltage_threshold(static_cast<uint8_t>(min_units));
  if (r != Result::OK) return r;
  return set_overvoltage_threshold(static_cast<uint8_t>(max_units));
}

Result STSServo::set_protect_release_alarms(AlarmSet alarms) {
  uint8_t mask = 0;
  if (!STS::units::native_from_alarms(alarms, &mask)) return Result::OUT_OF_RANGE;
  return set_unload_alarm_mask(mask);
}

Result STSServo::set_speed(uint32_t mdeg_s) {
  const uint32_t steps = STS::units::mdeg_s_to_steps_s(mdeg_s);
  if (steps > MAX_SPEED) return Result::OUT_OF_RANGE;
  return set_speed_native(static_cast<uint16_t>(steps));
}

Result STSServo::set_acceleration(uint32_t mdeg_s2) {
  const uint32_t units = STS::units::mdeg_s2_to_accel_units(mdeg_s2);
  if (units > MAX_ACCELERATION) return Result::OUT_OF_RANGE;
  return set_acceleration_native(static_cast<uint8_t>(units));
}

Result STSServo::cancel_motion() {
  m_cancel_step = CancelStep::NONE;
  Result r = set_servo_enabled(false);
  if (r != Result::OK) {
    m_cancel_step = CancelStep::DISABLE_TORQUE;
    return r;
  }
  uint8_t b[2];
  r = read_with_status(STS::REG_POSITION, b, sizeof(b));
  if (r != Result::OK) {
    m_cancel_step = CancelStep::READ_POSITION;
    return r;
  }
  const int32_t steps = STS::units::decode_position_steps(word(b, 0));
  if (steps < -MAX_TARGET_POSITION || steps > MAX_TARGET_POSITION) {
    m_cancel_step = CancelStep::POSITION_OUT_OF_DOMAIN;
    return Result::OUT_OF_RANGE;
  }
  r = set_target_position(steps);
  if (r != Result::OK) m_cancel_step = CancelStep::WRITE_TARGET;
  return r;
}

Result STSServo::set_torque_enabled(bool enable) { return set_servo_enabled(enable); }

Result STSServo::read_torque_enabled(bool* out) { return get_servo_enabled(out); }

Result STSServo::read_telemetry(Telemetry* out) {
  namespace u = STS::units;
  if (out == nullptr) return Result::PARAM;
  uint8_t b[TELEMETRY_LEN];
  const Result r = read_with_status(STS::REG_POSITION, b, sizeof(b));
  if (r != Result::OK) return r;

  Telemetry t;
  int16_t load = 0;
  if (!u::decode_load_permille(word(b, T_LOAD), &load) || b[T_MOVE] > 1) return Result::BAD_RESPONSE;
  t.position_mdeg = u::steps_to_mdeg(u::decode_position_steps(word(b, T_POS)));
  t.current_ma = u::current_units_to_ma(u::decode_current_units(word(b, T_CURRENT)));
  t.temperature_c = b[T_TEMP];
  t.alarms = u::alarms_from_native(static_cast<uint8_t>(b[T_STATUS] | m_last_status));
  t.load_valid = true;
  t.load_permille = load;
  t.voltage_valid = true;
  t.voltage_mv = static_cast<uint16_t>(u::voltage_units_to_mv(b[T_VOLTAGE]));
  t.moving_valid = true;
  t.moving = b[T_MOVE] == STS::SERVO_MOVING;
  *out = t;
  return Result::OK;
}

Result STSServo::read_alarms(AlarmSet* out) {
  if (out == nullptr) return Result::PARAM;
  uint8_t status = 0;
  const Result r = read_with_status(STS::REG_STATUS, &status, 1);
  if (r != Result::OK) return r;
  *out = STS::units::alarms_from_native(static_cast<uint8_t>(status | m_last_status));
  return Result::OK;
}

}  // namespace servokit
