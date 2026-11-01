#include <algorithm>

#include "servokit/stseries/sts_servo.h"

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

Capabilities STSServo::capabilities() const {
  Capabilities c;
  for (Feature f : {Feature::ABSOLUTE_POSITION, Feature::SET_SPEED, Feature::SET_ACCEL, Feature::TORQUE_LIMIT,
                    Feature::START_TORQUE, Feature::TORQUE_ENABLE, Feature::HOLD, Feature::READ_POSITION,
                    Feature::READ_CURRENT, Feature::READ_LOAD, Feature::READ_VOLTAGE, Feature::READ_TEMPERATURE,
                    Feature::READ_MOVING, Feature::READ_ALARMS, Feature::PROTECT_CURRENT, Feature::PROTECT_TEMP,
                    Feature::PROTECT_RELEASE_ON_ALARM, Feature::PING, Feature::READ_IDENTITY, Feature::RAW_REGISTERS,
                    Feature::PERSIST_CONFIG}) {
    c.features.set(f);
  }
  Limits& l = c.limits;
  l.angle_min_mdeg = STS::units::steps_to_mdeg(-MAX_TARGET_POSITION);
  l.angle_max_mdeg = STS::units::steps_to_mdeg(MAX_TARGET_POSITION);
  l.resolution_mdeg = 88;
  l.max_speed_mdeg_s = STS::units::steps_s_to_mdeg_s(MAX_SPEED);
  l.max_accel_mdeg_s2 = STS::units::accel_units_to_mdeg_s2(MAX_ACCELERATION);
  l.max_torque_permille = MAX_TORQUE;
  l.current_min_ma = -STS::units::current_units_to_ma(500);
  l.current_max_ma = STS::units::current_units_to_ma(500);
  l.reportable_alarms = STS::units::reportable_alarms();
  return c;
}

Result STSServo::move_to(int32_t angle_mdeg) {
  const Limits l = capabilities().limits;
  if (angle_mdeg < l.angle_min_mdeg || angle_mdeg > l.angle_max_mdeg) return Result::OUT_OF_RANGE;
  return set_target_position(STS::units::mdeg_to_steps(angle_mdeg));
}

Result STSServo::read_identity(Identity* out) {
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

Result STSServo::prepare() {
  // Responses first so that the following write is acknowledged.
  const Result r = set_response_mode(STS::RSP_ALL);
  if (r != Result::OK) return r;
  return set_operating_mode(STS::OP_SERVO);
}

Result STSServo::set_protection(const Protection& p) {
  namespace u = STS::units;
  const int32_t current_units = u::ma_to_current_units(p.stall_current_ma);
  const uint32_t time_units = u::ms_to_time_units(p.stall_time_ms);
  uint8_t mask = 0;
  if (current_units > MAX_CURRENT_LIMIT || time_units > MAX_PROTECTION_TIME || p.max_temperature_c < 0 ||
      p.max_temperature_c > MAX_TEMPERATURE_LIMIT || !u::native_from_alarms(p.release_on_alarm, &mask)) {
    return Result::OUT_OF_RANGE;
  }
  Result r = set_overcurrent_threshold(static_cast<uint16_t>(current_units));
  if (r != Result::OK) return r;
  r = set_overcurrent_time(static_cast<uint8_t>(time_units));
  if (r != Result::OK) return r;
  r = set_overtemp_threshold(static_cast<uint8_t>(p.max_temperature_c));
  if (r != Result::OK) return r;
  return set_unload_alarm_mask(mask);
}

Result STSServo::set_profile(const Profile& p) {
  namespace u = STS::units;
  const uint16_t torque = std::min<uint16_t>(p.torque_permille, MAX_TORQUE);
  const uint16_t start = std::min<uint16_t>(p.start_torque_permille, MAX_TORQUE);
  const uint8_t accel =
      static_cast<uint8_t>(std::min<uint32_t>(u::mdeg_s2_to_accel_units(p.accel_mdeg_s2), MAX_ACCELERATION));
  const uint16_t speed = static_cast<uint16_t>(std::min<uint32_t>(u::mdeg_s_to_steps_s(p.speed_mdeg_s), MAX_SPEED));
  Result r = set_torque_limit(torque);
  if (r != Result::OK) return r;
  r = set_start_torque(start);
  if (r != Result::OK) return r;
  r = set_acceleration(accel);
  if (r != Result::OK) return r;
  return set_speed(speed);
}

Result STSServo::hold() {
  m_hold_step = HoldStep::NONE;
  uint8_t b[2];
  Result r = read_with_status(STS::REG_POSITION, b, sizeof(b));
  if (r != Result::OK) {
    m_hold_step = HoldStep::READ_POSITION;
    return r;
  }
  r = set_target_position(STS::units::decode_position_steps(word(b, 0)));
  if (r != Result::OK) m_hold_step = HoldStep::WRITE_TARGET;
  return r;
}

Result STSServo::set_torque(bool enable) { return set_servo_enabled(enable); }

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
