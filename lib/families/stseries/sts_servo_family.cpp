
#include "servokit/families/stseries/sts_servo.h"

namespace servokit {

namespace {
constexpr uint16_t SIGN_BIT = 1u << 15;

// Sign-magnitude as used by REG_CTRL_POS and REG_CTRL_SPEED: bit 15 is the
// sign, the low 15 bits the magnitude. Zero is always encoded as 0.
uint16_t sign_magnitude(int32_t v) {
  return v < 0 ? static_cast<uint16_t>(SIGN_BIT | static_cast<uint16_t>(-v)) : static_cast<uint16_t>(v);
}
}  // namespace

Result STSServo::read_bool(uint16_t addr, bool* out) {
  if (out == nullptr) return Result::PARAM;
  uint8_t v = 0;
  const Result r = read_u8(m_id, addr, &v);
  if (r != Result::OK) return r;
  if (v > 1) return Result::BAD_RESPONSE;
  *out = v == 1;
  return Result::OK;
}

// --- Modes ---

Result STSServo::get_operating_mode(STS::op_mode_t* mode) {
  if (mode == nullptr) return Result::PARAM;
  uint8_t v = 0;
  const Result r = read_u8(m_id, STS::REG_OPERATION_MODE, &v);
  if (r != Result::OK) return r;
  if (v > STS::OP_STEP) return Result::BAD_RESPONSE;
  *mode = static_cast<STS::op_mode_t>(v);
  return Result::OK;
}

Result STSServo::set_operating_mode(STS::op_mode_t mode) {
  if (static_cast<unsigned>(mode) > STS::OP_STEP) return Result::OUT_OF_RANGE;
  return write_u8(m_id, STS::REG_OPERATION_MODE, static_cast<uint8_t>(mode));
}

Result STSServo::get_response_mode(STS::rsp_mode_t* mode) {
  if (mode == nullptr) return Result::PARAM;
  uint8_t v = 0;
  const Result r = read_u8(m_id, STS::REG_RESPONSE_MODE, &v);
  if (r != Result::OK) return r;
  if (v > STS::RSP_ALL) return Result::BAD_RESPONSE;
  *mode = static_cast<STS::rsp_mode_t>(v);
  return Result::OK;
}

Result STSServo::set_response_mode(STS::rsp_mode_t mode) {
  if (static_cast<unsigned>(mode) > STS::RSP_ALL) return Result::OUT_OF_RANGE;
  return write_u8(m_id, STS::REG_RESPONSE_MODE, static_cast<uint8_t>(mode));
}

Result STSServo::get_response_delay(uint8_t* delay_2us) {
  if (delay_2us == nullptr) return Result::PARAM;
  return read_u8(m_id, STS::REG_RESPONSE_DELAY, delay_2us);
}

Result STSServo::set_response_delay(uint8_t delay_2us) {
  if (delay_2us > MAX_RESPONSE_DELAY) return Result::OUT_OF_RANGE;
  return write_u8(m_id, STS::REG_RESPONSE_DELAY, delay_2us);
}

// --- Motion (SRAM) ---

Result STSServo::get_servo_enabled(bool* enabled) { return read_bool(STS::REG_SERVO_EN, enabled); }

Result STSServo::set_servo_enabled(bool enabled) {
  return write_u8(m_id, STS::REG_SERVO_EN, enabled ? STS::SERVO_ENABLE : STS::SERVO_DISABLE);
}

Result STSServo::set_acceleration_native(uint8_t accel) {
  if (accel > MAX_ACCELERATION) return Result::OUT_OF_RANGE;
  return write_u8(m_id, STS::REG_CTRL_ACCEL, accel);
}

Result STSServo::set_speed_native(uint16_t steps_per_s) {
  if (steps_per_s > MAX_SPEED) return Result::OUT_OF_RANGE;
  return write_u16(m_id, STS::REG_CTRL_SPEED, steps_per_s);
}

Result STSServo::set_signed_speed(int32_t steps_per_s) {
  if (steps_per_s > MAX_SPEED || steps_per_s < -MAX_SPEED) return Result::OUT_OF_RANGE;
  return write_u16(m_id, STS::REG_CTRL_SPEED, sign_magnitude(steps_per_s));
}

Result STSServo::set_torque_limit(uint16_t permille) {
  if (permille > MAX_TORQUE) return Result::OUT_OF_RANGE;
  return write_u16(m_id, STS::REG_CTRL_TORQUE, permille);
}

Result STSServo::set_start_torque(uint16_t permille) {
  if (permille > MAX_TORQUE) return Result::OUT_OF_RANGE;
  return write_u16(m_id, STS::REG_START_TORQUE, permille);
}

Result STSServo::set_target_position(int32_t steps) {
  if (steps > MAX_TARGET_POSITION || steps < -MAX_TARGET_POSITION) return Result::OUT_OF_RANGE;
  return write_u16(m_id, STS::REG_CTRL_POS, sign_magnitude(steps));
}

// --- EEPROM lock and phase ---
// The lock is changed only by these calls; no setter touches it.

Result STSServo::get_eeprom_locked(bool* locked) { return read_bool(STS::REG_EEPROM_LOCK, locked); }
Result STSServo::unlock_eeprom() { return write_u8(m_id, STS::REG_EEPROM_LOCK, STS::EEPROM_UNLOCKED); }
Result STSServo::lock_eeprom() { return write_u8(m_id, STS::REG_EEPROM_LOCK, STS::EEPROM_LOCKED); }

Result STSServo::get_multiturn_feedback(bool* enabled) {
  if (enabled == nullptr) return Result::PARAM;
  uint8_t v = 0;
  const Result r = read_u8(m_id, STS::REG_PHASE, &v);
  if (r == Result::OK) *enabled = (v & PHASE_MULTITURN_BIT) != 0;
  return r;
}

Result STSServo::set_multiturn_feedback(bool enabled) {
  // The other REG_PHASE bits must keep their values, so the current byte is
  // read and only the multi-turn bit is changed before the single write.
  uint8_t v = 0;
  const Result r = read_u8(m_id, STS::REG_PHASE, &v);
  if (r != Result::OK) return r;
  v = enabled ? static_cast<uint8_t>(v | PHASE_MULTITURN_BIT) : static_cast<uint8_t>(v & ~PHASE_MULTITURN_BIT);
  return write_u8(m_id, STS::REG_PHASE, v);
}

// --- Protection ---

Result STSServo::set_overtemp_threshold(uint8_t celsius) {
  if (celsius > MAX_TEMPERATURE_LIMIT) return Result::OUT_OF_RANGE;
  return write_u8(m_id, STS::REG_OVERTEMP_THRESH, celsius);
}

Result STSServo::set_overvoltage_threshold(uint8_t decivolts) {
  if (decivolts > MAX_VOLTAGE_LIMIT) return Result::OUT_OF_RANGE;
  return write_u8(m_id, STS::REG_OVERVOLT_THRESH, decivolts);
}

Result STSServo::set_undervoltage_threshold(uint8_t decivolts) {
  if (decivolts > MAX_VOLTAGE_LIMIT) return Result::OUT_OF_RANGE;
  return write_u8(m_id, STS::REG_UNDERVOLT_THRESH, decivolts);
}

Result STSServo::set_overcurrent_threshold(uint16_t units) {
  if (units > MAX_CURRENT_LIMIT) return Result::OUT_OF_RANGE;
  return write_u16(m_id, STS::REG_OVERCURR_THRESH, units);
}

Result STSServo::set_overcurrent_time(uint8_t units_10ms) {
  if (units_10ms > MAX_PROTECTION_TIME) return Result::OUT_OF_RANGE;
  return write_u8(m_id, STS::REG_OVERCURR_TIME, units_10ms);
}

Result STSServo::set_overload_threshold(uint8_t percent) {
  if (percent > MAX_OVERLOAD_LIMIT) return Result::OUT_OF_RANGE;
  return write_u8(m_id, STS::REG_OVERLOAD_THRESH, percent);
}

Result STSServo::set_overload_time(uint8_t units_10ms) {
  if (units_10ms > MAX_PROTECTION_TIME) return Result::OUT_OF_RANGE;
  return write_u8(m_id, STS::REG_OVERLOAD_TIME, units_10ms);
}

Result STSServo::set_led_alarm_mask(uint8_t mask) {
  if ((mask & ~ALARM_MASK_BITS) != 0) return Result::OUT_OF_RANGE;
  return write_u8(m_id, STS::REG_LED_ALARM_MASK, mask);
}

Result STSServo::set_unload_alarm_mask(uint8_t mask) {
  if ((mask & ~ALARM_MASK_BITS) != 0) return Result::OUT_OF_RANGE;
  return write_u8(m_id, STS::REG_UNLOAD_ALARM_MASK, mask);
}

}  // namespace servokit
