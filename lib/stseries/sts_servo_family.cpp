#include <cstring>

#include "servokit/stseries/sts_servo.h"

namespace servokit {

namespace {
constexpr uint16_t SIGN_BIT = 1u << 15;

// Sign-magnitude as used by REG_CTRL_POS and REG_CTRL_SPEED: bit 15 is the
// sign, the low 15 bits the magnitude. Zero is always encoded as 0.
uint16_t sign_magnitude(int32_t v) {
  return v < 0 ? static_cast<uint16_t>(SIGN_BIT | static_cast<uint16_t>(-v)) : static_cast<uint16_t>(v);
}

EepromResult rejected(Result r) {
  EepromResult e;
  e.result = r;
  return e;
}
}  // namespace

// --- EEPROM-region update ---

EepromResult STSServo::eeprom_update(uint16_t addr, const uint8_t* value, const uint8_t* mask, size_t len,
                                     bool persist) {
  uint8_t current[2];
  uint8_t desired[2];
  if (len == 0 || len > sizeof(current) || (mask != nullptr && len != 1)) return rejected(Result::PARAM);

  EepromResult out;
  Result r = read_register(m_id, addr, current, len);
  if (r != Result::OK) {
    out.result = r;
    out.failed_step = EepromStep::READ;
    return out;
  }
  std::memcpy(desired, value, len);
  if (mask != nullptr) desired[0] = static_cast<uint8_t>((current[0] & ~*mask) | (value[0] & *mask));
  if (std::memcmp(current, desired, len) == 0) return out;  // already holds the value: send nothing

  if (!persist) {
    r = write_register(m_id, addr, desired, len);
    if (r == Result::OK) {
      out.wrote = true;
    } else {
      out.result = r;
      out.failed_step = EepromStep::WRITE;
    }
    return out;
  }

  r = unlock_eeprom();
  if (r != Result::OK) {
    out.result = r;
    out.failed_step = EepromStep::UNLOCK;
  } else {
    r = write_register(m_id, addr, desired, len);
    if (r == Result::OK) {
      out.wrote = true;
    } else {
      out.result = r;
      out.failed_step = EepromStep::WRITE;
    }
  }

  // Lock again on every path once an unlock was attempted: an unacknowledged
  // unlock may still have taken effect.
  r = lock_eeprom();
  if (r != Result::OK) {
    if (out.result == Result::OK) {
      out.result = r;
      out.failed_step = EepromStep::RELOCK;
    } else {
      out.relock_failed = true;
    }
  }
  return out;
}

EepromResult STSServo::eeprom_update_u8(uint16_t addr, uint8_t value, bool persist) {
  return eeprom_update(addr, &value, nullptr, 1, persist);
}

EepromResult STSServo::eeprom_update_u16(uint16_t addr, uint16_t value, bool persist) {
  const uint8_t b[2] = {static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>(value >> 8)};
  return eeprom_update(addr, b, nullptr, 2, persist);
}

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

EepromResult STSServo::set_operating_mode(STS::op_mode_t mode, bool persist) {
  if (static_cast<unsigned>(mode) > STS::OP_STEP) return rejected(Result::OUT_OF_RANGE);
  return eeprom_update_u8(STS::REG_OPERATION_MODE, static_cast<uint8_t>(mode), persist);
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

EepromResult STSServo::set_response_mode(STS::rsp_mode_t mode, bool persist) {
  if (static_cast<unsigned>(mode) > STS::RSP_ALL) return rejected(Result::OUT_OF_RANGE);
  return eeprom_update_u8(STS::REG_RESPONSE_MODE, static_cast<uint8_t>(mode), persist);
}

Result STSServo::get_response_delay(uint8_t* delay_2us) {
  if (delay_2us == nullptr) return Result::PARAM;
  return read_u8(m_id, STS::REG_RESPONSE_DELAY, delay_2us);
}

EepromResult STSServo::set_response_delay(uint8_t delay_2us, bool persist) {
  if (delay_2us > MAX_RESPONSE_DELAY) return rejected(Result::OUT_OF_RANGE);
  return eeprom_update_u8(STS::REG_RESPONSE_DELAY, delay_2us, persist);
}

// --- Motion (SRAM) ---

Result STSServo::get_servo_enabled(bool* enabled) { return read_bool(STS::REG_SERVO_EN, enabled); }

Result STSServo::set_servo_enabled(bool enabled) {
  return write_u8(m_id, STS::REG_SERVO_EN, enabled ? STS::SERVO_ENABLE : STS::SERVO_DISABLE);
}

Result STSServo::set_acceleration(uint8_t accel) {
  if (accel > MAX_ACCELERATION) return Result::OUT_OF_RANGE;
  return write_u8(m_id, STS::REG_CTRL_ACCEL, accel);
}

Result STSServo::set_speed(uint16_t steps_per_s) {
  if (steps_per_s > MAX_SPEED) return Result::OUT_OF_RANGE;
  return write_u16(m_id, STS::REG_CTRL_SPEED, steps_per_s);
}

Result STSServo::set_signed_speed(int32_t steps_per_s) {
  if (steps_per_s > MAX_SPEED || steps_per_s < -MAX_SPEED) return Result::OUT_OF_RANGE;
  return write_u16(m_id, STS::REG_CTRL_SPEED, sign_magnitude(steps_per_s));
}

Result STSServo::set_torque_limit(uint16_t torque) {
  if (torque > MAX_TORQUE) return Result::OUT_OF_RANGE;
  return write_u16(m_id, STS::REG_CTRL_TORQUE, torque);
}

EepromResult STSServo::set_start_torque(uint16_t torque, bool persist) {
  if (torque > MAX_TORQUE) return rejected(Result::OUT_OF_RANGE);
  return eeprom_update_u16(STS::REG_START_TORQUE, torque, persist);
}

Result STSServo::set_target_position(int32_t steps) {
  if (steps > MAX_TARGET_POSITION || steps < -MAX_TARGET_POSITION) return Result::OUT_OF_RANGE;
  return write_u16(m_id, STS::REG_CTRL_POS, sign_magnitude(steps));
}

// --- EEPROM lock and phase ---

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

EepromResult STSServo::set_multiturn_feedback(bool enabled, bool persist) {
  const uint8_t bits = enabled ? PHASE_MULTITURN_BIT : 0;
  const uint8_t mask = PHASE_MULTITURN_BIT;
  return eeprom_update(STS::REG_PHASE, &bits, &mask, 1, persist);
}

// --- Protection ---

EepromResult STSServo::set_overtemp_threshold(uint8_t celsius, bool persist) {
  if (celsius > MAX_TEMPERATURE_LIMIT) return rejected(Result::OUT_OF_RANGE);
  return eeprom_update_u8(STS::REG_OVERTEMP_THRESH, celsius, persist);
}

EepromResult STSServo::set_overvoltage_threshold(uint8_t decivolts, bool persist) {
  if (decivolts > MAX_VOLTAGE_LIMIT) return rejected(Result::OUT_OF_RANGE);
  return eeprom_update_u8(STS::REG_OVERVOLT_THRESH, decivolts, persist);
}

EepromResult STSServo::set_undervoltage_threshold(uint8_t decivolts, bool persist) {
  if (decivolts > MAX_VOLTAGE_LIMIT) return rejected(Result::OUT_OF_RANGE);
  return eeprom_update_u8(STS::REG_UNDERVOLT_THRESH, decivolts, persist);
}

EepromResult STSServo::set_overcurrent_threshold(uint16_t units, bool persist) {
  if (units > MAX_CURRENT_LIMIT) return rejected(Result::OUT_OF_RANGE);
  return eeprom_update_u16(STS::REG_OVERCURR_THRESH, units, persist);
}

EepromResult STSServo::set_overcurrent_time(uint8_t units_10ms, bool persist) {
  if (units_10ms > MAX_PROTECTION_TIME) return rejected(Result::OUT_OF_RANGE);
  return eeprom_update_u8(STS::REG_OVERCURR_TIME, units_10ms, persist);
}

EepromResult STSServo::set_overload_threshold(uint8_t percent, bool persist) {
  if (percent > MAX_OVERLOAD_LIMIT) return rejected(Result::OUT_OF_RANGE);
  return eeprom_update_u8(STS::REG_OVERLOAD_THRESH, percent, persist);
}

EepromResult STSServo::set_overload_time(uint8_t units_10ms, bool persist) {
  if (units_10ms > MAX_PROTECTION_TIME) return rejected(Result::OUT_OF_RANGE);
  return eeprom_update_u8(STS::REG_OVERLOAD_TIME, units_10ms, persist);
}

EepromResult STSServo::set_led_alarm_mask(uint8_t mask, bool persist) {
  if ((mask & ~ALARM_MASK_BITS) != 0) return rejected(Result::OUT_OF_RANGE);
  return eeprom_update_u8(STS::REG_LED_ALARM_MASK, mask, persist);
}

EepromResult STSServo::set_unload_alarm_mask(uint8_t mask, bool persist) {
  if ((mask & ~ALARM_MASK_BITS) != 0) return rejected(Result::OUT_OF_RANGE);
  return eeprom_update_u8(STS::REG_UNLOAD_ALARM_MASK, mask, persist);
}

}  // namespace servokit
