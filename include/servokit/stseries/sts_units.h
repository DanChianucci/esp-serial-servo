//------------------------------------------------------------------------
// Filename  :  sts_units.h
// Project   :  ESP Serial Servo
//
// Description:
//    Unit conversions, feedback decoders and the alarm mapping table of the
//    STS family. Header-only, pure functions: no bus access, no state, safe
//    to call from any thread.
//
//    Native units (ST3215 memory table):
//      position       1 step, 4096 steps per revolution = 360000 mdeg
//      speed          1 step/s
//      acceleration   100 steps/s^2
//      torque, load   0.1 % (so the same number as permille)
//      current        6.5 mA
//      voltage        0.1 V
//      protection time 10 ms
//
//    Rounding: conversions to native units round to nearest, halves away from
//    zero. Conversions never saturate; the caller clamps to the register
//    range it needs.
//
//    Feedback encodings (read from the ST3215 register map and the vendor
//    reference code; not yet confirmed on every hardware revision):
//      position  bit 15 sign, bits 0..14 magnitude
//      load      bit 10 sign, bits 0..9 magnitude (0..1000)
//      current   bit 15 sign, bits 0..14 magnitude
//------------------------------------------------------------------------

#ifndef STS_UNITS__H
#define STS_UNITS__H

#include <cstdint>

#include "servokit/core/servo_types.h"
#include "servokit/stseries/stseries_defines.h"

namespace servokit {
namespace STS {
namespace units {

constexpr int64_t STEPS_PER_REV = 4096;
constexpr int64_t MDEG_PER_REV = 360000;
constexpr int64_t CURRENT_UNIT_X2_MA = 13;  ///< 6.5 mA expressed in half milliamps.
constexpr int64_t VOLTAGE_UNIT_MV = 100;
constexpr int64_t TIME_UNIT_MS = 10;

/** Signed division rounding to nearest, halves away from zero. @p den > 0. */
constexpr int64_t round_div(int64_t num, int64_t den) {
  return num >= 0 ? (num + den / 2) / den : -((-num + den / 2) / den);
}

/** Steps to mdeg: steps * 87.890625. */
constexpr int32_t steps_to_mdeg(int32_t steps) {
  return static_cast<int32_t>(round_div(static_cast<int64_t>(steps) * MDEG_PER_REV, STEPS_PER_REV));
}

/** Mdeg to steps, rounded to nearest. The result may lie outside any
 *  register range. */
constexpr int32_t mdeg_to_steps(int32_t mdeg) {
  return static_cast<int32_t>(round_div(static_cast<int64_t>(mdeg) * STEPS_PER_REV, MDEG_PER_REV));
}

/** mdeg/s to steps/s, rounded. */
constexpr uint32_t mdeg_s_to_steps_s(uint32_t mdeg_s) {
  return static_cast<uint32_t>(round_div(static_cast<int64_t>(mdeg_s) * STEPS_PER_REV, MDEG_PER_REV));
}

/** steps/s to mdeg/s, rounded. */
constexpr uint32_t steps_s_to_mdeg_s(uint32_t steps_s) {
  return static_cast<uint32_t>(round_div(static_cast<int64_t>(steps_s) * MDEG_PER_REV, STEPS_PER_REV));
}

/** mdeg/s^2 to acceleration units of 100 steps/s^2, rounded. */
constexpr uint32_t mdeg_s2_to_accel_units(uint32_t mdeg_s2) {
  return static_cast<uint32_t>(round_div(static_cast<int64_t>(mdeg_s2) * STEPS_PER_REV, MDEG_PER_REV * 100));
}

/** Acceleration units of 100 steps/s^2 to mdeg/s^2, rounded. */
constexpr uint32_t accel_units_to_mdeg_s2(uint32_t units) {
  return static_cast<uint32_t>(round_div(static_cast<int64_t>(units) * 100 * MDEG_PER_REV, STEPS_PER_REV));
}

/** Raw current (6.5 mA units, signed) to mA, rounded. */
constexpr int32_t current_units_to_ma(int32_t units) {
  return static_cast<int32_t>(round_div(static_cast<int64_t>(units) * CURRENT_UNIT_X2_MA, 2));
}

/** mA to current units of 6.5 mA, rounded. */
constexpr int32_t ma_to_current_units(int32_t ma) {
  return static_cast<int32_t>(round_div(static_cast<int64_t>(ma) * 2, CURRENT_UNIT_X2_MA));
}

/** Raw voltage (0.1 V units) to mV. */
constexpr uint32_t voltage_units_to_mv(uint32_t units) { return static_cast<uint32_t>(units * VOLTAGE_UNIT_MV); }

/** Milliseconds to protection time units of 10 ms, rounded. */
constexpr uint32_t ms_to_time_units(uint32_t ms) {
  return static_cast<uint32_t>(round_div(static_cast<int64_t>(ms), TIME_UNIT_MS));
}

/** Sign-magnitude decode: @p sign_bit selects the sign, all lower bits are
 *  the magnitude. Negative zero decodes to 0. Bits above the sign bit must be
 *  checked by the caller. */
constexpr int32_t decode_sign_magnitude(uint16_t raw, unsigned sign_bit) {
  const uint16_t sign = static_cast<uint16_t>(1u << sign_bit);
  const int32_t magnitude = raw & (sign - 1u);
  return (raw & sign) != 0 ? -magnitude : magnitude;
}

/** Position feedback in steps (bit 15 sign). */
constexpr int32_t decode_position_steps(uint16_t raw) { return decode_sign_magnitude(raw, 15); }

/** Current feedback in 6.5 mA units (bit 15 sign). */
constexpr int32_t decode_current_units(uint16_t raw) { return decode_sign_magnitude(raw, 15); }

/** Load feedback in permille (bit 10 sign, magnitude 0..1000).
 *  @param[out] out Written on success only.
 *  @return false if a bit above bit 10 is set or the magnitude exceeds 1000. */
constexpr bool decode_load_permille(uint16_t raw, int16_t* out) {
  if ((raw & 0xF800u) != 0) return false;
  const int32_t v = decode_sign_magnitude(raw, 10);
  if (v > 1000 || v < -1000) return false;
  *out = static_cast<int16_t>(v);
  return true;
}

/** One row of the native-to-generic alarm table. */
struct AlarmMapRow {
  uint8_t native_bit;  ///< sts_err_code_t bit
  AlarmSet generic;    ///< Generic alarms reported for it
};

/** REG_STATUS bit to generic alarms. Bit 0 does not say whether the supply is
 *  too high or too low, so it reports both voltage alarms: a caller acting on
 *  either one never misses the fault. */
constexpr AlarmMapRow ALARM_TABLE[] = {
    {STAT_VOLTAGE, AlarmSet(Alarm::OVER_VOLTAGE) | AlarmSet(Alarm::UNDER_VOLTAGE)},
    {STAT_SENSOR, AlarmSet(Alarm::ENCODER)},
    {STAT_TEMPERATURE, AlarmSet(Alarm::OVER_TEMPERATURE)},
    {STAT_CURRENT, AlarmSet(Alarm::OVER_CURRENT)},
    {STAT_ANGLE, AlarmSet(Alarm::ANGLE_LIMIT)},
    {STAT_OVERLOAD, AlarmSet(Alarm::OVERLOAD)},
};

/** Bits of a native status byte that have a row in ALARM_TABLE. */
constexpr uint8_t MAPPED_STATUS_BITS = 0x3F;

/** Translates a native status byte. Every set bit is reported: bits without a
 *  table row (6, 7) set Alarm::OTHER, so no fault is dropped. */
constexpr AlarmSet alarms_from_native(uint8_t status) {
  AlarmSet out;
  for (const AlarmMapRow& row : ALARM_TABLE) {
    if ((status & row.native_bit) != 0) out |= row.generic;
  }
  if ((status & ~MAPPED_STATUS_BITS) != 0) out.set(Alarm::OTHER);
  return out;
}

/** Alarms an STS servo can report (every generic alarm that has a table row,
 *  plus OTHER). */
constexpr AlarmSet reportable_alarms() {
  AlarmSet out(Alarm::OTHER);
  for (const AlarmMapRow& row : ALARM_TABLE) out |= row.generic;
  return out;
}

/** Native mask for a generic alarm set, for REG_UNLOAD_ALARM_MASK.
 *  @param[out] native Written on success only.
 *  @return false if @p alarms contains an alarm with no native bit. */
constexpr bool native_from_alarms(AlarmSet alarms, uint8_t* native) {
  uint8_t mask = 0;
  AlarmSet covered;
  for (const AlarmMapRow& row : ALARM_TABLE) {
    if (alarms.has_any(row.generic)) mask = static_cast<uint8_t>(mask | row.native_bit);
    covered |= row.generic;
  }
  if (!covered.has_all(alarms)) return false;
  *native = mask;
  return true;
}

}  // namespace units
}  // namespace STS
}  // namespace servokit
#endif  // STS_UNITS__H
