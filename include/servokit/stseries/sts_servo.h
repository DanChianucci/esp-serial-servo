//------------------------------------------------------------------------
// Filename  :  sts_servo.h
// Project   :  ESP Serial Servo
//
// Description:
//    STS-series servo: register-level access plus typed helpers in the
//    family's native units.
//
//    Protocol (ST3215 communication manual and memory table):
//      Command   FF FF id len instr params... chk   (len = params + 2)
//      Response  FF FF id len status data... chk    (len = data + 2)
//      chk       ~(id + len + instr/status + params/data), low byte.
//      Words     Little endian: low byte at the lower register address.
//      Status    0 = healthy; otherwise a bitmask of sts_err_code_t.
//
//    Contract common to every call:
//      Blocking  Synchronous. One call is one command/response transaction,
//                bounded by the controller's transaction budget; no retries.
//      Threads   Not thread-safe, and neither is the bus. Serialize all calls
//                per bus (including checksum accessors).
//      Errors    Returned, never thrown: TIMEOUT (no complete response),
//                BAD_RESPONSE (framing, checksum, wrong ID, wrong length, or
//                the servo reported an error status; see last_status()),
//                OUT_OF_RANGE (register span outside the control table),
//                PARAM (null pointer, empty or oversized span, broadcast or
//                invalid ID, no controller), BUS (transport failure).
//      Lifetimes Pointer arguments are used only during the call and never
//                retained. Output buffers are written on OK only.
//      Replies   A servo whose REG_RESPONSE_MODE is RSP_READ_PING sends no
//                reply to writes; write calls then return TIMEOUT even though
//                the write took effect.
//
//    Typed helpers (set_*/get_*):
//      Target    The servo whose ID is device_id(). A broadcast or invalid ID
//                makes every helper that uses the bus return PARAM.
//      Ranges    Only hardware ranges are enforced. An out-of-range argument
//                returns OUT_OF_RANGE and sends nothing.
//      Units     Native register units, stated per helper.
//      EEPROM    Registers in the EEPROM region are written like any other
//                register, by read-compare-write: nothing is sent when the
//                register already holds the requested value. Whether a value
//                survives a power cycle is decided by REG_EEPROM_LOCK: with the
//                lock off (0) it is stored, with the lock on (1) it takes
//                effect but is lost at power-off. Helpers never change the
//                lock unless called with persist = true, which unlocks, writes
//                and locks again, attempting the lock on every exit path.
//                Without persist the lock is not touched, so persistence
//                follows whatever the lock currently is. EEPROM helpers return
//                EepromResult, which names the step that failed. Nothing in
//                this class writes EEPROM implicitly.
//      Compare   The compare is against the live register value, so a value
//                written while the lock was on is not re-stored by a later
//                persist call that requests the same value.
//      Raw       The raw register calls above stay independent of these
//                helpers; no register value is cached between calls.
//------------------------------------------------------------------------

#ifndef STS_SERVO__H
#define STS_SERVO__H

#include <cstddef>
#include <cstdint>
#include <memory>

#include "servokit/core/servo.h"
#include "servokit/core/servo_types.h"
#include "servokit/serial/servo_controller.h"
#include "servokit/stseries/stseries_defines.h"

namespace servokit {

/** Step of an EEPROM-region update that failed. */
enum class EepromStep : uint8_t {
  NONE = 0,  ///< No step failed.
  READ,      ///< Reading the current value (the read-compare).
  UNLOCK,    ///< Writing REG_EEPROM_LOCK to unlock (persist only).
  WRITE,     ///< Writing the new value.
  RELOCK     ///< Writing REG_EEPROM_LOCK to lock again (persist only).
};

/** Outcome of an EEPROM-region helper. */
struct EepromResult {
  Result result = Result::OK;                 ///< First failure, or OK.
  EepromStep failed_step = EepromStep::NONE;  ///< Step that produced @c result.
  bool wrote = false;                         ///< The value write was acknowledged (false when skipped as identical).
  bool relock_failed = false;                 ///< Persist only: the lock could not be restored after an earlier
                                              ///< failure; the servo may be left unlocked.
  bool ok() const { return result == Result::OK; }
};

class STSServo : public Servo {
 public:
  // Hardware limits of the typed helpers.
  static constexpr int32_t MAX_TARGET_POSITION = 30719;   ///< Target position magnitude, steps.
  static constexpr int32_t MAX_SPEED = 3400;              ///< Speed magnitude, steps/s.
  static constexpr uint8_t MAX_ACCELERATION = 254;        ///< Acceleration, units of 100 steps/s^2.
  static constexpr uint16_t MAX_TORQUE = 1000;            ///< Torque, units of 0.1 %.
  static constexpr uint8_t MAX_RESPONSE_DELAY = 254;      ///< Response delay, units of 2 us.
  static constexpr uint8_t MAX_TEMPERATURE_LIMIT = 100;   ///< Over-temperature threshold, degC.
  static constexpr uint8_t MAX_VOLTAGE_LIMIT = 254;       ///< Voltage thresholds, units of 0.1 V.
  static constexpr uint16_t MAX_CURRENT_LIMIT = 511;      ///< Over-current threshold, units of 6.5 mA.
  static constexpr uint8_t MAX_PROTECTION_TIME = 254;     ///< Protection durations, units of 10 ms.
  static constexpr uint8_t MAX_OVERLOAD_LIMIT = 100;      ///< Overload threshold, percent of maximum torque.
  static constexpr uint8_t ALARM_MASK_BITS = 0x3F;        ///< Defined sts_err_code_t bits (0..5).
  static constexpr uint8_t PHASE_MULTITURN_BIT = 1 << 4;  ///< REG_PHASE bit enabling multi-turn feedback.

  /** Highest register address plus one: the control table is addressed by one byte. */
  static constexpr size_t REGISTER_SPACE = 256;
  /** Longest register span (bytes) one read or write call accepts. */
  static constexpr size_t MAX_SPAN = 64;

  /** Borrows a controller.
   *  @param controller Transaction engine; must outlive this object. Null makes
   *                    every bus call return PARAM.
   *  @param id Device ID used by the Servo overrides (ping(), read_raw(),
   *            write_raw()). */
  explicit STSServo(ServoController* controller, uint8_t id = 1);

  /** Takes ownership of a controller (destroyed with this object).
   *  The controller's SerialPort must outlive this object. */
  explicit STSServo(std::unique_ptr<ServoController> controller, uint8_t id = 1);

  /** Device ID used by the Servo overrides. Does not touch the bus. */
  uint8_t device_id() const { return m_id; }
  void set_device_id(uint8_t id) { m_id = id; }

  /** Status byte of the most recent valid response (a bitmask of
   *  sts_err_code_t, 0 when healthy). Reset to 0 at the start of every call;
   *  it stays 0 when no valid response arrived. */
  uint8_t last_status() const { return m_last_status; }

  /** Checks that the servo with @p id answers.
   *  @param id Unicast device ID, 0..0xFD (0xFE broadcast is rejected: it would
   *            make every servo on the bus answer at once).
   *  @return OK, or an error as described in the file header. */
  Result ping(uint8_t id);

  /** Reads @p len bytes from register @p addr of servo @p id.
   *  @param addr First register address; addr + len must not exceed REGISTER_SPACE.
   *  @param[out] data Caller buffer of at least @p len bytes; written on OK only.
   *  @param len 1..MAX_SPAN. */
  Result read_register(uint8_t id, uint16_t addr, uint8_t* data, size_t len);

  /** Writes @p len bytes to register @p addr of servo @p id. Same limits as
   *  read_register(). Registers are written as given: no EEPROM-lock handling
   *  and no read-compare. */
  Result write_register(uint8_t id, uint16_t addr, const uint8_t* data, size_t len);

  /** Reads one byte register. @param[out] out Written on OK only. */
  Result read_u8(uint8_t id, uint16_t addr, uint8_t* out);
  /** Reads a 16-bit register (little endian, @p addr = low byte). */
  Result read_u16(uint8_t id, uint16_t addr, uint16_t* out);
  /** Writes one byte register. */
  Result write_u8(uint8_t id, uint16_t addr, uint8_t value);
  /** Writes a 16-bit register (little endian, @p addr = low byte). */
  Result write_u16(uint8_t id, uint16_t addr, uint16_t value);

  // --- Typed helpers (see "Typed helpers" in the file header) ---
  // Every EEPROM-region setter takes @p persist (default false): see the
  // EEPROM paragraph of the file header.

  /** Reads REG_OPERATION_MODE. @param[out] mode Written on OK only.
   *  @return BAD_RESPONSE if the servo reports a value that is not a mode. */
  Result get_operating_mode(STS::op_mode_t* mode);
  /** Sets REG_OPERATION_MODE. @param mode OP_SERVO..OP_STEP; anything else is
   *  OUT_OF_RANGE. The mode decides how the REG_CTRL_* registers are read. */
  EepromResult set_operating_mode(STS::op_mode_t mode, bool persist = false);

  /** Reads REG_RESPONSE_MODE. @param[out] mode Written on OK only. */
  Result get_response_mode(STS::rsp_mode_t* mode);
  /** Sets REG_RESPONSE_MODE. With RSP_READ_PING the servo stops answering
   *  writes, so later write calls return TIMEOUT. */
  EepromResult set_response_mode(STS::rsp_mode_t mode, bool persist = false);
  /** Reads REG_RESPONSE_DELAY. @param[out] delay_2us Units of 2 us. */
  Result get_response_delay(uint8_t* delay_2us);
  /** Sets REG_RESPONSE_DELAY. @param delay_2us 0..MAX_RESPONSE_DELAY, units of 2 us. */
  EepromResult set_response_delay(uint8_t delay_2us, bool persist = false);

  /** Reads REG_SERVO_EN. @param[out] enabled true when torque output is on. */
  Result get_servo_enabled(bool* enabled);
  /** Writes REG_SERVO_EN (SRAM): true turns torque output on, false off.
   *  Never writes the mid-point calibration value. */
  Result set_servo_enabled(bool enabled);

  /** Writes REG_CTRL_ACCEL (SRAM). @param accel 0..MAX_ACCELERATION, units of 100 steps/s^2. */
  Result set_acceleration(uint8_t accel);
  /** Writes REG_CTRL_SPEED as an unsigned magnitude (SRAM), the speed used by
   *  position moves. @param steps_per_s 0..MAX_SPEED. */
  Result set_speed(uint16_t steps_per_s);
  /** Writes REG_CTRL_SPEED as sign-magnitude (bit 15 = reverse) for constant
   *  speed mode (SRAM). @param steps_per_s -MAX_SPEED..MAX_SPEED. */
  Result set_signed_speed(int32_t steps_per_s);
  /** Writes REG_CTRL_TORQUE (SRAM), the runtime torque limit.
   *  @param torque 0..MAX_TORQUE, units of 0.1 %. Reloaded from the stored
   *  maximum torque at power-up. */
  Result set_torque_limit(uint16_t torque);
  /** Writes REG_START_TORQUE, the minimum starting torque.
   *  @param torque 0..MAX_TORQUE, units of 0.1 %. */
  EepromResult set_start_torque(uint16_t torque, bool persist = false);

  /** Writes REG_CTRL_POS (SRAM) as sign-magnitude (bit 15 = negative).
   *  @param steps -MAX_TARGET_POSITION..MAX_TARGET_POSITION, 4096 steps per
   *  revolution. Values beyond +-MAX_TARGET_POSITION return OUT_OF_RANGE. */
  Result set_target_position(int32_t steps);

  /** Reads REG_EEPROM_LOCK. @param[out] locked true when the lock register is 1. */
  Result get_eeprom_locked(bool* locked);
  /** Writes REG_EEPROM_LOCK = 0 so EEPROM-region writes are stored. */
  Result unlock_eeprom();
  /** Writes REG_EEPROM_LOCK = 1 so EEPROM-region writes are volatile. */
  Result lock_eeprom();

  /** Reads the REG_PHASE multi-turn feedback bit. @param[out] enabled Written on OK only. */
  Result get_multiturn_feedback(bool* enabled);
  /** Sets or clears only the multi-turn feedback bit of REG_PHASE; the other
   *  bits are preserved. */
  EepromResult set_multiturn_feedback(bool enabled, bool persist = false);

  // Protection setters. Thresholds and times are in native register units;
  // their meaning on a given servo revision is not verified here.
  /** REG_OVERTEMP_THRESH, 0..MAX_TEMPERATURE_LIMIT degC. */
  EepromResult set_overtemp_threshold(uint8_t celsius, bool persist = false);
  /** REG_OVERVOLT_THRESH, 0..MAX_VOLTAGE_LIMIT, units of 0.1 V. */
  EepromResult set_overvoltage_threshold(uint8_t decivolts, bool persist = false);
  /** REG_UNDERVOLT_THRESH, 0..MAX_VOLTAGE_LIMIT, units of 0.1 V. */
  EepromResult set_undervoltage_threshold(uint8_t decivolts, bool persist = false);
  /** REG_OVERCURR_THRESH (16-bit), 0..MAX_CURRENT_LIMIT, units of 6.5 mA. */
  EepromResult set_overcurrent_threshold(uint16_t units, bool persist = false);
  /** REG_OVERCURR_TIME, 0..MAX_PROTECTION_TIME, units of 10 ms. */
  EepromResult set_overcurrent_time(uint8_t units_10ms, bool persist = false);
  /** REG_OVERLOAD_THRESH, 0..MAX_OVERLOAD_LIMIT percent of maximum torque. */
  EepromResult set_overload_threshold(uint8_t percent, bool persist = false);
  /** REG_OVERLOAD_TIME, 0..MAX_PROTECTION_TIME, units of 10 ms. */
  EepromResult set_overload_time(uint8_t units_10ms, bool persist = false);
  /** REG_LED_ALARM_MASK: bitmask of sts_err_code_t whose fault flashes the
   *  LED. Bits above ALARM_MASK_BITS return OUT_OF_RANGE. */
  EepromResult set_led_alarm_mask(uint8_t mask, bool persist = false);
  /** REG_UNLOAD_ALARM_MASK: bitmask of sts_err_code_t whose fault releases
   *  torque. Bits above ALARM_MASK_BITS return OUT_OF_RANGE. */
  EepromResult set_unload_alarm_mask(uint8_t mask, bool persist = false);

  // --- Servo overrides ---
  using Servo::ping;
  Result ping() override;
  Result read_raw(uint16_t addr, uint8_t* data, size_t len) override;
  Result write_raw(uint16_t addr, const uint8_t* data, size_t len) override;
  Result set_verify_checksum(bool enable) override;
  bool verify_checksum() const override;
  uint32_t checksum_errors() const override;

  // Placeholders; the family API supplies the real behaviour.
  Result move_to(int32_t angle_mdeg) override;
  Capabilities capabilities() const override;

 private:
  /** Read-compare-write of @p len (1 or 2) bytes at @p addr. @p mask
   *  (nullable, 1-byte registers only) selects the bits to change. */
  EepromResult eeprom_update(uint16_t addr, const uint8_t* value, const uint8_t* mask, size_t len, bool persist);
  EepromResult eeprom_update_u8(uint16_t addr, uint8_t value, bool persist);
  EepromResult eeprom_update_u16(uint16_t addr, uint16_t value, bool persist);
  Result read_bool(uint16_t addr, bool* out);
  Result transact(uint8_t id, uint8_t instruction, const uint8_t* params, size_t param_len, uint8_t* out,
                  size_t out_len);

  std::unique_ptr<ServoController> m_owned;
  ServoController* m_ctl;
  uint8_t m_id;
  uint8_t m_last_status = 0;
};

}  // namespace servokit
#endif  // STS_SERVO__H
