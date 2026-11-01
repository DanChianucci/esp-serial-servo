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
//                per bus (including the SerialLink and RegisterAccess calls).
//      Errors    Returned, never thrown: TIMEOUT (no complete response),
//                BAD_RESPONSE (framing, checksum, wrong ID, wrong length, or
//                the servo reported an error status; see get_last_status()),
//                OUT_OF_RANGE (register span outside the control table),
//                PARAM (null pointer, empty or oversized span, broadcast or
//                invalid ID, no controller), BUS (transport failure).
//      Lifetimes Pointer arguments are used only during the call and never
//                retained. Output buffers are written on OK only.
//      Replies   A servo whose REG_RESPONSE_MODE is RSP_READ_PING sends no
//                reply to writes; write calls then return TIMEOUT even though
//                the write took effect.
//
//    Readback check (set_readback_check(), off by default):
//      Effect    After a successful write of a register span, one read of the
//                same span is sent and each byte is compared with the byte
//                written, through that byte's compare mask (see readback_rule()).
//                The read is addressed to the written ID, except that a write
//                covering REG_DEVICE_ID reads from the newly written ID. When
//                that ID is above STS::MAX_ID (0xFC) the servo cannot be
//                addressed, so nothing is read back and the write returns OK.
//      Scope     Done once in write_register(), so write_u8(), write_u16(),
//                the RegisterAccess calls, every typed helper and every generic operation
//                that writes are checked alike. Exactly one extra read frame
//                per write that has at least one compared byte; none for a
//                span with only volatile or unlisted bytes. With the setting
//                off, no extra frame is sent.
//      Results   A write error is returned as is and nothing is read. A failed
//                read-back returns that read's own error (the write took
//                effect). A difference in a compared bit returns VERIFY_FAILED
//                (the write took effect; the servo holds a different value).
//      Registers Each writable register is classified in readback_rule():
//                  STABLE         compared in full;
//                  STABLE_MASKED  compared only in the bits of its mask, the
//                                 other bits having self-clearing, write-only
//                                 or hardware-updated behaviour;
//                  VOLATILE       the servo may change the value on its own,
//                                 so it is never compared. This means "changes
//                                 behind the scenes", not "lost at power-off".
//                A register absent from the table (read-only registers, gaps,
//                addresses beyond the map) is treated as VOLATILE and is never
//                compared. A multi-register span applies each byte's own
//                classification.
//      Table     (address, bytes, register: class)
//                0x05 1 DEVICE_ID           STABLE
//                0x06 1 BAUD_RATE           VOLATILE (the link rate may change
//                                           at once, so the read-back may not
//                                           be reachable)
//                0x07 1 RESPONSE_DELAY      STABLE
//                0x08 1 RESPONSE_MODE       STABLE
//                0x09 2 MIN_ANGLE_THRESH    STABLE
//                0x0B 2 MAX_ANGLE_THRESH    STABLE
//                0x0D 1 OVERTEMP_THRESH     STABLE
//                0x0E 1 OVERVOLT_THRESH     STABLE
//                0x0F 1 UNDERVOLT_THRESH    STABLE
//                0x10 2 CONTROL_TORQUE_NV   STABLE
//                0x12 1 PHASE               STABLE_MASKED, PHASE_READBACK_MASK
//                0x13 1 UNLOAD_ALARM_MASK   STABLE
//                0x14 1 LED_ALARM_MASK      STABLE
//                0x15 1 POS_COEF_P          STABLE
//                0x16 1 POS_COEF_D          STABLE
//                0x17 1 POS_COEF_I          STABLE
//                0x18 2 START_TORQUE        STABLE
//                0x1A 1 CW_DEADZONE         STABLE
//                0x1B 1 CCW_DEADZONE        STABLE
//                0x1C 2 OVERCURR_THRESH     STABLE
//                0x1E 1 ANGLE_RESOLUTION    STABLE
//                0x1F 2 POS_OFFSET          STABLE
//                0x21 1 OPERATION_MODE      STABLE
//                0x22 1 UNLOAD_CTRL_TORQUE  STABLE
//                0x23 1 OVERLOAD_TIME       STABLE
//                0x24 1 OVERLOAD_THRESH     STABLE
//                0x25 1 SPEED_COEF_P        STABLE
//                0x26 1 OVERCURR_TIME       STABLE
//                0x27 1 SPEED_COEF_I        STABLE
//                0x28 1 SERVO_EN            VOLATILE (a protection trip can
//                                           clear it, and the calibration value
//                                           128 is a command that may read back
//                                           differently; torque-enable writes
//                                           are never read back)
//                0x29 1 CTRL_ACCEL          VOLATILE
//                0x2A 2 CTRL_POS            VOLATILE
//                0x2C 2 CTRL_PWM            VOLATILE
//                0x2E 2 CTRL_SPEED          VOLATILE
//                0x30 2 CTRL_TORQUE         VOLATILE
//                0x37 1 EEPROM_LOCK         STABLE
//
//    Typed helpers (set_*/get_*):
//      Target    The servo whose ID is get_device_id(). A broadcast or invalid ID
//                makes every helper that uses the bus return PARAM.
//      Ranges    Only hardware ranges are enforced. An out-of-range argument
//                returns OUT_OF_RANGE and sends nothing.
//      Units     Native register units, stated per helper.
//      Writes    Every setter performs exactly one register write (plus the
//                verification read when enabled) and returns its Result. No setter reads before it writes
//                or touches REG_EEPROM_LOCK; with the readback check enabled,
//                the write is followed by a single read-back and compare. The one exception is
//                set_multiturn_feedback(), which reads REG_PHASE first because
//                only one bit of that byte may change.
//      EEPROM    Whether a configuration write survives a power cycle is
//                decided by REG_EEPROM_LOCK: with the lock off (0) it is
//                stored; with the lock on (1) it takes effect but is not saved
//                across a power cycle (per the register map; not verified on
//                hardware). The caller sequences unlock_eeprom(), the writes,
//                then lock_eeprom(); nothing relocks automatically. A failed
//                write leaves the lock as it was.
//      Registers The register calls above stay independent of these
//                helpers; no register value is cached between calls.
//------------------------------------------------------------------------

#ifndef STS_SERVO__H
#define STS_SERVO__H

#include <cstddef>
#include <cstdint>
#include <memory>

#include "servokit/core/register_access.h"
#include "servokit/core/servo.h"
#include "servokit/core/servo_types.h"
#include "servokit/serial/serial_link.h"
#include "servokit/serial/servo_controller.h"
#include "servokit/families/stseries/stseries_defines.h"
#include "servokit/families/stseries/sts_units.h"

namespace servokit {

class STSServo : public Servo, public RegisterAccess, public SerialLink {
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
  /** Stall torque of the ST3215 in mN*m: 19.5 kgf*cm at a supply of 7.4 V with
   *  the rotor locked (manufacturer datasheet). It is lower at lower supply
   *  voltages (16.5 kgf*cm at 6 V). Informational only: torque commands use
   *  permille. */
  static constexpr uint32_t STALL_TORQUE_MNM = 1912;
  static constexpr uint8_t ALARM_MASK_BITS = 0x3F;        ///< Defined sts_err_code_t bits (0..5).
  static constexpr uint8_t PHASE_MULTITURN_BIT = 1 << 4;  ///< REG_PHASE bit enabling multi-turn feedback.
  /** REG_PHASE bits compared by the readback check: every bit except bit 3
   *  (direction) and bit 6 (speed mode). The register map does not document
   *  the bit meanings; the excluded bits are those community documentation
   *  assigns to behaviour the servo may rewrite, unverified on hardware. */
  static constexpr uint8_t PHASE_READBACK_MASK = 0xFF & ~((1 << 3) | (1 << 6));

  /** How the readback check treats a register byte. */
  enum class ReadbackClass : uint8_t {
    STABLE,         ///< Compared in full.
    STABLE_MASKED,  ///< Compared in the bits of ReadbackRule::mask only.
    VOLATILE        ///< Never compared (also: absent from the table).
  };
  /** Compare rule of one register byte. */
  struct ReadbackRule {
    ReadbackClass kind;  ///< Classification.
    uint8_t mask;      ///< Bits compared: 0xFF STABLE, a subset STABLE_MASKED, 0 VOLATILE.
  };
  /** Readback rule of the register byte at @p addr, from the table in
   *  the file header. Pure; touches no bus. A byte absent from the table (any
   *  read-only register, a gap, an address of 256 or more) is VOLATILE with
   *  mask 0, so it is never compared. Every byte of a multi-byte register has
   *  the register's classification. */
  static ReadbackRule readback_rule(uint16_t addr);

  /** Highest register address plus one: the control table is addressed by one byte. */
  static constexpr size_t REGISTER_SPACE = 256;
  /** Longest register span (bytes) one read or write call accepts. */
  static constexpr size_t MAX_SPAN = 64;

  /** Borrows a controller.
   *  @param controller Transaction engine; must outlive this object. Null makes
   *                    every bus call return PARAM.
   *  @param id Device ID used by the Servo and RegisterAccess overrides. */
  explicit STSServo(ServoController* controller, uint8_t id = 1);

  /** Takes ownership of a controller (destroyed with this object).
   *  The controller's SerialPort must outlive this object. */
  explicit STSServo(std::unique_ptr<ServoController> controller, uint8_t id = 1);

  /** Device ID used by the Servo overrides. Does not touch the bus. */
  uint8_t get_device_id() const { return m_id; }
  void set_device_id(uint8_t id) { m_id = id; }

  /** Status byte of the most recent valid response (a bitmask of
   *  sts_err_code_t, 0 when healthy). Reset to 0 at the start of every call;
   *  it stays 0 when no valid response arrived. */
  uint8_t get_last_status() const { return m_last_status; }

  /** Checks that the servo with @p id answers.
   *  @param id Unicast device ID, 0..0xFD (0xFE broadcast is rejected: it would
   *            make every servo on the bus answer at once).
   *  @return OK, or an error as described in the file header. */
  Result probe(uint8_t id);

  /** Reads @p len bytes from register @p addr of servo @p id.
   *  @param addr First register address; addr + len must not exceed REGISTER_SPACE.
   *  @param[out] data Caller buffer of at least @p len bytes; written on OK only.
   *  @param len 1..MAX_SPAN. */
  Result read_register(uint8_t id, uint16_t addr, uint8_t* data, size_t len);

  /** Writes @p len bytes to register @p addr of servo @p id. Same limits as
   *  read_register(). Registers are written as given, without EEPROM-lock
   *  handling. With get_readback_check() on, a successful write is followed by one
   *  read of the span, compared per readback_rule(); returns VERIFY_FAILED on a
   *  difference or the read's own error. See "Readback check" in the file
   *  header. */
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
  // Setters for EEPROM-region registers write them like any other register;
  // see the EEPROM paragraph of the file header for the lock.

  /** Reads REG_OPERATION_MODE. @param[out] mode Written on OK only.
   *  @return BAD_RESPONSE if the servo reports a value that is not a mode. */
  Result get_operating_mode(STS::op_mode_t* mode);
  /** Sets REG_OPERATION_MODE. @param mode OP_SERVO..OP_STEP; anything else is
   *  OUT_OF_RANGE. The mode decides how the REG_CTRL_* registers are read. */
  Result set_operating_mode(STS::op_mode_t mode);

  /** Reads REG_RESPONSE_MODE. @param[out] mode Written on OK only. */
  Result get_response_mode(STS::rsp_mode_t* mode);
  /** Sets REG_RESPONSE_MODE. With RSP_READ_PING the servo stops answering
   *  writes, so later write calls return TIMEOUT. */
  Result set_response_mode(STS::rsp_mode_t mode);
  /** Reads REG_RESPONSE_DELAY. @param[out] delay_2us Units of 2 us. */
  Result get_response_delay(uint8_t* delay_2us);
  /** Sets REG_RESPONSE_DELAY. @param delay_2us 0..MAX_RESPONSE_DELAY, units of 2 us. */
  Result set_response_delay(uint8_t delay_2us);

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
  Result set_start_torque(uint16_t torque);

  /** Writes REG_CTRL_POS (SRAM) as sign-magnitude (bit 15 = negative).
   *  @param steps -MAX_TARGET_POSITION..MAX_TARGET_POSITION, 4096 steps per
   *  revolution. Values beyond +-MAX_TARGET_POSITION return OUT_OF_RANGE. */
  Result set_target_position(int32_t steps);

  /** Reads REG_EEPROM_LOCK. @param[out] locked true when the lock register is 1. */
  Result get_eeprom_locked(bool* locked);
  /** Writes REG_EEPROM_LOCK = 0 so EEPROM-region writes are stored. */
  Result unlock_eeprom();
  /** Writes REG_EEPROM_LOCK = 1: EEPROM-region writes take effect but are not
   *  saved across a power cycle. */
  Result lock_eeprom();

  /** Reads the REG_PHASE multi-turn feedback bit. @param[out] enabled Written on OK only. */
  Result get_multiturn_feedback(bool* enabled);
  /** Sets or clears only the multi-turn feedback bit of REG_PHASE; the other
   *  bits are preserved. Reads REG_PHASE, then writes the modified byte (two
   *  frames); a failed read sends no write. */
  Result set_multiturn_feedback(bool enabled);

  // Protection setters. Thresholds and times are in native register units;
  // their meaning on a given servo revision is not verified here.
  /** REG_OVERTEMP_THRESH, 0..MAX_TEMPERATURE_LIMIT degC. */
  Result set_overtemp_threshold(uint8_t celsius);
  /** REG_OVERVOLT_THRESH, 0..MAX_VOLTAGE_LIMIT, units of 0.1 V. */
  Result set_overvoltage_threshold(uint8_t decivolts);
  /** REG_UNDERVOLT_THRESH, 0..MAX_VOLTAGE_LIMIT, units of 0.1 V. */
  Result set_undervoltage_threshold(uint8_t decivolts);
  /** REG_OVERCURR_THRESH (16-bit), 0..MAX_CURRENT_LIMIT, units of 6.5 mA. */
  Result set_overcurrent_threshold(uint16_t units);
  /** REG_OVERCURR_TIME, 0..MAX_PROTECTION_TIME, units of 10 ms. */
  Result set_overcurrent_time(uint8_t units_10ms);
  /** REG_OVERLOAD_THRESH, 0..MAX_OVERLOAD_LIMIT percent of maximum torque. */
  Result set_overload_threshold(uint8_t percent);
  /** REG_OVERLOAD_TIME, 0..MAX_PROTECTION_TIME, units of 10 ms. */
  Result set_overload_time(uint8_t units_10ms);
  /** REG_LED_ALARM_MASK: bitmask of sts_err_code_t whose fault flashes the
   *  LED. Bits above ALARM_MASK_BITS return OUT_OF_RANGE. */
  Result set_led_alarm_mask(uint8_t mask);
  /** REG_UNLOAD_ALARM_MASK: bitmask of sts_err_code_t whose fault releases
   *  torque. Bits above ALARM_MASK_BITS return OUT_OF_RANGE. */
  Result set_unload_alarm_mask(uint8_t mask);

  // --- Servo overrides ---
  using Servo::probe;
  /** probe(get_device_id()). */
  Result probe() override;
  /** true calls unlock_eeprom(), false calls lock_eeprom(). Keeps no state. */
  Result set_persist(bool persist) override;
  /** @return this, as a RegisterAccess for the stored device ID. */
  RegisterAccess* get_registers() override { return this; }
  /** @return this, as a SerialLink over the controller. */
  SerialLink* get_serial_link() override { return this; }

  // --- RegisterAccess (the stored device ID) ---
  // Same as the explicit-ID calls above with id = get_device_id(). The explicit-ID
  // overloads stay available on STSServo.
  using RegisterAccess::read_register;
  using RegisterAccess::read_u16;
  using RegisterAccess::read_u8;
  using RegisterAccess::write_register;
  using RegisterAccess::write_u16;
  using RegisterAccess::write_u8;
  Result read_register(uint16_t addr, uint8_t* data, size_t len) override;
  Result write_register(uint16_t addr, const uint8_t* data, size_t len) override;
  Result read_u8(uint16_t addr, uint8_t* out) override;
  Result read_u16(uint16_t addr, uint16_t* out) override;
  Result write_u8(uint16_t addr, uint8_t value) override;
  Result write_u16(uint16_t addr, uint16_t value) override;
  /** Turns the readback check on or off; see "Readback check" in the file
   *  header. Touches no bus. Always OK. */
  Result set_readback_check(bool enable) override;
  bool get_readback_check() const override { return m_readback_check; }

  // --- SerialLink (the controller's settings) ---
  Result set_verify_checksum(bool enable) override;
  bool get_verify_checksum() const override;
  uint32_t get_checksum_errors() const override;

  // --- Generic Servo implementation (see sts_servo_generic.cpp) ---
  // Built on the register helpers above. Angles use 4096 steps per revolution
  // (87.890625 mdeg/step). Every write is performed as given: no operation
  // here reads before writing, compares, or changes REG_EEPROM_LOCK; the
  // caller sequences set_persist() around configuration writes. Atomic
  // telemetry reads and the position read of hold() are the only reads.

  /** Absolute move: converts @p angle_mdeg to steps (rounded to nearest) and
   *  writes REG_CTRL_POS once.
   *  @return OK, OUT_OF_RANGE outside get_capabilities().limits angle domain
   *          (nothing sent), or the write's error. */
  Result move_to(int32_t angle_mdeg) override;

  /** Fixed description of the STS family; does not touch the bus and does not
   *  depend on identify(). Features are those the generic operations
   *  below perform. The current range is the reportable range: the largest
   *  magnitude of the 15-bit current feedback, so a telemetry sample never lies
   *  outside it. The range a stall-current threshold may be set to is narrower
   *  (MAX_CURRENT_LIMIT units); set_protection() rejects larger values. Not asserted because they are unverified on hardware:
   *  MULTI_TURN_POS and RETAINS_POSITION_UNPOWERED. The domain is the
   *  -MAX_TARGET_POSITION..MAX_TARGET_POSITION steps; resolution_mdeg is the step size
   *  87.890625 mdeg rounded to 88. */
  Capabilities get_capabilities() const override;

  /** Reads the model, firmware version and device ID registers in one
   *  transaction. The device ID is the value stored in the servo. */
  Result identify(Identity* out) override;

  /** Writes REG_RESPONSE_MODE = RSP_ALL, then REG_OPERATION_MODE = OP_SERVO
   *  (position mode). That position mode is the right one for the
   *  generic operations is not verified on hardware. Two plain writes, no reads. If the servo was in RSP_READ_PING
   *  its reply to the first write is missing: that write takes effect but
   *  TIMEOUT is returned and the second write is not sent; call again. */
  Result prepare() override;

  /** Programs stall_current_ma into REG_OVERCURR_THRESH (6.5 mA units),
   *  stall_time_ms into REG_OVERCURR_TIME (10 ms units), max_temperature_c
   *  into REG_OVERTEMP_THRESH and release_on_alarm into REG_UNLOAD_ALARM_MASK.
   *  All values are validated first; any value that does not fit a register,
   *  or an alarm with no native bit (STALLED, COMMUNICATION, UNLOADED, OTHER),
   *  returns OUT_OF_RANGE and sends nothing. Values are never clamped because
   *  that would silently weaken a threshold. Zero is written as given.
   *  Overload and voltage thresholds are not part of Protection and are not
   *  written. OVER_VOLTAGE and UNDER_VOLTAGE share one native bit (the servo
   *  does not distinguish them), so requesting either releases torque on both.
   *  Four plain writes, no reads; stops at the first failure. The
   *  units and the meaning of the thresholds are unverified on hardware. */
  Result set_protection(const Protection& protection) override;

  /** Writes REG_CTRL_TORQUE, REG_START_TORQUE, REG_CTRL_ACCEL, REG_CTRL_SPEED
   *  (in that order, one plain write each, no reads). Speed, acceleration and
   *  torques are clamped to get_capabilities().limits and converted to native
   *  units. REG_START_TORQUE is an EEPROM-region register, so it follows the
   *  lock state. Stops at the first failure. */
  Result set_profile(const Profile& profile) override;

  /** Reads REG_POSITION, then writes it as the target (REG_CTRL_POS), so the
   *  servo stops where it is. Torque must already be enabled. Two transactions.
   *  Returns the failing sub-operation's error; get_hold_failed_step() says which.
   *  A position read outside -MAX_TARGET_POSITION..MAX_TARGET_POSITION returns
   *  OUT_OF_RANGE with POSITION_OUT_OF_DOMAIN and nothing is written.
   *  A servo that reports an alarm in its status byte still yields its
   *  position. Never reports OK for a write that was not acknowledged. */
  Result hold() override;

  /** Sub-operation that failed in the most recent hold(). */
  enum class HoldStep : uint8_t { NONE, READ_POSITION, POSITION_OUT_OF_DOMAIN, WRITE_TARGET };
  HoldStep get_hold_failed_step() const { return m_hold_step; }

  /** Writes REG_SERVO_EN (true = torque on). */
  Result set_torque(bool enable) override;

  /** One atomic sample from a single 15-byte read of REG_POSITION..REG_CURRENT.
   *  Filled on OK only; any transport error, short reply or undecodable field
   *  (load outside +-1000 permille, moving flag other than 0/1) fails the
   *  whole call. A servo that reports an alarm still returns its sample, with
   *  the alarm in Telemetry::alarms (REG_STATUS and the reply status byte
   *  combined). Position is single-turn feedback; the load and current
   *  encodings are unverified on hardware. */
  Result read_telemetry(Telemetry* out) override;

  /** Reads REG_STATUS (one byte) and returns it combined with the reply status
   *  byte, translated through STS::units::alarms_from_native(). */
  Result read_alarms(AlarmSet* out) override;

 private:
  Result read_bool(uint16_t addr, bool* out);
  Result read_with_status(uint16_t addr, uint8_t* data, size_t len);
  Result transact(uint8_t id, uint8_t instruction, const uint8_t* params, size_t param_len, uint8_t* out,
                  size_t out_len, bool tolerate_status = false);

  std::unique_ptr<ServoController> m_owned;
  ServoController* m_ctl;
  uint8_t m_id;
  uint8_t m_last_status = 0;
  bool m_readback_check = false;
  HoldStep m_hold_step = HoldStep::NONE;
};

}  // namespace servokit
#endif  // STS_SERVO__H
