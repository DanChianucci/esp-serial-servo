//------------------------------------------------------------------------
// Filename  :  servo.h
// Project   :  servokit
//
// Description:
//    Family-agnostic Servo interface (tier 1 of the access tiers).
//
//    Contract common to every operation:
//      Units      Angles in mdeg, speed mdeg/s, accel mdeg/s^2, current mA,
//                 temperature C, voltage mV, torque/load in permille.
//      Blocking   Every call is synchronous and blocks the caller until the
//                 bus transaction finishes or times out.
//      Timeout    Each call is bounded by the timeouts configured on the
//                 implementation's controller and returns TIMEOUT rather
//                 than blocking forever. One call is one transaction unless
//                 stated; no hidden retries.
//      Threads    A Servo object is NOT thread-safe, and neither is the bus
//                 it shares. Serialize all calls (including calls
//                 made through get_registers() and get_serial_link()) per
//                 bus, e.g. from a single task or a mutex.
//      Lifetimes  Pointer and reference arguments are used only during the
//                 call and never retained. Returned values are copies.
//                 The Servo does not own the bus; the bus must outlive it.
//      Support    Operations a servo does not provide return UNSUPPORTED;
//                 the library never emulates missing features. Check
//                 get_capabilities().features first.
//------------------------------------------------------------------------

#ifndef SERVO__H
#define SERVO__H

#include <cstddef>
#include <cstdint>

#include "servokit/core/servo_types.h"

namespace servokit {
class RegisterAccess;
class SerialLink;

/** Family-neutral servo. Only move_to() and get_capabilities() are pure
 *  virtual; every other operation defaults to Result::UNSUPPORTED (or the
 *  stated default value) and is overridden by a family that provides it. See
 *  the file header for the contract common to every operation. */
class Servo {
 public:
  /** Virtual: a Servo may be destroyed through a base pointer. Destroying it
   *  sends nothing to the bus. */
  virtual ~Servo() = default;

  /** Absolute move in the servo's frame. Does not wait for the move to
   *  finish: it returns once the target is accepted.
   *  @param angle_mdeg Target angle in mdeg.
   *  @return OK, OUT_OF_RANGE outside the limits' angle domain, or a bus error. */
  virtual Result move_to(int32_t angle_mdeg) = 0;

  /** Capabilities of the connected servo. Static per model; may depend on a
   *  prior identify(). Does not touch the bus. */
  virtual Capabilities get_capabilities() const = 0;

  /** One transaction to check the servo responds. No retries.
   *  @return OK when the servo answers, TIMEOUT if it does not, or another
   *          bus error; UNSUPPORTED by default. */
  virtual Result probe() { return Result::UNSUPPORTED; }

  /** Reads family, model, servo firmware version and device ID.
   *  @param[out] out Filled on OK only; null returns PARAM where supported. */
  virtual Result identify(Identity* out) {
    (void)out;
    return Result::UNSUPPORTED;
  }

  /** Selects the motion mode. Writes the servo's mode setting as given; no
   *  reads. Default: UNSUPPORTED, nothing sent.
   *  @param mode Mode to select.
   *  @return OK, OUT_OF_RANGE for a value that is not a MotionMode (nothing
   *          sent), UNSUPPORTED, or a bus error. */
  virtual Result set_mode(MotionMode mode) {
    (void)mode;
    return Result::UNSUPPORTED;
  }

  // Protection setters. The servo enforces the thresholds itself, even if the
  // host stops. Each setter is gated by its own feature and returns UNSUPPORTED
  // (nothing sent) when that feature is absent, independent of the others.
  // Arguments are validated first: a value the servo cannot hold returns
  // OUT_OF_RANGE with nothing sent and is never clamped. Then plain writes are
  // made, no reads. A setter that writes several registers stops at the first
  // failed write and returns that write's Result; earlier writes of the same
  // call may already have taken effect. Whether the values survive a power
  // cycle follows set_persist(); see PersistScope.

  /** Current protection (Feature::PROTECT_CURRENT).
   *  @param ma Current above which the timer runs, mA.
   *  @param time_ms Time above @p ma before the servo trips, ms. */
  virtual Result set_protect_current(uint32_t ma, uint32_t time_ms) {
    (void)ma;
    (void)time_ms;
    return Result::UNSUPPORTED;
  }

  /** Overload protection (Feature::PROTECT_OVERLOAD).
   *  @param permille Load above which the timer runs, permille of full scale.
   *  @param time_ms Time above @p permille before the servo trips, ms. */
  virtual Result set_protect_overload(uint16_t permille, uint32_t time_ms) {
    (void)permille;
    (void)time_ms;
    return Result::UNSUPPORTED;
  }

  /** Temperature protection (Feature::PROTECT_TEMP).
   *  @param c Trip temperature, degrees C. */
  virtual Result set_protect_temperature(int16_t c) {
    (void)c;
    return Result::UNSUPPORTED;
  }

  /** Supply voltage protection (Feature::PROTECT_VOLTAGE).
   *  @param min_mv Lowest allowed supply, mV.
   *  @param max_mv Highest allowed supply, mV; @p min_mv above @p max_mv is
   *         OUT_OF_RANGE. */
  virtual Result set_protect_voltage(uint32_t min_mv, uint32_t max_mv) {
    (void)min_mv;
    (void)max_mv;
    return Result::UNSUPPORTED;
  }

  /** Selects the alarms that make the servo release torque
   *  (Feature::PROTECT_RELEASE_ON_ALARM). An alarm the servo has no setting
   *  for is OUT_OF_RANGE and nothing is sent. An empty set releases on none.
   *  @param alarms Alarms that release torque. */
  virtual Result set_protect_release_alarms(AlarmSet alarms) {
    (void)alarms;
    return Result::UNSUPPORTED;
  }

  // Motion parameters for following moves. Out-of-range values return
  // OUT_OF_RANGE with nothing sent; nothing is clamped. Callers clamp using
  // get_capabilities().limits. Conversion to the servo's native unit rounds to
  // nearest (halves away from zero); the family documents the exact rule.

  /** Speed for following moves (Feature::SET_SPEED). @param mdeg_s mdeg/s. */
  virtual Result set_speed(uint32_t mdeg_s) {
    (void)mdeg_s;
    return Result::UNSUPPORTED;
  }

  /** Acceleration for following moves (Feature::SET_ACCEL). @param mdeg_s2 mdeg/s^2. */
  virtual Result set_acceleration(uint32_t mdeg_s2) {
    (void)mdeg_s2;
    return Result::UNSUPPORTED;
  }

  /** Torque limit (Feature::TORQUE_LIMIT). @param permille 0..max_torque_permille. */
  virtual Result set_torque_limit(uint16_t permille) {
    (void)permille;
    return Result::UNSUPPORTED;
  }

  /** Minimum starting torque (Feature::START_TORQUE). @param permille 0..max_torque_permille. */
  virtual Result set_start_torque(uint16_t permille) {
    (void)permille;
    return Result::UNSUPPORTED;
  }

  /** Cancels any move in progress (Feature::CANCEL_MOTION).
   *  On OK the servo is stationary, its target equals its position and torque
   *  is DISABLED. Torque is not required to be on beforehand. The servo is
   *  unpowered between the torque-off and the end of the sequence and nothing
   *  restores torque: the caller enables it again with set_torque_enabled().
   *  If a step after the torque-off fails, torque stays disabled and the
   *  failure is reported through the returned Result (the implementation
   *  documents how to learn which step failed). If the torque-off itself
   *  fails, the torque state is unknown. Several transactions, no retries. */
  virtual Result cancel_motion() { return Result::UNSUPPORTED; }

  /** Enables or disables motor torque (Feature::TORQUE_ENABLE).
   *  @param enable true: motor driven; false: motor released. */
  virtual Result set_torque_enabled(bool enable) {
    (void)enable;
    return Result::UNSUPPORTED;
  }

  /** Reads whether motor torque is enabled (Feature::TORQUE_ENABLE).
   *  @param[out] out Filled on OK only; null returns PARAM. */
  virtual Result read_torque_enabled(bool* out) {
    (void)out;
    return Result::UNSUPPORTED;
  }

  /** Selects whether configuration writes survive power loss.
   *  true: configuration writes made afterwards are kept across a power cycle.
   *  false: they take effect but are not kept across a power cycle.
   *  Applies to every configuration write until changed again; the library
   *  holds no state for it. Default: UNSUPPORTED (see Feature::PERSIST_CONFIG). */
  virtual Result set_persist(bool persist) {
    (void)persist;
    return Result::UNSUPPORTED;
  }

  /** Reads one atomic telemetry sample.
   *  @param[out] out Filled on OK only. */
  virtual Result read_telemetry(Telemetry* out) {
    (void)out;
    return Result::UNSUPPORTED;
  }

  /** Reads the active alarm set.
   *  @param[out] out Filled on OK only. */
  virtual Result read_alarms(AlarmSet* out) {
    (void)out;
    return Result::UNSUPPORTED;
  }

  /** Clears latched alarms where the servo supports it. */
  virtual Result clear_alarms() { return Result::UNSUPPORTED; }

  /** Register-level access to this servo's control table.
   *  @return Non-owning pointer valid for the lifetime of this Servo, or
   *          nullptr when the servo does not set Feature::REGISTER_ACCESS.
   *          Non-null exactly when that feature is set. */
  virtual RegisterAccess* get_registers() { return nullptr; }

  /** Link-level settings of a serial-bus servo (checksum verification).
   *  @return Non-owning pointer valid for the lifetime of this Servo, or
   *          nullptr when the servo is not on a checksummed serial link. */
  virtual SerialLink* get_serial_link() { return nullptr; }
};
}  // namespace servokit

#endif  // SERVO__H
