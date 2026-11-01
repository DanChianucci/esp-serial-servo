//------------------------------------------------------------------------
// Filename  :  servo.h
// Project   :  ESP Serial Servo
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

class Servo {
 public:
  virtual ~Servo() = default;

  /** Absolute move in the servo's frame.
   *  @param angle_mdeg Target angle in mdeg.
   *  @return OK, OUT_OF_RANGE outside the limits' angle domain, or a bus error. */
  virtual Result move_to(int32_t angle_mdeg) = 0;

  /** Capabilities of the connected servo. Static per model; may depend on a
   *  prior identify(). Does not touch the bus. */
  virtual Capabilities get_capabilities() const = 0;

  /** One transaction to check the servo responds. No retries. */
  virtual Result probe() { return Result::UNSUPPORTED; }

  /** Reads family, model, firmware version and device ID.
   *  @param[out] out Filled on OK only. */
  virtual Result identify(Identity* out) {
    (void)out;
    return Result::UNSUPPORTED;
  }

  /** Puts the servo in absolute-target mode over the full domain. Writes the
   *  mode registers as given. */
  virtual Result prepare() { return Result::UNSUPPORTED; }

  /** Programs servo-side protection.
   *  @param protection Thresholds; unsupported fields are ignored. */
  virtual Result set_protection(const Protection& protection) {
    (void)protection;
    return Result::UNSUPPORTED;
  }

  /** Sets speed, acceleration and torque for following moves; clamps to the
   *  servo's limits. */
  virtual Result set_profile(const Profile& profile) {
    (void)profile;
    return Result::UNSUPPORTED;
  }

  /** Stops and holds position (no coast). */
  virtual Result hold() { return Result::UNSUPPORTED; }

  /** Enables or disables motor torque. */
  virtual Result set_torque(bool enable) {
    (void)enable;
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
