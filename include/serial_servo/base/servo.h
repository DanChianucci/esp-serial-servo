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
//                 it shares. Serialize all calls (including checksum
//                 accessors) per bus, e.g. from a single task or a mutex.
//      Lifetimes  Pointer and reference arguments are used only during the
//                 call and never retained. Returned values are copies.
//                 The Servo does not own the bus; the bus must outlive it.
//      Support    Operations a servo does not provide return UNSUPPORTED;
//                 the library never emulates missing features. Check
//                 capabilities().features first.
//------------------------------------------------------------------------

#ifndef SERVO__H
#define SERVO__H

#include <cstddef>
#include <cstdint>

#include "serial_servo/base/servo_types.h"

namespace serial_servo {
class Servo {
 public:
  virtual ~Servo() = default;

  /** Absolute move in the servo's frame.
   *  @param angle_mdeg Target angle in mdeg.
   *  @return OK, OUT_OF_RANGE outside the limits' angle domain, or a bus error. */
  virtual Result move_to(int32_t angle_mdeg) = 0;

  /** Capabilities of the connected servo. Static per model; may depend on a
   *  prior read_identity(). Does not touch the bus. */
  virtual Capabilities capabilities() const = 0;

  /** One transaction to check the servo responds. No retries. */
  virtual Result ping() { return Result::UNSUPPORTED; }

  /** Reads family, model, firmware version and device ID.
   *  @param[out] out Filled on OK only. */
  virtual Result read_identity(Identity* out) {
    (void)out;
    return Result::UNSUPPORTED;
  }

  /** Puts the servo in absolute-target mode over the full domain. Reads first
   *  and writes only on difference (EEPROM wear). */
  virtual Result prepare() { return Result::UNSUPPORTED; }

  /** Programs servo-side protection (read-compare-write).
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

  /** Raw register read (tier 3). Address and meaning are family-defined; no
   *  unit conversion.
   *  @param addr Register address.
   *  @param[out] data Caller buffer of at least @p len bytes.
   *  @param len Bytes to read. */
  virtual Result read_raw(uint16_t addr, uint8_t* data, size_t len) {
    (void)addr;
    (void)data;
    (void)len;
    return Result::UNSUPPORTED;
  }

  /** Raw register write (tier 3). See read_raw().
   *  @param data Bytes to write, read during the call only. */
  virtual Result write_raw(uint16_t addr, const uint8_t* data, size_t len) {
    (void)addr;
    (void)data;
    (void)len;
    return Result::UNSUPPORTED;
  }

  /** Enables/disables response checksum verification (default on in
   *  implementations). Returns UNSUPPORTED if the protocol has no checksum. */
  virtual Result set_verify_checksum(bool enable) {
    (void)enable;
    return Result::UNSUPPORTED;
  }

  /** Current checksum verification setting; true if unsupported/unknown. */
  virtual bool verify_checksum() const { return true; }

  /** Count of responses with a bad checksum seen; 0 if unsupported. */
  virtual uint32_t checksum_errors() const { return 0; }
};
}  // namespace serial_servo

#endif  // SERVO__H
