//------------------------------------------------------------------------
// Filename  :  serial_link.h
// Project   :  servokit
//
// Description:
//    Link-level settings of servos that talk over a serial bus with
//    checksummed frames. Reached through Servo::get_serial_link(), which
//    returns nullptr for servos without such a link (for example PWM servos).
//    Lives in serial/ because it describes the serial transport; core/ only
//    forward-declares it.
//
//    Contract common to every call:
//      Blocking   None. No call touches the bus.
//      Threads    Not thread-safe; serialize with all other calls on the bus.
//      Lifetimes  The object is owned by the Servo that returned it.
//      Scope      A setting applies to the whole bus controller behind the
//                 servo, so servos sharing one controller share it.
//------------------------------------------------------------------------

#ifndef SERIAL_LINK__H
#define SERIAL_LINK__H

#include <cstdint>

#include "servokit/core/servo_types.h"

namespace servokit {

/** Link-level settings of a servo on a checksummed serial bus. Implemented
 *  by servo classes that sit on a ServoController (for example STSServo).
 *  See the file header for the contract common to every call. */
class SerialLink {
 public:
  virtual ~SerialLink() = default;

  /** Enables or disables response checksum verification (default on). When off,
   *  a response with a wrong checksum is accepted if its framing and ID are
   *  valid, and the mismatch is still counted. Transmitted frames always carry
   *  a correct checksum.
   *  @return OK, or PARAM if the servo has no bus controller. */
  virtual Result set_verify_checksum(bool enable) = 0;

  /** Current checksum verification setting; true when there is no controller. */
  virtual bool get_verify_checksum() const = 0;

  /** Number of responses with a bad checksum seen, counted whether or not
   *  verification is on; 0 when there is no controller. Wraps at 2^32. */
  virtual uint32_t get_checksum_errors() const = 0;
};

}  // namespace servokit

#endif  // SERIAL_LINK__H
