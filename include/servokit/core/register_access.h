//------------------------------------------------------------------------
// Filename  :  register_access.h
// Project   :  ESP Serial Servo
//
// Description:
//    Register-level access to the servo's own control table (tier 3).
//    Reached through Servo::get_registers(), which returns a non-null pointer
//    exactly when the servo sets Feature::REGISTER_ACCESS.
//
//    Contract common to every call:
//      Target     The servo this interface belongs to; the device ID is held by
//                 the implementation. Addresses and meanings are defined by
//                 the servo family; no unit conversion is done.
//      Byte order Multi-byte values are little endian: the low byte is at the
//                 lower address. This holds for read_u16() and write_u16().
//      Blocking   Synchronous. Each call is one bus transaction and blocks
//                 until it finishes or times out; no hidden retries.
//      Timeout    Bounded by the timeouts of the implementation's bus; a
//                 silent bus returns TIMEOUT.
//      Threads    Not thread-safe, and neither is the bus. Serialize all calls
//                 per bus.
//      Lifetimes  Pointer arguments are used only during the call and never
//                 retained. Output buffers are written on OK only.
//      Errors     Returned as Result, never thrown. PARAM for a null pointer,
//                 empty span or invalid target; OUT_OF_RANGE for a span
//                 outside the control table; bus errors as for any operation.
//------------------------------------------------------------------------

#ifndef REGISTER_ACCESS__H
#define REGISTER_ACCESS__H

#include <cstddef>
#include <cstdint>

#include "servokit/core/servo_types.h"

namespace servokit {

class RegisterAccess {
 public:
  virtual ~RegisterAccess() = default;

  /** Reads @p len bytes starting at register @p addr.
   *  @param addr First register address.
   *  @param[out] data Caller buffer of at least @p len bytes; written on OK only.
   *  @param len Number of bytes to read; the maximum is family-defined. */
  virtual Result read_register(uint16_t addr, uint8_t* data, size_t len) = 0;

  /** Writes @p len bytes starting at register @p addr, as given. With the
   *  readback check on, a successful write is followed by one read of the
   *  written bytes; a difference returns Result::VERIFY_FAILED.
   *  @param addr First register address.
   *  @param data Bytes to write; read during the call only.
   *  @param len Number of bytes to write; the maximum is family-defined. */
  virtual Result write_register(uint16_t addr, const uint8_t* data, size_t len) = 0;

  /** Reads one byte register. @param[out] out Written on OK only. */
  virtual Result read_u8(uint16_t addr, uint8_t* out) = 0;

  /** Reads a 16-bit register, little endian, @p addr = low byte.
   *  @param[out] out Written on OK only. */
  virtual Result read_u16(uint16_t addr, uint16_t* out) = 0;

  /** Writes one byte register. */
  virtual Result write_u8(uint16_t addr, uint8_t value) = 0;

  /** Writes a 16-bit register, little endian, @p addr = low byte. */
  virtual Result write_u16(uint16_t addr, uint16_t value) = 0;

  /** Turns the readback check on or off (default off). When on, each register
   *  write is followed by one read of the written bytes, compared with what
   *  was written; a difference returns Result::VERIFY_FAILED. Registers whose
   *  value the servo may change on its own are not compared. Touches no bus.
   *  @return OK, or UNSUPPORTED if the implementation has no readback check. */
  virtual Result set_readback_check(bool enable) = 0;

  /** Current readback-check setting. Does not touch the bus. */
  virtual bool get_readback_check() const = 0;
};

}  // namespace servokit

#endif  // REGISTER_ACCESS__H
