//------------------------------------------------------------------------
// Filename  :  sts_servo.h
// Project   :  ESP Serial Servo
//
// Description:
//    STS-series servo, register-level access (tier 3 of the access tiers).
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

class STSServo : public Servo {
 public:
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
  Result transact(uint8_t id, uint8_t instruction, const uint8_t* params, size_t param_len, uint8_t* out,
                  size_t out_len);

  std::unique_ptr<ServoController> m_owned;
  ServoController* m_ctl;
  uint8_t m_id;
  uint8_t m_last_status = 0;
};

}  // namespace servokit
#endif  // STS_SERVO__H
