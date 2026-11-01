//------------------------------------------------------------------------
// Filename  :  servo_controller.h
// Project   :  servokit
//
// Description:
//    ServoController: runs one command/response transaction over a
//    SerialPort within a bounded time, and to_result(), the mapping from
//    transport status codes to the generic Result.
//
//    Contract common to every call:
//      Blocking   Synchronous. Each call blocks until it finishes or its
//                 time runs out; nothing is retried.
//      Timeouts   A timeout argument equal to DEF_TIMEOUT (zero) selects the
//                 configured value, so a zero timeout cannot be requested
//                 explicitly. Other values are used as given.
//      Threads    Not thread-safe, and neither is the SerialPort under it.
//                 Serialize all calls per bus.
//      Lifetimes  Pointer and reference arguments are used only during the
//                 call and never retained.
//      Errors     Returned as srv_stat_t / srv_result_t status codes
//                 (servo_utils.h), never thrown.
//------------------------------------------------------------------------

#ifndef SERVO_CONTROLLER__H
#define SERVO_CONTROLLER__H


#include <chrono>
#include <cstdint>
#include <span>

#include "servokit/serial/serial_port.h"
#include "servokit/serial/servo_packet.h"
#include "servokit/core/servo_types.h"
#include "servokit/core/servo_utils.h"

namespace servokit {

/** Maps an srv_* status code (0 = OK, negative = error) to the generic Result.
 *  SRV_OK -> OK; SRV_TIMEOUT -> TIMEOUT; SRV_READ_FAIL, SRV_WRITE_FAIL -> BUS;
 *  SRV_BAD_FRAME, SRV_BAD_CHECKSUM, SRV_BAD_ID -> BAD_RESPONSE;
 *  SRV_PARAM_ERR, SRV_OVERFLOW (caller buffer too small) -> PARAM;
 *  SRV_FAIL and any unknown code -> BUS (generic transport/driver failure).
 *  There is no srv_* source for UNSUPPORTED or OUT_OF_RANGE. */
Result to_result(int srv_code);
/** Maps a status (error) object as the integer overload does. */
inline Result to_result(const srv_stat_t& stat) { return to_result(stat.error()); }
/** A held value (byte count) maps to OK; an error maps as the status code. */
inline Result to_result(const srv_result_t& r) { return r.has_value() ? Result::OK : to_result(r.error()); }

/** Sends commands and reads responses over a SerialPort, with every step
 *  bounded by a time budget. Borrows the port, which must outlive the
 *  controller. Not thread-safe (see the file header). */
class ServoController {
 public:
  /** Timeout argument value meaning "use the configured timeout" (zero). */
  static constexpr timeout_duration_t DEF_TIMEOUT = timeout_duration_t::zero();
  /** Tag for log output. */
  static constexpr char LOG_TAG[] = "ServoController";
  /** Transaction budget used when tx+rx timeouts are not positive. */
  static constexpr timeout_duration_t FALLBACK_TRANSACTION_TIMEOUT = std::chrono::milliseconds(100);

  /** Configures the controller; sends nothing.
   *  @param bus Port to use; borrowed, must outlive the controller.
   *  @param tx_timeout Default time allowed for sending one command.
   *  @param rx_timeout Default time allowed for receiving one response.
   *  @param auto_disable_rx True: the receiver is disabled while a command is
   *         transmitted (single-wire half-duplex buses, where the port would
   *         otherwise hear its own frame) and enabled again afterwards, even
   *         if the write failed. False: the receiver stays enabled.
   *  The transaction budget (get_transaction_timeout()) starts as
   *  tx_timeout + rx_timeout, saturating, or FALLBACK_TRANSACTION_TIMEOUT if
   *  that sum is not positive. */
  ServoController(SerialPort* bus, timeout_duration_t tx_timeout, timeout_duration_t rx_timeout,
                  bool auto_disable_rx = true);

  /** Discards buffered input, then writes the packet's bytes.
   *  @param cmd Packet to send; its bytes are read during the call.
   *  @param timeout Time allowed for transmission; DEF_TIMEOUT uses the
   *         configured transmit timeout.
   *  @return The number of bytes written.
   *  @return A status code from the port (for example SRV_WRITE_FAIL,
   *          SRV_TIMEOUT) or from re-enabling the receiver. */
  virtual srv_result_t send_command(ServoPacket* cmd, timeout_duration_t timeout = DEF_TIMEOUT);
  /** As the packet overload, for raw bytes. */
  virtual srv_result_t send_command(buffer_const_t data, timeout_duration_t timeout = DEF_TIMEOUT);

  /** Consumes received bytes up to and including @p sync_pattern.
   *  @param timeout Time allowed; DEF_TIMEOUT uses the configured response
   *         timeout.
   *  @return SRV_OK, SRV_TIMEOUT if the pattern did not arrive, or the port's error. */
  virtual srv_stat_t sync_response(const buffer_const_t& sync_pattern, timeout_duration_t timeout = DEF_TIMEOUT);

  /** Reads one whole frame into @p rsp: the sync pattern, then the bytes up
   *  to the length field, then as many bytes as the length field announces.
   *  The packet is reset first and resized to the frame afterwards. The
   *  frame is then validated by the packet itself.
   *  @param timeout Total time for the whole frame (one deadline); DEF_TIMEOUT
   *         uses the configured response timeout.
   *  @return The frame size in bytes including the sync pattern.
   *  @return SRV_TIMEOUT for a missing or truncated frame.
   *  @return SRV_BAD_FRAME for a length that does not fit the packet buffer
   *          or a frame that fails the packet's frame check.
   *  @return SRV_BAD_CHECKSUM for a bad checksum while verification is on.
   *  @return The port's error otherwise. */
  virtual srv_result_t read_response(ServoPacket* rsp, timeout_duration_t timeout = DEF_TIMEOUT);
  /** Raw read of @p len bytes into the start of @p buffer.
   *  @param timeout Time allowed; DEF_TIMEOUT uses the configured response
   *         timeout.
   *  @return The number of bytes read (may be fewer than @p len; not an error).
   *  @return SRV_OVERFLOW if @p len exceeds the buffer size.
   *  @return The port's error otherwise. */
  virtual srv_result_t read_response(const buffer_t& buffer, size_t len, timeout_duration_t timeout = DEF_TIMEOUT);

  /** One transaction: sends @p cmd and reads the response into @p rsp.
   *  The whole call is bounded by get_transaction_timeout(): the send step is
   *  clamped to it and the response step gets only the time left, so a slow
   *  send shortens the receive window.
   *  @param cmd_timeout Time for the send step; DEF_TIMEOUT uses the
   *         configured transmit timeout.
   *  @param rsp_timeout Time for the response step; DEF_TIMEOUT or any value
   *         that is not positive or exceeds the time left uses the time left.
   *  @return As read_response(), plus SRV_BAD_ID when the response carries a
   *          different device ID than the command (both IDs known), and
   *          SRV_TIMEOUT when the send step used up the budget. */
  virtual srv_result_t command_response(ServoPacket* cmd, ServoPacket* rsp,
                                        timeout_duration_t cmd_timeout = DEF_TIMEOUT,
                                        timeout_duration_t rsp_timeout = DEF_TIMEOUT);

  /** Discards any bytes buffered on the receive side. Called automatically before
   *  every command; safe to call manually. */
  srv_stat_t flush_rx();

  /** Total time (send + response) one command_response() may take. Always finite
   *  and positive; defaults to tx_timeout + rx_timeout from the constructor.
   *  Zero or negative values are rejected with SRV_PARAM_ERR. */
  timeout_duration_t get_transaction_timeout() const { return m_transaction_timeout; }
  /** Sets the transaction budget.
   *  @return SRV_OK, or SRV_PARAM_ERR if @p timeout is not positive (unchanged). */
  srv_stat_t set_transaction_timeout(timeout_duration_t timeout);

  /** Response checksum verification (default true). When false, a response whose
   *  checksum is wrong is still accepted if frame (sync/length) and ID validate;
   *  the mismatch is counted. Transmitted frames always carry a correct checksum. */
  void set_verify_checksum(bool enable) { m_verify_checksum = enable; }
  /** @return The checksum verification setting. */
  bool get_verify_checksum() const { return m_verify_checksum; }
  /** Number of responses with a bad checksum seen (counted whether or not verification is on).
   *  Wraps at 2^32. */
  uint32_t get_checksum_errors() const { return m_checksum_errors; }

  /** The borrowed port. Public configuration: may be changed between calls,
   *  never during one. */
  SerialPort* m_servo_bus;
  timeout_duration_t m_transmit_timeout;  ///< Default time allowed for transmission.
  timeout_duration_t m_response_timeout;  ///< Default time allowed for reception.
  bool m_auto_disable_rx;                 ///< True: disable RX while transmitting; false: keep RX enabled.

 protected:
  timeout_duration_t m_transaction_timeout;  ///< Budget of one command_response(); always positive.
  bool m_verify_checksum = true;             ///< Reject responses with a bad checksum.
  uint32_t m_checksum_errors = 0;            ///< Responses with a bad checksum seen so far.

  /** Reads sync pattern, then the bytes through the length field, then the
   *  announced remainder, into @p buffer, all within one deadline.
   *  @param sync_pattern Frame start marker to wait for (not stored in @p buffer).
   *  @param buffer Receives everything after the sync pattern.
   *  @param len_pos Index in @p buffer of the length byte.
   *  @param timeout Total time for all three steps.
   *  @return Total frame size including the sync pattern, SRV_TIMEOUT for a
   *          short read, SRV_BAD_FRAME if the announced length does not fit. */
  srv_result_t read_raw_response(buffer_const_t sync_pattern, const buffer_t& buffer, size_t len_pos,
                                 timeout_duration_t timeout);
};
}  // namespace servokit

#endif //SERVO_CONTROLLER__H
