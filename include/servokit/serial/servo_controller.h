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
inline Result to_result(const srv_stat_t& stat) { return to_result(stat.error()); }
/** A held value (byte count) maps to OK; an error maps as the status code. */
inline Result to_result(const srv_result_t& r) { return r.has_value() ? Result::OK : to_result(r.error()); }

class ServoController {
 public:
  static constexpr timeout_duration_t DEF_TIMEOUT = timeout_duration_t::zero();
  static constexpr char LOG_TAG[] = "ServoController";
  /** Transaction budget used when tx+rx timeouts are not positive. */
  static constexpr timeout_duration_t FALLBACK_TRANSACTION_TIMEOUT = std::chrono::milliseconds(100);

  ServoController(SerialPort* bus, timeout_duration_t tx_timeout, timeout_duration_t rx_timeout,
                  bool auto_disable_rx = true);

  virtual srv_result_t send_command(ServoPacket* cmd, timeout_duration_t timeout = DEF_TIMEOUT);
  virtual srv_result_t send_command(buffer_const_t data, timeout_duration_t timeout = DEF_TIMEOUT);

  virtual srv_stat_t sync_response(const buffer_const_t& sync_pattern, timeout_duration_t timeout = DEF_TIMEOUT);

  virtual srv_result_t read_response(ServoPacket* rsp, timeout_duration_t timeout = DEF_TIMEOUT);
  virtual srv_result_t read_response(const buffer_t& buffer, size_t len, timeout_duration_t timeout = DEF_TIMEOUT);

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
  srv_stat_t set_transaction_timeout(timeout_duration_t timeout);

  /** Response checksum verification (default true). When false, a response whose
   *  checksum is wrong is still accepted if frame (sync/length) and ID validate;
   *  the mismatch is counted. Transmitted frames always carry a correct checksum. */
  void set_verify_checksum(bool enable) { m_verify_checksum = enable; }
  bool get_verify_checksum() const { return m_verify_checksum; }
  /** Number of responses with a bad checksum seen (counted whether or not verification is on). */
  uint32_t get_checksum_errors() const { return m_checksum_errors; }

  SerialPort* m_servo_bus;
  timeout_duration_t m_transmit_timeout;  // Timeout allowed for transmission
  timeout_duration_t m_response_timeout;  // Timeout allowed for reception
  bool m_auto_disable_rx;                 // 1=disable RX while transmitting, 0=keep RX enabled

 protected:
  timeout_duration_t m_transaction_timeout;
  bool m_verify_checksum = true;
  uint32_t m_checksum_errors = 0;

  srv_result_t read_raw_response(buffer_const_t sync_pattern, const buffer_t& buffer, size_t len_pos,
                                 timeout_duration_t timeout);
};
}  // namespace servokit

#endif //SERVO_CONTROLLER__H
