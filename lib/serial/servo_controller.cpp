
#include "servokit/serial/servo_controller.h"
#include "servokit/core/servo_utils.h"
#include "servokit/serial/servo_internal.h"

namespace servokit {
using namespace timeout;

ServoController::ServoController(SerialPort* bus, timeout_duration_t tx_timeout, timeout_duration_t rx_timeout,
                                 bool auto_disable_rx) {
  m_servo_bus = bus;
  m_transmit_timeout = tx_timeout;
  m_response_timeout = rx_timeout;
  m_auto_disable_rx = auto_disable_rx;
  m_transaction_timeout = (tx_timeout + rx_timeout > timeout_duration_t::zero()) ? tx_timeout + rx_timeout
                                                                                 : FALLBACK_TRANSACTION_TIMEOUT;
}

Result to_result(int code) {
  if (code == 0) return Result::OK;
  if (code == SRV_TIMEOUT.error()) return Result::TIMEOUT;
  if (code == SRV_READ_FAIL.error() || code == SRV_WRITE_FAIL.error()) return Result::BUS;
  if (code == SRV_BAD_FRAME.error() || code == SRV_BAD_CHECKSUM.error() || code == SRV_BAD_ID.error())
    return Result::BAD_RESPONSE;
  if (code == SRV_PARAM_ERR.error() || code == SRV_OVERFLOW.error()) return Result::PARAM;
  return Result::BUS;  // SRV_FAIL and unknown codes
}

srv_stat_t ServoController::flush_rx() { return m_servo_bus->discard_input(); }

srv_stat_t ServoController::set_transaction_timeout(timeout_duration_t timeout) {
  SRV_RETURN_IF(timeout <= timeout_duration_t::zero(), SRV_PARAM_ERR);
  m_transaction_timeout = timeout;
  return SRV_OK;
}

srv_result_t ServoController::send_command(const buffer_const_t data, timeout_duration_t timeout) {
  srv_result_t result = 0;
  if (timeout == DEF_TIMEOUT) timeout = m_transmit_timeout;

  // Drop stale bytes so they cannot be mistaken for (part of) this command's response.
  SRV_RETURN_ON_ERR(flush_rx());

  if (m_auto_disable_rx) {
    SRV_RETURN_ON_ERR(m_servo_bus->enable_rx(false));
    SRV_RETURN_ON_ERR(result = m_servo_bus->write_bytes(data, timeout));
    SRV_RETURN_ON_ERR(m_servo_bus->enable_rx(true));
  } else {
    SRV_RETURN_ON_ERR(result = m_servo_bus->write_bytes(data, timeout));
  }
  return result;
};

srv_result_t ServoController::send_command(ServoPacket* command, timeout_duration_t timeout) {
  return send_command(command->as_bytes(), timeout);
}

srv_stat_t ServoController::sync_response(const buffer_const_t& sync_pattern, timeout_duration_t timeout) {
  if (timeout == DEF_TIMEOUT) timeout = m_response_timeout;
  SRV_RETURN_ON_ERR(this->m_servo_bus->read_sync(sync_pattern, timeout));
  return SRV_OK;
}

srv_result_t ServoController::read_response(const buffer_t& buffer, size_t readlen, timeout_duration_t timeout) {
  if (timeout == DEF_TIMEOUT) timeout = m_response_timeout;
  SRV_RETURN_IF(readlen > buffer.size(), SRV_OVERFLOW);
  return this->m_servo_bus->read_bytes(buffer.first(readlen), timeout);
}

srv_result_t ServoController::read_response(ServoPacket* response, timeout_duration_t timeout) {
  srv_result_t result = 0;
  if (timeout == DEF_TIMEOUT) timeout = m_response_timeout;
  SRV_RETURN_ON_ERR(response->reset());
  SRV_RETURN_ON_ERR(
      result = read_raw_response(response->sync_buffer(), response->data_buffer(), response->len_index(), timeout));
  SRV_RETURN_ON_ERR(response->resize(get_value(result)));
  SRV_RETURN_IF_NOT(response->is_frame_valid(), SRV_BAD_FRAME);
  if (!response->is_checksum_valid()) {
    ++m_checksum_errors;
    SRV_RETURN_IF(m_verify_checksum, SRV_BAD_CHECKSUM);
  }
  return result;
}

srv_result_t ServoController::read_raw_response(const buffer_const_t sync_buffer, const buffer_t& data_buffer,
                                                size_t len_pos, timeout_duration_t timeout) {
  using clock = std::chrono::steady_clock;
  int readlen = 0;
  int pktlen = 0;

  // Own deadline: every step gets only the time left, so the whole response is bounded by `timeout`.
  const clock::time_point deadline = clock::now() + timeout;
  auto remaining = [&]() { return deadline - clock::now(); };

  // 1. Read until sync pattern is found
  SRV_RETURN_IF(remaining() <= timeout_duration_t::zero(), SRV_TIMEOUT);
  SRV_RETURN_ON_ERR(sync_response(sync_buffer, remaining()));
  // 2. Read dev id, length
  readlen = (len_pos + 1);
  SRV_RETURN_IF(remaining() <= timeout_duration_t::zero(), SRV_TIMEOUT);
  srv_result_t got = read_response(data_buffer.subspan(pktlen), readlen, remaining());
  SRV_RETURN_ON_ERR(got);
  SRV_RETURN_IF(get_value(got) < readlen, SRV_TIMEOUT);  // short read: the rest never arrived
  pktlen += readlen;
  // 3. Read until checksum is found
  readlen = data_buffer[len_pos];
  SRV_RETURN_IF((size_t)readlen > data_buffer.size() - pktlen, SRV_BAD_FRAME);  // length byte exceeds buffer
  if (readlen > 0) {
    SRV_RETURN_IF(remaining() <= timeout_duration_t::zero(), SRV_TIMEOUT);
    got = read_response(data_buffer.subspan(pktlen), readlen, remaining());
    SRV_RETURN_ON_ERR(got);
    SRV_RETURN_IF(get_value(got) < readlen, SRV_TIMEOUT);
  }
  pktlen += readlen;

  return (int)(pktlen + sync_buffer.size());
}

srv_result_t ServoController::command_response(ServoPacket* cmd, ServoPacket* rsp, timeout_duration_t cmd_timeout,
                                               timeout_duration_t rsp_timeout) {
  using clock = std::chrono::steady_clock;
  const timeout_duration_t budget = m_transaction_timeout;
  const clock::time_point start = clock::now();

  if (cmd_timeout > budget) cmd_timeout = budget;
  SRV_RETURN_ON_ERR(send_command(cmd, cmd_timeout));

  const timeout_duration_t remaining = budget - (clock::now() - start);
  SRV_RETURN_IF(remaining <= timeout_duration_t::zero(), SRV_TIMEOUT);
  if (rsp_timeout == DEF_TIMEOUT) rsp_timeout = m_response_timeout;
  if (rsp_timeout <= timeout_duration_t::zero() || rsp_timeout > remaining) rsp_timeout = remaining;

  srv_result_t result = read_response(rsp, rsp_timeout);
  SRV_RETURN_ON_ERR(result);
  const int cmd_id = cmd->packet_device_id();
  const int rsp_id = rsp->packet_device_id();
  SRV_RETURN_IF(cmd_id >= 0 && rsp_id >= 0 && cmd_id != rsp_id, SRV_BAD_ID);
  return result;
}
}  // namespace servokit
