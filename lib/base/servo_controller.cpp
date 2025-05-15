
#include "serial_servo/base/servo_controller.h"
#include "serial_servo/base/servo_utils.h"
#include "serial_servo/base/servo_internal.h"

namespace esp_serial_servo {
using namespace timeout;

ServoController::ServoController(ServoBus* bus, timeout_duration_t tx_timeout, timeout_duration_t rx_timeout,
                                 bool auto_disable_rx) {
  m_servo_bus = bus;
  m_transmit_timeout = tx_timeout;
  m_response_timeout = rx_timeout;
  m_auto_disable_rx = auto_disable_rx;
}

srv_result_t ServoController::send_command(const buffer_const_t data, timeout_duration_t timeout) {
  srv_result_t result = 0;
  if (timeout == DEF_TIMEOUT) timeout = m_transmit_timeout;

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
  return result;
}

srv_result_t ServoController::read_raw_response(const buffer_const_t sync_buffer, const buffer_t& data_buffer,
                                                size_t len_pos, timeout_duration_t timeout) {
  int readlen = 0;
  int pktlen = 0;

  timeout_duration_t remaining = timeout;
  timeout_ctrl_t timeout_ctrl;
  init_timeout_ctrl(&timeout_ctrl);

  // 1. Read until sync pattern is found
  SRV_RETURN_ON_ERR(sync_response(sync_buffer, remaining));
  SRV_RETURN_IF(update_timeout(&timeout_ctrl, &remaining), SRV_TIMEOUT);
  // 2. Read dev id, length
  readlen = (len_pos + 1);
  SRV_RETURN_ON_ERR(read_response(data_buffer.subspan(pktlen), readlen, remaining));
  SRV_RETURN_IF(update_timeout(&timeout_ctrl, &remaining), SRV_TIMEOUT);
  pktlen += readlen;
  // 3. Read until checksum is found
  readlen = data_buffer[len_pos];
  SRV_RETURN_ON_ERR(read_response(data_buffer.subspan(pktlen), readlen, remaining));
  SRV_RETURN_IF(update_timeout(&timeout_ctrl, &remaining), SRV_TIMEOUT);
  pktlen += readlen;

  return (int)(pktlen + sync_buffer.size());
}

srv_result_t ServoController::command_response(ServoPacket* cmd, ServoPacket* rsp, timeout_duration_t cmd_timeout,
                                               timeout_duration_t rsp_timeout) {
  SRV_RETURN_ON_ERR(send_command(cmd, cmd_timeout));
  return read_response(rsp, rsp_timeout);
}
}  // namespace esp_serial_servo