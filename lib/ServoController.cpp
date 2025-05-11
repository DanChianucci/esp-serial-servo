
#include "ServoController.h"

#include "ServoUtils.h"

ServoController::ServoController(ServoBus* bus, timeout_duration_t tx_timeout, timeout_duration_t rx_timeout,
                                 int auto_disable_rx) {
  m_servo_bus = bus;
  m_transmit_timeout = tx_timeout;
  m_response_timeout = rx_timeout;
  m_auto_disable_rx = auto_disable_rx >= 0 ? (auto_disable_rx != 0) : (bus->get_tx_pin() == bus->get_rx_pin());
}

int ServoController::send_command(const buffer_const_t data) {
  if (m_auto_disable_rx) m_servo_bus->enable_rx(false);
  int ret = m_servo_bus->write_bytes(data, m_transmit_timeout);
  if (m_auto_disable_rx) m_servo_bus->enable_rx(true);
  m_servo_bus->discard_input();
  return ret;
};

int ServoController::send_command(ServoPacket* command) { return send_command(command->as_bytes()); }


int ServoController::read_response(const buffer_t& buffer, size_t readlen, timeout_duration_t timeout) {
  if (readlen > buffer.size()) return SRV_OVERFLOW;
  if (timeout == timeout_duration_t::zero()) timeout = m_response_timeout;
  return this->m_servo_bus->read_bytes(buffer.first(readlen), timeout);
}


int ServoController::read_response(ServoPacket* response) {
  response->reset();
  int size = read_raw_response(response->sync_buffer(), response->data_buffer(), response->len_index());
  SRV_RETURN_ON_ERR(size);

  response->resize(size);
  return size;
}

int ServoController::sync_response(const buffer_const_t& sync_pattern, timeout_duration_t timeout) {
  if (timeout == timeout_duration_t::zero()) timeout = m_response_timeout;
  SRV_RETURN_ON_ERR(this->m_servo_bus->read_sync(sync_pattern, timeout));
  return SRV_OK;
}

int ServoController::read_raw_response(const buffer_const_t sync_buffer, const buffer_t& data_buffer, size_t len_pos) {
  size_t readlen = 0;
  size_t pktlen = 0;
  timeout_duration_t remaining = m_response_timeout;

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

  return pktlen + sync_buffer.size();
}
