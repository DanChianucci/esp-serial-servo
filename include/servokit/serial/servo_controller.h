#ifndef SERVO_CONTROLLER__H
#define SERVO_CONTROLLER__H


#include <cstdint>
#include <span>

#include "servokit/serial/serial_port.h"
#include "servokit/serial/servo_packet.h"
#include "servokit/core/servo_utils.h"

namespace servokit {
class ServoController {
 public:
  static constexpr timeout_duration_t DEF_TIMEOUT = timeout_duration_t::zero();
  static constexpr char LOG_TAG[] = "ServoController";

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

  SerialPort* m_servo_bus;
  timeout_duration_t m_transmit_timeout;  // Timeout allowed for transmission
  timeout_duration_t m_response_timeout;  // Timeout allowed for reception
  bool m_auto_disable_rx;                 // 1=disable RX while transmitting, 0=keep RX enabled

 protected:
  srv_result_t read_raw_response(buffer_const_t sync_pattern, const buffer_t& buffer, size_t len_pos,
                                 timeout_duration_t timeout);
};
}  // namespace servokit

#endif //SERVO_CONTROLLER__H
