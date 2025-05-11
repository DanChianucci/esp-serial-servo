#ifndef ServoController__H
#define ServoController__H

#include <cstdint>
#include <span>

#include "ServoBus.h"
#include "ServoPacket.h"
#include "ServoUtils.h"

class ServoController {
 public:
  static constexpr char LOG_TAG[] = "ServoController";

  ServoController(ServoBus* bus, timeout_duration_t tx_timeout, timeout_duration_t rx_timeout,
                  int auto_disable_rx = -1);

  virtual int send_command(ServoPacket* cmd);
  virtual int send_command(buffer_const_t data);

  virtual int sync_response(const buffer_const_t& sync_pattern,
                            timeout_duration_t timeout = timeout_duration_t::zero());

  virtual int read_response(ServoPacket* rsp);
  virtual int read_response(const buffer_t& buffer, size_t len,
                            timeout_duration_t timeout = timeout_duration_t::zero());

 protected:
  ServoBus* m_servo_bus;
  timeout_duration_t m_transmit_timeout;  // Timeout allowed for transmission
  timeout_duration_t m_response_timeout;  // Timeout allowed for reception
  bool m_auto_disable_rx;                 // 1=disable RX while transmitting, 0=keep RX enabled
  int read_raw_response(buffer_const_t sync_pattern, const buffer_t& buffer, size_t len_pos = 3);
};

#endif  // ServoController__H
