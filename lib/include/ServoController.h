#ifndef ServoController__H
#define ServoController__H

#include <cstdint>

#include "ServoBus.h"
#include "ServoPacket.h"

class ServoController {
 protected:
 public:
  ServoController(ServoBus* bus, uint32_t tx_timeout_ticks,
                  uint32_t rx_timeout_ticks, int auto_disable_rx = -1);
  virtual int send_command(ServoPacket* cmd);
  virtual int read_response(ServoPacket* rsp);

 protected:
  static constexpr const char LOG_TAG[] = "ServoController";
  ServoBus* m_servo_bus;

  uint32_t m_transmit_timeout;  // Timeout allowed for transmission
  uint32_t m_response_timeout;  // Timeout allowed for reception
  bool m_auto_disable_rx;  // 1=disable RX while transmitting, 0=keep RX enabled
  int send_raw_command(const std::span<const uint8_t> data);
  int read_raw_response(const std::span<const uint8_t> sync_pattern,
                        const std::span<uint8_t>& buffer, size_t len_pos = 3);
};

#endif  // ServoController__H
