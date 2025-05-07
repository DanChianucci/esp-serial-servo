#ifndef ServoController__H
#define ServoController__H

#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <vector>
#include <esp_check.h>

#include "ServoBus.h"
#include "ServoPacket.h"

template<typename T>
class ServoController {
 protected:
  static constexpr const char LOG_TAG[] = "ServoController";
  ServoBus* m_servo_bus;

  int m_transmit_timeout;  // Timeout allowed for transmission
  int m_response_timeout;  // Timeout allowed for reception
  int m_auto_disable_rx;   // 1=disable RX while transmitting, 0=keep RX enabled

 public:
  ServoController(ServoBus* bus, int tx_timeout_ticks, int rx_timeout_ticks, int auto_disable_rx=-1) {
    m_servo_bus = bus;
    m_transmit_timeout = tx_timeout_ticks;
    m_response_timeout = rx_timeout_ticks;
    m_auto_disable_rx  = auto_disable_rx>=0 ? auto_disable_rx : bus->get_tx_pin()==bus->get_rx_pin();
  }

  virtual int send_raw_command(T& cmd) =0;
  virtual int read_raw_response(T& rsp) =0;

  virtual int send_raw_cmd_rsp(T& cmd, T& rsp) {
    ESP_RETURN_ON_ERROR(send_raw_command(cmd),  LOG_TAG, "Failed to send_raw_cmd");
    ESP_RETURN_ON_ERROR(read_raw_response(rsp), LOG_TAG, "Failed to read_raw_response");
    return ESP_OK;
  }

};

#endif  // ServoController__H
