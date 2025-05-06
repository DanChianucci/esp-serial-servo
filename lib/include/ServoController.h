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
 private:
  static constexpr const char* LOG_TAG = "ServoController";
 protected:
  ServoBus* m_servo_bus;

  int m_transmit_timeout;  // Timeout allowed for transmission
  int m_response_timeout;  // Timeout allowed for reception
  int m_auto_disable_tx;   // 1=disable RX while transmitting, 0=keep RX enabled

 public:
  ServoController(ServoBus* bus, int tx_timeout_ms, int rx_timeout_ms) {
    m_servo_bus = bus;
    m_transmit_timeout = pdMS_TO_TICKS(tx_timeout_ms);
    m_response_timeout = pdMS_TO_TICKS(rx_timeout_ms);
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
