#include "STServoController.h"
#include "STServoPacket.h"
#include "ServoBus.h"



STServoController::STServoController(ServoBus* bus, int tx_timeout_ms, int rx_timeout_ms, int auto_disable_rx)
    : ServoController(bus, tx_timeout_ms, rx_timeout_ms,auto_disable_rx) {
}


int STServoController::send_raw_command(STServoPacket& cmd)   {

  ESP_LOGD(LOG_TAG, "Sending command: ");
  ESP_LOG_BUFFER_HEXDUMP(LOG_TAG, cmd.get_buffer().data(), cmd.get_buffer().size(), ESP_LOG_DEBUG);

  if (m_auto_disable_rx) m_servo_bus->enable_rx(false);
  int ret = m_servo_bus->write_bytes(cmd.get_buffer(), m_transmit_timeout);
  if (m_auto_disable_rx) m_servo_bus->enable_rx(true);
  m_servo_bus->discard_input();
  return ret != ESP_FAIL ? ESP_OK : ESP_FAIL;
};

//TODO:  Adda m_max_response_length and an override to this function
int STServoController::read_raw_response(STServoPacket& response) {
  esp_err_t stat;

  TimeOut_t xTimeOut;
  TickType_t ticks_to_wait = m_response_timeout;
  bool timed_out;
  vTaskSetTimeOutState( &xTimeOut );

  // 1. Read until header is found
  std::vector<uint8_t> buffer({0xFF,0xFF});
  stat = this->m_servo_bus->read_sync(buffer, ticks_to_wait);
  ESP_RETURN_ON_FALSE(stat!=ESP_FAIL,ESP_FAIL,LOG_TAG,"Failed to read sync pattern");
  timed_out = xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE;
  ESP_RETURN_ON_FALSE(!timed_out,ESP_FAIL,LOG_TAG,"Timeout waiting for sync pattern");


  // 2. Read dev id, length
  buffer.resize(4);
  stat = this->m_servo_bus->read_bytes(buffer.begin()+2, buffer.end(),ticks_to_wait);
  ESP_RETURN_ON_FALSE(stat!=ESP_FAIL,ESP_FAIL,LOG_TAG,"Failed to read packet header");
  timed_out = xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE;
  ESP_RETURN_ON_FALSE(!timed_out,ESP_FAIL,LOG_TAG,"Timeout waiting for packet header");

  // 3. Read until checksum is found
  buffer.resize(buffer[3]+4);
  stat = this->m_servo_bus->read_bytes(buffer.begin()+4, buffer.end(), ticks_to_wait);
  ESP_RETURN_ON_FALSE(stat!=ESP_FAIL,ESP_FAIL,LOG_TAG,"Failed to read packet contents");
  timed_out = xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE;
  ESP_RETURN_ON_FALSE(!timed_out,ESP_FAIL,LOG_TAG,"Timeout waiting for packet contents");


  ESP_LOGD(LOG_TAG, "Received Response: ");
  ESP_LOG_BUFFER_HEXDUMP(LOG_TAG, buffer.data(), buffer.size(), ESP_LOG_DEBUG);

  // 4. Construct response packet
  response.set_buffer(buffer);
  return ESP_OK;
};