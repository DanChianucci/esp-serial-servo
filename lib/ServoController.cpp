
#include "ServoController.h"

#include <esp_check.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>

ServoController::ServoController(ServoBus* bus, uint32_t tx_timeout_ticks,
                                 uint32_t rx_timeout_ticks,
                                 int auto_disable_rx) {
  m_servo_bus = bus;
  m_transmit_timeout = tx_timeout_ticks;
  m_response_timeout = rx_timeout_ticks;
  m_auto_disable_rx = auto_disable_rx >= 0
                          ? (auto_disable_rx != 0)
                          : (bus->get_tx_pin() == bus->get_rx_pin());
}

int ServoController::send_command(ServoPacket* command) {
  return send_raw_command(command->as_bytes());
}
int ServoController::read_response(ServoPacket* response) {
  response->reset();

  int size = read_raw_response(response->sync_buffer(), response->data_buffer(),
                               response->len_index());
  if (size == ESP_FAIL) return ESP_FAIL;

  response->resize(size);
  return size;
}

int ServoController::send_raw_command(const std::span<const uint8_t> data) {
  ESP_LOGD(LOG_TAG, "send_raw_command: ");
  ESP_LOG_BUFFER_HEXDUMP(LOG_TAG, data.data(), data.size(), ESP_LOG_DEBUG);

  if (m_auto_disable_rx) m_servo_bus->enable_rx(false);
  int ret = m_servo_bus->write_bytes(data, m_transmit_timeout);
  if (m_auto_disable_rx) m_servo_bus->enable_rx(true);
  m_servo_bus->discard_input();
  return ret;
};

// TODO FAIL if read less than requested.
// TODO Fail if packet doesn't have size info
// TODO consolodate the 3 sections
int ServoController::read_raw_response(
    const std::span<const uint8_t> sync_buffer,
    const std::span<uint8_t>& data_buffer, size_t len_pos) {
  ESP_LOGD(LOG_TAG, "read_raw_rsp: ");
  ESP_LOG_BUFFER_HEXDUMP(LOG_TAG, sync_buffer.data(), sync_buffer.size(),
                         ESP_LOG_DEBUG);

  esp_err_t stat;

  size_t readlen = 0;
  size_t pktlen = 0;
  TickType_t ticks_to_wait = m_response_timeout;

  TimeOut_t xTimeOut;
  vTaskSetTimeOutState(&xTimeOut);

  // 1. Read until sync pattern is found
  stat = this->m_servo_bus->read_sync(sync_buffer, ticks_to_wait);
  ESP_RETURN_ON_FALSE(stat != ESP_FAIL, ESP_FAIL, LOG_TAG,
                      "Failed to read sync pattern");
  stat = xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait);
  ESP_RETURN_ON_FALSE(stat == pdFALSE, ESP_FAIL, LOG_TAG,
                      "Timeout waiting for sync pattern");

  // 2. Read dev id, length
  readlen = (len_pos + 1);
  if ((pktlen + readlen) > data_buffer.size())
    readlen = data_buffer.size() - pktlen;
  if (readlen > 0) {
    stat = this->m_servo_bus->read_bytes(data_buffer.subspan(pktlen, readlen),
                                         ticks_to_wait);
    ESP_RETURN_ON_FALSE(stat != ESP_FAIL, ESP_FAIL, LOG_TAG,
                        "Failed to read packet header");
    stat = xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait);
    ESP_RETURN_ON_FALSE(stat == pdFALSE, ESP_FAIL, LOG_TAG,
                        "Timeout waiting for packet header");
    pktlen += readlen;
  }

  // 3. Read until checksum is found
  readlen = data_buffer[len_pos];
  if ((pktlen + readlen) > data_buffer.size())
    readlen = data_buffer.size() - pktlen;
  if (readlen > 0) {
    stat = this->m_servo_bus->read_bytes(data_buffer.subspan(pktlen, readlen),
                                         ticks_to_wait);
    ESP_RETURN_ON_FALSE(stat != ESP_FAIL, ESP_FAIL, LOG_TAG,
                        "Failed to read packet contents");
    stat = xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait);
    ESP_RETURN_ON_FALSE(stat == pdFALSE, ESP_FAIL, LOG_TAG,
                        "Timeout waiting for packet contents");
    pktlen += readlen;
  }
  ESP_LOGD(LOG_TAG, "read_raw_rsp: ");
  ESP_LOG_BUFFER_HEXDUMP(LOG_TAG, data_buffer.data(), pktlen, ESP_LOG_DEBUG);

  return pktlen + sync_buffer.size();
}
