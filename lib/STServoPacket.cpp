#include "STServoPacket.h"

#include <esp_err.h>
#include <esp_log.h>

#include <algorithm>
#include <numeric>

typedef STServoPacket::buffer_t buffer_t;
typedef STServoPacket::buffer_const_t buffer_const_t;

using std::clamp;
using std::max;

// ##############################################################################
//<> Initialization
// ##############################################################################

STServoPacket::STServoPacket(const buffer_t& buffer) : ServoPacket(buffer) {
  reset();
}

void STServoPacket::reset() {
  m_size = 2;
  set_sync(STS_SYNC_PATTERN);
}

STServoPacket& STServoPacket::init(uint16_t sync, uint8_t dev_id,
                                   uint8_t length, uint8_t code,
                                   const buffer_const_t data,
                                   uint8_t checksum) {
  set_sync(sync);
  set_dev_id(dev_id);
  set_length(length);
  set_code(code);
  set_data(data);
  set_checksum(checksum);
  return *this;
}

STServoPacket& STServoPacket::init(uint8_t dev_id, uint8_t code,
                                   const buffer_const_t data) {
  init(STS_SYNC_PATTERN, dev_id, data.size() + 2, code, data, 0x00);
  update_checksum();
  return *this;
}

// ##############################################################################
//<> Buffer Access
// ##############################################################################

buffer_t STServoPacket::sync_buffer() { return m_buffer.first(2); }

buffer_t STServoPacket::data_buffer() { return m_buffer.subspan(2); }

size_t STServoPacket::len_index() { return 1; }

// ##############################################################################
//<> Validation
// ##############################################################################

bool STServoPacket::is_well_formed() {
  return (size() >= 6) &&  // 1. Packet at least 6 bytes
         (get_sync() == STS_SYNC_PATTERN) &&
         (get_length() ==
          compute_length()) &&  // 2. Length field matches actual packet length
         (get_checksum() ==
          compute_checksum());  // 3. Checksum field matches expected
}

uint8_t STServoPacket::compute_checksum() {
  return ~std::accumulate(m_buffer.begin() + 2, m_buffer.begin() + m_size - 1,
                          0);
}

uint8_t STServoPacket::compute_length() { return m_size - 4; }

// ##############################################################################
//<> Getters
// ##############################################################################

uint16_t STServoPacket::get_sync() { return get_word(0); }

uint8_t STServoPacket::get_device_id() { return get_byte(1); }

uint8_t STServoPacket::get_length() { return get_byte(2); }

uint8_t STServoPacket::get_code() { return get_byte(3); }

uint8_t STServoPacket::get_checksum() {
  return m_size >= 6 ? get_byte(m_size - 1) : 0x00;
}

buffer_const_t STServoPacket::get_data() {
  if (m_size > 6)
    return m_buffer.subspan(5, m_size - 6);
  else
    return buffer_const_t();
}

uint8_t STServoPacket::get_byte(uint8_t idx, uint8_t default_val) {
  if (idx >= m_size) return default_val;
  return m_buffer[idx];
}

uint16_t STServoPacket::get_word(uint8_t idx, uint16_t default_val) {
  if ((uint16_t)idx + 1 >= m_size) return default_val;
  return (uint16_t)(m_buffer[idx + 1]) << 8 | (m_buffer[idx]);
}

// ##############################################################################
//<> Setters
// ##############################################################################

STServoPacket& STServoPacket::set_sync(uint16_t val) {
  resize(max((size_t)2, m_size));
  return set_word(0, val);
}

STServoPacket& STServoPacket::set_dev_id(uint8_t val) {
  resize(max((size_t)3, m_size));
  return set_byte(2, val);
}

STServoPacket& STServoPacket::set_length(uint8_t val) {
  resize(max((size_t)4, m_size));
  return set_byte(3, val);
}

STServoPacket& STServoPacket::set_code(uint8_t val) {
  resize(max((size_t)5, m_size));
  return set_byte(4, val);
}

STServoPacket& STServoPacket::set_checksum(uint8_t val) {
  resize(max((size_t)6, m_size));
  return set_byte(m_size - 1, val);
}

STServoPacket& STServoPacket::set_data(const buffer_const_t val) {
  ESP_LOGE(LOG_TAG, "set_data: %d   %d", m_size, val.size());
  ESP_LOG_BUFFER_HEXDUMP(LOG_TAG, val.data(), val.size(), ESP_LOG_ERROR);

  if (capacity() < (5 + val.size())) {
    ESP_LOGE(LOG_TAG, "set_data: data[%d+6] Out of bounds %d", val.size(),
             capacity());
    ESP_ERROR_CHECK(ESP_FAIL);
  }
  std::copy(val.begin(), val.end(), m_buffer.begin() + 5);
  resize(6 + val.size());
  return *this;
}

STServoPacket& STServoPacket::update_length() {
  return set_length(compute_length());
}

STServoPacket& STServoPacket::update_checksum() {
  return set_checksum(compute_checksum());
}

STServoPacket& STServoPacket::set_byte(size_t idx, uint8_t val) {
  if (capacity() <= idx) {
    ESP_LOGE(LOG_TAG, "set_byte: %d Out of bounds %d", idx, capacity());
    ESP_ERROR_CHECK(ESP_FAIL);
  }
  m_buffer[idx] = val;
  return *this;
}

STServoPacket& STServoPacket::set_word(size_t idx, uint16_t val) {
  if (capacity() <= idx + 1) {
    ESP_LOGE(LOG_TAG, "set_word: %d Out of bounds %d", idx, capacity());
    ESP_ERROR_CHECK(ESP_FAIL);
  }
  m_buffer[0] = val >> 8;
  m_buffer[1] = val & 0xFF;
  return *this;
}