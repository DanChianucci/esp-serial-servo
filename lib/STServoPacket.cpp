#include "STServoPacket.h"

#include <algorithm>
#include <numeric>

using std::clamp;
using std::max;

// ##############################################################################
//<> Initialization
// ##############################################################################

STServoPacket::STServoPacket(const buffer_t& buffer) : ServoPacket(buffer) { reset(); }

int STServoPacket::reset() {
  m_size = 0;
  return set_sync(STS_SYNC_PATTERN);
}

int STServoPacket::init(uint16_t sync, uint8_t dev_id, uint8_t length, uint8_t code, const buffer_const_t data,
                        uint8_t checksum) {
  SRV_RETURN_ON_ERR(set_sync(sync));
  SRV_RETURN_ON_ERR(set_device_id(dev_id));
  SRV_RETURN_ON_ERR(set_length(length));
  SRV_RETURN_ON_ERR(set_code(code));
  SRV_RETURN_ON_ERR(set_data(data));
  SRV_RETURN_ON_ERR(set_checksum(checksum));
  return SRV_OK;
}

int STServoPacket::init(uint8_t dev_id, uint8_t code, const buffer_const_t data) {
  SRV_RETURN_ON_ERR(init(STS_SYNC_PATTERN, dev_id, data.size() + 2, code, data, 0x00));
  return update_checksum();
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
         (get_length() == compute_length()) &&    // 2. Length field matches actual packet length
         (get_checksum() == compute_checksum());  // 3. Checksum field matches expected
}

uint8_t STServoPacket::compute_checksum() {
  return ~std::accumulate(m_buffer.begin() + 2, m_buffer.begin() + m_size - 1, 0);
}

uint8_t STServoPacket::compute_length() { return m_size - 4; }

// ##############################################################################
//<> Getters
// ##############################################################################

uint16_t STServoPacket::get_sync() { return get_word(0); }

uint8_t STServoPacket::get_device_id() { return get_byte(1); }

uint8_t STServoPacket::get_length() { return get_byte(2); }

uint8_t STServoPacket::get_code() { return get_byte(3); }

uint8_t STServoPacket::get_checksum() { return m_size >= 6 ? get_byte(m_size - 1) : 0x00; }

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

int STServoPacket::set_sync(uint16_t val) {
  SRV_RETURN_ON_ERR(resize(max((size_t)2, m_size)));
  return set_word(0, val);
}

int STServoPacket::set_device_id(uint8_t val) {
  SRV_RETURN_ON_ERR(resize(max((size_t)3, m_size)));
  return set_byte(2, val);
}

int STServoPacket::set_length(uint8_t val) {
  SRV_RETURN_ON_ERR(resize(max((size_t)4, m_size)));
  return set_byte(3, val);
}

int STServoPacket::set_code(uint8_t val) {
  SRV_RETURN_ON_ERR(resize(max((size_t)5, m_size)));
  return set_byte(4, val);
}

int STServoPacket::set_checksum(uint8_t val) {
  SRV_RETURN_ON_ERR(resize(max((size_t)6, m_size)));
  return set_byte(m_size - 1, val);
}

int STServoPacket::set_data(const buffer_const_t val) {
  SRV_RETURN_ON_ERR(resize(6 + val.size()));
  if (capacity() < (5 + val.size())) return SRV_OVERFLOW;
  std::copy(val.begin(), val.end(), m_buffer.begin() + 5);
  return SRV_OK;
}

int STServoPacket::update_length() { return set_length(compute_length()); }

int STServoPacket::update_checksum() { return set_checksum(compute_checksum()); }

int STServoPacket::set_byte(size_t idx, uint8_t val) {
  if (capacity() <= idx) return SRV_OVERFLOW;
  m_buffer[idx] = val;
  return SRV_OK;
}

int STServoPacket::set_word(size_t idx, uint16_t val) {
  if (capacity() <= idx + 1) return SRV_OVERFLOW;
  m_buffer[0] = val >> 8;
  m_buffer[1] = val & 0xFF;
  return SRV_OK;
}