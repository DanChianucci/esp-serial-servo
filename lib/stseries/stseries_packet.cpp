#include "serial_servo/stseries/stseries_packet.h"

#include <algorithm>
#include <cassert>
#include <numeric>

#include "serial_servo/base/servo_controller.h"
#include "serial_servo/base/servo_internal.h"

namespace esp_serial_servo {



// ##############################################################################
//<> Initialization
// ##############################################################################

using namespace STS;

STSeriesPacket::STSeriesPacket(const buffer_t& buffer) : ServoPacket(buffer) {
  assert((m_buffer.size() >= NUM_META_BYTES));
  reset();
}


uint8_t STSeriesPacket::len_index() { return LENGTH_IDX-NUM_SYNC_BYTES; };
uint8_t STSeriesPacket::checksum_index() { return (m_size > 0) ? (m_size - 1) : 0; };
uint8_t STSeriesPacket::data_size() { return (m_size >= NUM_META_BYTES) ? (m_size - NUM_META_BYTES) : 0; }



srv_stat_t STSeriesPacket::reset() {
  m_size = 0;
  return set_sync(SYNC_PATTERN);
}

srv_stat_t STSeriesPacket::init(uint16_t sync, uint8_t dev_id, uint8_t length, uint8_t code, const buffer_const_t data,
                             uint8_t checksum) {
  m_size = 0;
  SRV_RETURN_ON_ERR(set_sync(sync));
  SRV_RETURN_ON_ERR(set_device_id(dev_id));
  SRV_RETURN_ON_ERR(set_length(length));
  SRV_RETURN_ON_ERR(set_code(code));
  SRV_RETURN_ON_ERR(set_data(data));
  SRV_RETURN_ON_ERR(set_checksum(checksum));
  return SRV_OK;
}

srv_stat_t STSeriesPacket::init(uint8_t dev_id, uint8_t code, const buffer_const_t data) {
  SRV_RETURN_ON_ERR(init(SYNC_PATTERN, dev_id, data.size() + (NUM_CODE_BYTES + NUM_CHKSUM_BYTES), code, data, 0x00));
  return update_checksum();
}

// ##############################################################################
//<> Buffer Access
// ##############################################################################

buffer_t STSeriesPacket::sync_buffer() { return m_buffer.first(NUM_SYNC_BYTES); }
buffer_t STSeriesPacket::data_buffer() { return m_buffer.subspan(NUM_SYNC_BYTES); }

// ##############################################################################
//<> Validation
// ##############################################################################

bool STSeriesPacket::is_well_formed() {
  return (size() >= NUM_META_BYTES) &&  // 1. Packet at least 6 bytes
         (get_sync() == SYNC_PATTERN) &&
         (get_length() == compute_length()) &&    // 2. Length field matches actual packet length
         (get_checksum() == compute_checksum());  // 3. Checksum field matches expected
}

uint8_t STSeriesPacket::compute_checksum() {
  return ~std::accumulate(m_buffer.begin() + NUM_SYNC_BYTES, m_buffer.begin() + m_size - 1, 0);
}

uint8_t STSeriesPacket::compute_length() { return (m_size - NUM_META_BYTES) + (NUM_CODE_BYTES + NUM_CHKSUM_BYTES); }

// ##############################################################################
//<> Getters
// ##############################################################################

uint16_t STSeriesPacket::get_sync() { return get_word(SYNC_IDX); }

uint8_t STSeriesPacket::get_device_id() { return get_byte(DEVID_IDX); }

uint8_t STSeriesPacket::get_length() { return get_byte(LENGTH_IDX); }

uint8_t STSeriesPacket::get_code() { return get_byte(CODE_IDX); }

uint8_t STSeriesPacket::get_checksum() { return m_size >= NUM_META_BYTES ? get_byte(m_size - 1) : 0x00; }

buffer_const_t STSeriesPacket::get_data() {
  if (m_size > NUM_META_BYTES)
    return m_buffer.subspan(DATA_IDX, m_size - NUM_META_BYTES);
  else
    return buffer_const_t();
}

uint8_t STSeriesPacket::get_byte(uint8_t idx, uint8_t default_val) {
  if (idx >= m_size) return default_val;
  return m_buffer[idx];
}

uint16_t STSeriesPacket::get_word(uint8_t idx, uint16_t default_val) {
  if ((uint16_t)idx + 1 >= m_size) return default_val;
  return (uint16_t)(m_buffer[idx + 1]) << 8 | (m_buffer[idx]);
}

uint8_t STSeriesPacket::get_data_byte(uint8_t idx, uint8_t default_val) { return get_byte(idx + DATA_IDX, default_val); }

uint16_t STSeriesPacket::get_data_word(uint8_t idx, uint16_t default_val) {
  return get_word(idx + DATA_IDX, default_val);
}

// ##############################################################################
//<> Setters
// ##############################################################################

srv_stat_t STSeriesPacket::set_sync(uint16_t val) {
  SRV_RETURN_ON_ERR(resize(std::max((size_t)SYNC_IDX + NUM_SYNC_BYTES, m_size)));
  return set_word(SYNC_IDX, val);
}

srv_stat_t STSeriesPacket::set_device_id(uint8_t val) {
  SRV_RETURN_ON_ERR(resize(std::max((size_t)DEVID_IDX + NUM_DEVID_BYTES, m_size)));
  return set_byte(DEVID_IDX, val);
}

srv_stat_t STSeriesPacket::set_length(uint8_t val) {
  SRV_RETURN_ON_ERR(resize(std::max((size_t)LENGTH_IDX + NUM_LENGTH_BYTES, m_size)));
  return set_byte(LENGTH_IDX, val);
}

srv_stat_t STSeriesPacket::set_code(uint8_t val) {
  SRV_RETURN_ON_ERR(resize(std::max((size_t)CODE_IDX + NUM_CODE_BYTES, m_size)));
  return set_byte(CODE_IDX, val);
}

srv_stat_t STSeriesPacket::set_checksum(uint8_t val) {
  SRV_RETURN_ON_ERR(resize(std::max((size_t)NUM_META_BYTES, m_size)));
  return set_byte(m_size - 1, val);
}

srv_stat_t STSeriesPacket::set_data(const buffer_const_t val) {
  SRV_RETURN_ON_ERR(resize(NUM_META_BYTES + val.size()));
  std::copy(val.begin(), val.end(), m_buffer.begin() + DATA_IDX);
  return SRV_OK;
}

srv_stat_t STSeriesPacket::update_length() { return set_length(compute_length()); }

srv_stat_t STSeriesPacket::update_checksum() { return set_checksum(compute_checksum()); }

srv_stat_t STSeriesPacket::set_byte(size_t idx, uint8_t val) {
  SRV_RETURN_IF(capacity() <= idx, SRV_OVERFLOW);
  m_buffer[idx] = val;
  return SRV_OK;
}

srv_stat_t STSeriesPacket::set_word(size_t idx, uint16_t val) {
  SRV_RETURN_IF(capacity() <= idx + 1, SRV_OVERFLOW);
  m_buffer[0] = val >> 8;
  m_buffer[1] = val & 0xFF;
  return SRV_OK;
}

}  // namespace esp_serial_servo