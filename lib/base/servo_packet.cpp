#include "serial_servo/base/servo_packet.h"
#include "serial_servo/base/servo_internal.h"
#include <utility>

namespace esp_serial_servo {

ServoPacket::ServoPacket(const buffer_t& buffer) {
  m_buffer = buffer;
  m_size = 0;
}

bool ServoPacket::is_well_formed() { return m_size > 0; }

srv_stat_t ServoPacket::reset() {
  m_size = 0;
  return SRV_OK;
}

srv_stat_t ServoPacket::resize(size_t size) {
  SRV_RETURN_IF(capacity() < size, SRV_OVERFLOW);
  m_size = size;
  return SRV_OK;
}

size_t ServoPacket::capacity() { return m_buffer.size(); }
size_t ServoPacket::size() { return m_size; }
buffer_t ServoPacket::raw_buffer() { return m_buffer; };

buffer_const_t ServoPacket::as_bytes() { return m_buffer.first(m_size); }

}  // namespace esp_serial_servo