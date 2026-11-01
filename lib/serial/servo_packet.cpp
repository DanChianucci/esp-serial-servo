// ServoPacket: the buffer bookkeeping shared by every protocol family.
#include "servokit/serial/servo_packet.h"
#include "servokit/serial/servo_internal.h"
#include <utility>

namespace servokit {

ServoPacket::ServoPacket(const buffer_t& buffer) {
  m_buffer = buffer;
  m_size = 0;
}

srv_stat_t ServoPacket::resize(size_t size) {
  SRV_RETURN_IF(get_capacity() < size, SRV_OVERFLOW);
  m_size = size;
  return SRV_OK;
}

size_t ServoPacket::get_capacity() { return m_buffer.size(); }
size_t ServoPacket::get_size() { return m_size; }
buffer_t ServoPacket::get_raw_buffer() { return m_buffer; };

buffer_const_t ServoPacket::as_bytes() { return m_buffer.first(m_size); }

}  // namespace servokit