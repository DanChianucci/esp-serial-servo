#include "ServoPacket.h"

#include <utility>
typedef ServoPacket::buffer_t buffer_t;
typedef ServoPacket::buffer_const_t buffer_const_t;

ServoPacket::ServoPacket(const buffer_t& buffer) {
  m_buffer = buffer;
  m_size = 0;
}

bool ServoPacket::is_well_formed() { return m_size > 0; }
void ServoPacket::reset() { m_size = 0; }

// TODO check capacity
void ServoPacket::resize(size_t size) { m_size = size; }

size_t ServoPacket::capacity() { return m_buffer.size(); }
size_t ServoPacket::size() { return m_size; }
buffer_t ServoPacket::raw_buffer() { return m_buffer; };

buffer_const_t ServoPacket::as_bytes() { return m_buffer.first(m_size); }