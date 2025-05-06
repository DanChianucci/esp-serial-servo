#include "ServoPacket.h"


uint8_t ServoPacket::size() {
  return raw_buffer.size();
}

const std::vector<uint8_t>& ServoPacket::get_buffer() {
  return raw_buffer;
}

const uint8_t& ServoPacket::operator[](int idx) const {
  return raw_buffer[idx];
}
