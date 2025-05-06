#include "STServoPacket.h"

#include <numeric>


//##############################################################################
//<> Utility Functions
//##############################################################################
inline uint8_t get(std::vector<uint8_t> &v, uint8_t idx, uint8_t default_val = 0x00) {
  if ( idx >= v.size())
    return default_val;
  return v[idx];
}

inline uint16_t get_word(std::vector<uint8_t> &v, uint8_t idx, uint16_t default_val = 0x0000) {
  if ((uint16_t)idx + 1 >= v.size())
    return default_val;

  uint16_t header = (uint16_t)(v[idx]) << 8 | (v[idx + 1]);
  if (std::endian::native != STS_ENDIANNESS) {
    header = std::byteswap(header);
  }
  return header;
}

//##############################################################################
//<> STServoPacket Methods
//##############################################################################
bool STServoPacket::is_well_formed() {
  return (size() >= 6) &&                        // 1. Packet at least 6 bytes
         (get_length() == compute_length()) &&   // 2. Length field matches actual packet length
         (get_checksum() == compute_checksum()); // 3. Checksum field matches expected
}

uint8_t STServoPacket::compute_checksum() {
  return ~std::accumulate(raw_buffer.begin()+2, raw_buffer.end() - 1, 0);
}
uint8_t STServoPacket::compute_length() {
  return raw_buffer.size() - 4;
}

uint16_t STServoPacket::get_header()    { return get_word(raw_buffer,0);   }
uint8_t  STServoPacket::get_device_id() { return get(raw_buffer,2);        }
uint8_t  STServoPacket::get_length()    { return get(raw_buffer,3);        }
uint8_t  STServoPacket::get_code()      { return get(raw_buffer,4);        }
uint8_t  STServoPacket::get_checksum()  { return get(raw_buffer, size()-1);}
std::span<const uint8_t> STServoPacket::get_data() {
  if (raw_buffer.size() > 6)
    return std::span(raw_buffer.begin() + 5, raw_buffer.size() - 6);
  else
    return std::span<const uint8_t>();
}
