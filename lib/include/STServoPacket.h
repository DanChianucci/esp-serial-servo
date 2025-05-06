// C(const C&) = default;               // Copy constructor
// C(C&&) = default;                    // Move constructor
// C& operator=(const C&) = default;  // Copy assignment operator
// C& operator=(C&&) = default;       // Move assignment operator
// virtual ~C() { }                     // Destructor

#ifndef STSERVOPACKET__H
#define STSERVOPACKET__H
#include <esp_log.h>

#include "STServoDefines.h"
#include "ServoPacket.h"

class STServoPacket : public ServoPacket {
 public:
  STServoPacket() = default;

  /**
   * @brief STServoPacket fullly specified constructor
   * @param header Header Field
   * @param dev_id Device ID Field
   * @param length Length Field
   * @param code   Instruction/Status Code Field
   * @param data   Data Parameters Field
   * @param chksum Checksum Field
   */
  template <std::input_iterator T>
  STServoPacket(uint16_t sync_pattern, uint8_t dev_id, uint8_t length,
                uint8_t code, T dbegin, T dend, uint8_t chksum) {
    if constexpr (std::endian::native != STS_ENDIANNESS) {
      sync_pattern = std::byteswap(sync_pattern);
    }
    raw_buffer = std::vector<uint8_t>(std::distance(dbegin, dend) + 6);

    auto t = {uint8_t(sync_pattern >> 8), uint8_t(sync_pattern & 0xFF), dev_id,
              length, code};
    std::copy(t.begin(), t.end(), raw_buffer.begin());

    std::copy(dbegin, dend, raw_buffer.begin() + 5);
    raw_buffer.back() = chksum;
  }

  template <std::ranges::range T>
  STServoPacket(uint16_t sync_pattern, uint8_t dev_id, uint8_t length,
                uint8_t code, T&& data, uint8_t chksum)
      : STServoPacket(sync_pattern, dev_id, length, code,
                      std::ranges::begin(data), std::ranges::end(data),
                      chksum) {}

  template <std::input_iterator T>
  STServoPacket(uint8_t dev_id, uint8_t code, T dbegin, T dend)
      : STServoPacket(STS_SYNC_PATTERN, dev_id, std::distance(dbegin, dend) + 2,
                      code, dbegin, dend, 0x00) {
    raw_buffer.back() = compute_checksum();
  }

  template <std::ranges::input_range T>
  STServoPacket(uint8_t dev_id, uint8_t code, T&& data)
      : STServoPacket(STS_SYNC_PATTERN, dev_id, std::ranges::size(data) + 2,
                      code, std::ranges::begin(data), std::ranges::end(data),
                      0x00) {
    raw_buffer.back() = compute_checksum();
  }

  template <typename T>
  void set_buffer(T&& sequence) {
    raw_buffer = std::forward<T>(sequence);
  }

  bool is_well_formed();
  uint8_t compute_checksum();
  uint8_t compute_length();

  uint16_t get_header();
  uint8_t get_device_id();
  uint8_t get_length();
  uint8_t get_code();
  std::span<const uint8_t> get_data();
  uint8_t get_checksum();
};

#endif  // STSERVOPACKET__H