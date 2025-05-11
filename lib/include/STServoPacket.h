#ifndef STSERVOPACKET__H
#define STSERVOPACKET__H

#include <cstdint>

#include "STServoDefines.h"
#include "ServoPacket.h"

class STServoPacket : public ServoPacket {
 public:
  STServoPacket() = default;
  STServoPacket(const buffer_t& buffer);

  virtual buffer_t sync_buffer();
  virtual buffer_t data_buffer();
  virtual size_t len_index();

  virtual void reset();

  virtual bool is_well_formed();

  uint8_t compute_checksum();
  uint8_t compute_length();

  uint16_t get_sync();
  uint8_t get_device_id();
  uint8_t get_length();
  uint8_t get_code();
  buffer_const_t get_data();
  uint8_t get_checksum();

  STServoPacket& set_sync(uint16_t val);
  STServoPacket& set_dev_id(uint8_t val);
  STServoPacket& set_length(uint8_t val);
  STServoPacket& set_code(uint8_t val);
  STServoPacket& set_data(const buffer_const_t val);
  STServoPacket& set_checksum(uint8_t val);

  STServoPacket& update_length();
  STServoPacket& update_checksum();

  /**
   * @brief STServoPacket fullly specified constructor
   * @param header Header Field
   * @param dev_id Device ID Field
   * @param length Length Field
   * @param code   Instruction/Status Code Field
   * @param data   Data Parameters Field
   * @param chksum Checksum Field
   */
  STServoPacket& init(uint16_t sync, uint8_t dev_id, uint8_t length,
                      uint8_t code, const buffer_const_t data,
                      uint8_t checksum);
  STServoPacket& init(uint8_t dev_id, uint8_t code, const buffer_const_t data);

 private:
  inline uint8_t get_byte(uint8_t idx, uint8_t default_val = 0x00);
  inline uint16_t get_word(uint8_t idx, uint16_t default_val = 0x0000);

  inline STServoPacket& set_byte(size_t idx, uint8_t val);
  inline STServoPacket& set_word(size_t idx, uint16_t val);
};

#endif  // STSERVOPACKET__H