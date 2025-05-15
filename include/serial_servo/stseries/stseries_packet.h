#ifndef STSERIES_PACKET__H
#define STSERIES_PACKET__H

#include <cstdint>

#include "serial_servo/base/servo_packet.h"
#include "serial_servo/stseries/stseries_defines.h"

namespace esp_serial_servo {
class ServoController;



class STSeriesPacket : public ServoPacket {
 public:
  STSeriesPacket() = default;
  STSeriesPacket(const buffer_t& buffer);

  virtual buffer_t sync_buffer();
  virtual buffer_t data_buffer();

  virtual uint8_t len_index();
  virtual uint8_t checksum_index();
  virtual uint8_t data_size();

  virtual srv_stat_t reset();

  virtual bool is_well_formed();

  uint8_t compute_checksum();
  uint8_t compute_length();

  uint16_t get_sync();
  uint8_t get_device_id();
  uint8_t get_length();
  uint8_t get_code();
  buffer_const_t get_data();
  uint8_t get_data_byte(uint8_t idx, uint8_t default_val = 0x00);
  uint16_t get_data_word(uint8_t idx, uint16_t default_val = 0x0000);
  uint8_t get_checksum();

  srv_stat_t set_sync(uint16_t val);
  srv_stat_t set_device_id(uint8_t val);
  srv_stat_t set_length(uint8_t val);
  srv_stat_t set_code(uint8_t val);
  srv_stat_t set_data(const buffer_const_t val);
  srv_stat_t set_checksum(uint8_t val);
  srv_stat_t update_length();
  srv_stat_t update_checksum();

  srv_stat_t init(uint16_t sync, uint8_t dev_id, uint8_t length, uint8_t code, const buffer_const_t data,
                uint8_t checksum);
  srv_stat_t init(uint8_t dev_id, uint8_t code, const buffer_const_t data);



 private:
  inline uint8_t get_byte(uint8_t idx, uint8_t default_val = 0x00);
  inline uint16_t get_word(uint8_t idx, uint16_t default_val = 0x0000);

  inline srv_stat_t set_byte(size_t idx, uint8_t val);
  inline srv_stat_t set_word(size_t idx, uint16_t val);
};

}  // namespace esp_serial_servo
#endif  // STSERIESPACKET__H