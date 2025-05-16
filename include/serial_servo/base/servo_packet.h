#ifndef SERVO_PACKET__H
#define SERVO_PACKET__H
#include <cstdint>
#include <span>

#include "serial_servo/base/servo_utils.h"

namespace esp_serial_servo {
class ServoPacket {
 public:
  static constexpr char LOG_TAG[] = "ServoPacket";

  ServoPacket() = delete;
  ServoPacket(const buffer_t& buffer);

  // returns a view into the packet buffer sync_header
  virtual buffer_t sync_buffer() = 0;
  // Retuns a view into the packet buffer without the sync_header
  virtual buffer_t data_buffer() = 0;
  // Returns the index of the length byte in the buffer
  virtual uint8_t len_index() = 0;

  /** Determines if the packet is valid
   * @return true if the packet is well formed and internally consistant
   */
  virtual bool is_well_formed()=0;

  /** Resets the packet to a known state containing only the sync header
   * @return SRV_OK always.
   */
  virtual srv_stat_t reset()=0;

  /** Updates the size of the stored packet. does not alter any bytes
   * @return SRV_OK on success.
   * @return SRV_OVERFLOW if size > buffer capacity
   */
  virtual srv_stat_t resize(size_t size);

  // The maximum capacity of the raw buffer
  virtual size_t capacity();

  // Returns a view into the raw buffer
  virtual buffer_t raw_buffer();

  // The Actual size of the packet
  virtual size_t size();

  // Returns a view of the packet as a sequence of bytes
  virtual buffer_const_t as_bytes();

 protected:
  buffer_t m_buffer;
  size_t m_size;
};

}  // namespace esp_serial_servo
#endif  // SERVOPACKET__H
