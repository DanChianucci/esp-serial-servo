#ifndef SERVOPACKET__H
#define SERVOPACKET__H
#include <cstdint>
#include <span>

class ServoPacket {
 public:
  typedef std::span<uint8_t> buffer_t;
  typedef std::span<const uint8_t> buffer_const_t;

  ServoPacket() = default;
  ServoPacket(const buffer_t& buffer);

  // returns a view into the packet buffer sync_header
  virtual buffer_t sync_buffer() = 0;
  // Retuns a view into the packet buffer without the sync_header
  virtual buffer_t data_buffer() = 0;
  // Returns the index of the length byte in the buffer
  virtual size_t len_index() = 0;

  // Returns true if the packet is well formed and internally consistant
  virtual bool is_well_formed();

  // Resets the packet to a known state containing only the sync header
  virtual void reset();

  // The maximum capacity of the raw buffer
  virtual size_t capacity();

  // Returns a view into the raw buffer
  virtual buffer_t raw_buffer();

  // The Actual size of the packet
  virtual size_t size();

  // Updates the size of the stored packet. does not alter any bytes
  virtual void resize(size_t size);

  // Returns a view of the packet as a sequence of bytes
  virtual buffer_const_t as_bytes();

 protected:
  static constexpr const char LOG_TAG[] = "ServoPacket";
  buffer_t m_buffer;
  size_t m_size;
};

#endif  // SERVOPACKET__H
