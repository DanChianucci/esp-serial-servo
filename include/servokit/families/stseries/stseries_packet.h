//------------------------------------------------------------------------
// Filename  :  stseries_packet.h
// Project   :  servokit
//
// Description:
//    STSeriesPacket: the STS frame in a caller-owned buffer.
//
//      index   0   1   2    3       4      5 ...        last
//      byte    FF  FF  id   length  code   data...      checksum
//
//    length   = number of bytes after the length byte (code + data + checksum).
//    code     = instruction in a command, status byte in a reply.
//    checksum = ~(id + length + code + data...), low byte.
//    A frame has at least NUM_META_BYTES (6) bytes. A 16-bit data word is
//    little endian.
//
//    Contract common to every method:
//      Blocking   None. Packets do no I/O.
//      Threads    Not thread-safe.
//      Lifetimes  The buffer is borrowed and must outlive the packet; views
//                 returned by the packet point into that buffer and are
//                 invalidated by the next call that changes the packet.
//      Errors     Setters return srv_stat_t: SRV_OK, or SRV_OVERFLOW when the
//                 buffer is too small. Getters never fail: a read beyond the
//                 current packet size returns the stated default.
//------------------------------------------------------------------------

#ifndef STSERIES_PACKET__H
#define STSERIES_PACKET__H

#include <cstdint>

#include "servokit/serial/servo_packet.h"
#include "servokit/families/stseries/stseries_defines.h"

namespace servokit {
class ServoController;

/** An STS command or reply frame (see the file header for the layout). */
class STSeriesPacket : public ServoPacket {
 public:
  /** Declared for completeness but not usable: ServoPacket has no default
   *  constructor, so this one is implicitly deleted. Use the buffer constructor. */
  STSeriesPacket() = default;
  /** Binds @p buffer (at least NUM_META_BYTES bytes, asserted) and resets the
   *  packet to the sync header. */
  STSeriesPacket(const buffer_t& buffer);

  /** @return The two sync bytes at the start of the buffer. */
  virtual buffer_t sync_buffer();
  /** @return The buffer after the sync bytes, up to its capacity. */
  virtual buffer_t data_buffer();

  /** @return The index of the length byte within data_buffer() (1: the
   *  device ID comes first). */
  virtual uint8_t len_index();
  /** @return The index of the checksum byte (the last byte of the packet),
   *  or 0 for an empty packet. */
  virtual uint8_t checksum_index();
  /** @return The number of data bytes (those between the code and the
   *  checksum); 0 while the packet is shorter than NUM_META_BYTES. */
  virtual uint8_t data_size();

  /** Empties the packet and writes the sync header.
   *  @return SRV_OK, or SRV_OVERFLOW if the buffer is smaller than the sync pattern. */
  virtual srv_stat_t reset();

  /** @return true if the frame is valid including its checksum. */
  virtual bool is_well_formed();
  /** @return true if the size is at least NUM_META_BYTES, the sync pattern is
   *  0xFFFF and the length field matches the packet size. Does not check the
   *  checksum. */
  virtual bool is_frame_valid();
  /** @return true if the size is at least NUM_META_BYTES and the stored
   *  checksum equals compute_checksum(). */
  virtual bool is_checksum_valid();
  /** @return The device ID byte, or -1 if the packet is too short to hold one. */
  virtual int get_packet_device_id();

  /** @return The checksum this frame should carry: the inverted low byte of
   *  the sum of every byte after the sync pattern up to, not including, the
   *  checksum byte. Requires get_size() >= NUM_META_BYTES. */
  uint8_t compute_checksum();
  /** @return The length field this frame should carry, from its current size
   *  (data bytes + code + checksum). Requires get_size() >= NUM_META_BYTES. */
  uint8_t compute_length();

  /** @return The first two bytes read as a little-endian word, or 0 if the
   *  packet holds fewer than two bytes. */
  uint16_t get_sync();
  /** @return The device ID byte, or 0 if the packet is too short. */
  uint8_t get_device_id();
  /** @return The length field byte, or 0 if the packet is too short. */
  uint8_t get_length();
  /** @return The instruction/status byte, or 0 if the packet is too short. */
  uint8_t get_code();
  /** @return A view of the data bytes (between the code and the checksum);
   *  empty when there are none. */
  buffer_const_t get_data();
  /** @param idx Index within the data bytes.
   *  @param default_val Returned when the byte lies beyond the packet size.
   *  @return The byte at data index @p idx. The bound is the packet size, not
   *  the end of the data region, so the checksum byte can be read this way. */
  uint8_t get_data_byte(uint8_t idx, uint8_t default_val = 0x00);
  /** @param idx Index of the low byte within the data bytes.
   *  @param default_val Returned when the word lies beyond the packet size.
   *  @return The little-endian 16-bit word at data index @p idx. */
  uint16_t get_data_word(uint8_t idx, uint16_t default_val = 0x0000);
  /** @return The last byte of a packet of at least NUM_META_BYTES, else 0. */
  uint8_t get_checksum();

  /** Writes the sync header word, growing the packet to at least two bytes.
   *  The high byte goes to index 0, so only a symmetric pattern such as
   *  SYNC_PATTERN reads back unchanged through get_sync(). */
  srv_stat_t set_sync(uint16_t val);
  /** Writes the device ID byte, growing the packet to at least 3 bytes. */
  srv_stat_t set_device_id(uint8_t val);
  /** Writes the length field, growing the packet to at least 4 bytes. */
  srv_stat_t set_length(uint8_t val);
  /** Writes the instruction/status byte, growing the packet to at least 5 bytes. */
  srv_stat_t set_code(uint8_t val);
  /** Copies @p val into the data region and resizes the packet to hold it
   *  plus the fixed fields (NUM_META_BYTES + val.size()). Leaves length and
   *  checksum unchanged.
   *  @return SRV_OVERFLOW if the buffer cannot hold the result. */
  srv_stat_t set_data(const buffer_const_t val);
  /** Writes the checksum into the last byte, growing the packet to at least
   *  NUM_META_BYTES. */
  srv_stat_t set_checksum(uint8_t val);
  /** set_length(compute_length()). */
  srv_stat_t update_length();
  /** set_checksum(compute_checksum()). */
  srv_stat_t update_checksum();

  /** Builds a frame from explicit fields, with no consistency checks (used to
   *  build deliberately malformed frames). The packet is emptied first.
   *  @return SRV_OK, or the first error from the field setters. */
  srv_stat_t init(uint16_t sync, uint8_t dev_id, uint8_t length, uint8_t code, const buffer_const_t data,
                uint8_t checksum);
  /** Builds a complete command frame: sync pattern, @p dev_id, the matching
   *  length, @p code, @p data and a correct checksum.
   *  @return SRV_OK, or SRV_OVERFLOW if the buffer is too small. */
  srv_stat_t init(uint8_t dev_id, uint8_t code, const buffer_const_t data);



 private:
  // Byte and word accessors on the raw buffer. Reads past the packet size
  // return the default; writes past the buffer capacity return SRV_OVERFLOW.
  inline uint8_t get_byte(uint8_t idx, uint8_t default_val = 0x00);
  inline uint16_t get_word(uint8_t idx, uint16_t default_val = 0x0000);

  inline srv_stat_t set_byte(size_t idx, uint8_t val);
  inline srv_stat_t set_word(size_t idx, uint16_t val);
};

}  // namespace servokit
#endif  // STSERIES_PACKET__H
