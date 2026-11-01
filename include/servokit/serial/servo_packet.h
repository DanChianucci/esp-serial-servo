//------------------------------------------------------------------------
// Filename  :  servo_packet.h
// Project   :  servokit
//
// Description:
//    ServoPacket: abstract frame buffer that a protocol family specializes
//    (for example STSeriesPacket). A packet is a view over a caller-owned
//    byte buffer, so it never allocates.
//
//    Contract common to every method:
//      Blocking   None. Packets do no I/O.
//      Threads    Not thread-safe.
//      Lifetimes  The buffer given to the constructor is borrowed and must
//                 outlive the packet.
//      Errors     Returned as srv_stat_t status codes (servo_utils.h).
//------------------------------------------------------------------------

#ifndef SERVO_PACKET__H
#define SERVO_PACKET__H
#include <cstdint>
#include <span>

#include "servokit/core/servo_utils.h"

namespace servokit {

/** Abstract protocol frame stored in a caller-supplied buffer. A frame is
 *  laid out as sync header, then the "data" region (everything after the sync
 *  header, including device ID, length, code, payload and checksum for the
 *  families that have them). Derived classes define the layout. */
class ServoPacket {
 public:
  /** Tag for log output. */
  static constexpr char LOG_TAG[] = "ServoPacket";

  ServoPacket() = delete;
  /** Binds the packet to @p buffer. The packet starts with size 0; the
   *  buffer is borrowed and must outlive the packet. */
  ServoPacket(const buffer_t& buffer);

  /** @return A view of the sync header at the start of the packet buffer. */
  virtual buffer_t sync_buffer() = 0;
  /** @return A view of the packet buffer after the sync header (the region
   *  a response is read into). */
  virtual buffer_t data_buffer() = 0;
  /** @return The index of the length byte within data_buffer(). */
  virtual uint8_t len_index() = 0;

  /** Determines if the packet is valid
   * @return true if the packet is well formed and internally consistent
   */
  virtual bool is_well_formed()=0;

  /** Frame-level validity: size, sync and length, but NOT the checksum.
   *  Default: same as is_well_formed() (families without a separate checksum). */
  virtual bool is_frame_valid() { return is_well_formed(); }

  /** True if the stored checksum matches the content. Default: true (no checksum). */
  virtual bool is_checksum_valid() { return true; }

  /** Device ID carried by the packet, or -1 if the family has none. */
  virtual int get_packet_device_id() { return -1; }

  /** Resets the packet to a known state containing only the sync header
   * @return SRV_OK always.
   */
  virtual srv_stat_t reset()=0;

  /** Updates the size of the stored packet. does not alter any bytes
   * @return SRV_OK on success.
   * @return SRV_OVERFLOW if size > buffer capacity
   */
  virtual srv_stat_t resize(size_t size);

  /** @return The maximum capacity of the raw buffer, in bytes. */
  virtual size_t get_capacity();

  /** @return A view of the whole raw buffer (capacity bytes, not just the packet). */
  virtual buffer_t get_raw_buffer();

  /** @return The actual size of the packet, in bytes. */
  virtual size_t get_size();

  /** @return A view of the packet (the first get_size() bytes) as the bytes to transmit. */
  virtual buffer_const_t as_bytes();

 protected:
  buffer_t m_buffer;  ///< The borrowed raw buffer.
  size_t m_size;      ///< Bytes of the buffer currently part of the packet.
};

}  // namespace servokit
#endif  // SERVO_PACKET__H
