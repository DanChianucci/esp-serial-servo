//------------------------------------------------------------------------
//                  Copyright 2024 - Dan Chianucci
//------------------------------------------------------------------------
//       Use of this source code is governed by an MIT-style
//       license that can be found in the LICENSE file or at
//              https://opensource.org/licenses/MIT
//------------------------------------------------------------------------
//
// Filename  :  serial_port.h
// Project   :  servokit
// Author    :  Dan Chianucci <dan.chianucci@gmail.com>
// Created   :  September 28 2024
//
// Description:
//    SerialPort: the byte-transport interface under ServoController. A
//    platform (a UART driver, a test fake) implements it; nothing above it
//    depends on the platform.
//
//    Contract common to every method:
//      Timeouts   Durations are std::chrono (timeout_duration_t), never RTOS
//                 ticks. A zero or negative timeout still makes one
//                 non-blocking attempt. A timeout too large to represent
//                 never expires.
//      Blocking   Each method blocks for at most its timeout (the platform
//                 documents any step it cannot bound).
//      Threads    Not thread-safe. Serialize all calls on one port.
//      Lifetimes  Buffers are used only during the call and never retained.
//      Errors     Returned as srv_stat_t / srv_result_t status codes
//                 (servo_utils.h), never thrown.
//------------------------------------------------------------------------

#ifndef SERIAL_PORT__H
#define SERIAL_PORT__H

#include <cstdint>
#include <span>

#include "servokit/core/servo_utils.h"

namespace servokit {

/** Byte transport for a half- or full-duplex serial bus. All methods are
 *  pure virtual. The port is borrowed by ServoController and must outlive
 *  it. The class has no virtual destructor: never delete an implementation
 *  through a SerialPort pointer. See the file header for the contract common
 *  to every method. */
class SerialPort {
 public:
  /** Tag for log output. */
  static constexpr char LOG_TAG[] = "SerialPort";

  /** Enables or disables the receiver. Used to mute the receiver while
   *  transmitting on a single-wire bus so the port does not hear its own
   *  frame.
   *  @param enable True to enable, false to disable
   *  @return SRV_OK on success.
   *  @return SRV_PARAM_ERR on failure.
   */
  virtual srv_stat_t enable_rx(bool enable)=0;

  /** Discards all buffered rx data.
   * @return SRV_OK on success.
   * @return SRV_PARAM_ERR on failure.
   */
  virtual srv_stat_t discard_input()=0;

  /** Block until all tx data has been transmitted.
   *  @param timeout Longest time to wait, as a duration. Zero or negative
   *         still checks once without blocking.
   *  @return SRV_OK when transmission is complete.
   *  @return SRV_TIMEOUT if it is not complete when the time has passed.
   *  @return SRV_FAIL on a driver error.
   */
  virtual srv_stat_t flush_output(timeout_duration_t timeout)=0;

  /** Read bytes from the bus, block until the pattern is found or the time
   *  expires. Bytes before and including the pattern are consumed.
   *  @param pattern The sync pattern; at most SyncMatcher::MAX_PATTERN bytes.
   *  @param timeout Longest total time to wait, as a duration. Zero or
   *         negative still makes one non-blocking read attempt.
   *  @return SRV_OK when the pattern was read.
   *  @return SRV_TIMEOUT if it was not found in time.
   *  @return SRV_PARAM_ERR if the pattern is longer than the maximum.
   *  @return SRV_READ_FAIL on a driver error.
   */
  virtual srv_stat_t read_sync(const buffer_const_t pattern, timeout_duration_t timeout)=0;

  /** Read bytes from the bus, block until the buffer is full or the time
   *  expires.
   *  @param data The buffer where the read data will be stored.
   *  @param timeout Longest total time to wait, as a duration. Zero or
   *         negative still makes one non-blocking read attempt.
   *  @return The number of bytes read (fewer than requested, possibly 0, if
   *          the time expired first).
   *  @return SRV_READ_FAIL on error.
   */
  virtual srv_result_t read_bytes(const buffer_t data, timeout_duration_t timeout)=0;

  /** Write bytes to the bus. The first argument is the buffer to write; it
   *  is read during the call only.
   *  @param timeout Longest time to wait for the bytes to be transmitted, as
   *         a duration. Implementations document whether the hand-off of the
   *         bytes to the transport is also bounded by it. The default is the
   *         maximum duration (wait as long as it takes).
   *  @return The number of bytes accepted for transmission.
   *  @return SRV_WRITE_FAIL on error.
   *  @return SRV_TIMEOUT if transmission is not complete in time.
   */
  virtual srv_result_t write_bytes(const buffer_const_t, timeout_duration_t timeout = timeout_duration_t::max())=0;

};
}  // namespace servokit
#endif  // SERIAL_PORT__H
