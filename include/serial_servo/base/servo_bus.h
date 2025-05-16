//------------------------------------------------------------------------
//                  Copyright 2024 - Dan Chianucci
//------------------------------------------------------------------------
//       Use of this source code is governed by an MIT-style
//       license that can be found in the LICENSE file or at
//              https://opensource.org/licenses/MIT
//------------------------------------------------------------------------
//
// Filename  :  servo_bus.h
// Project   :  ESP LX16A Servo
// Author    :  Dan Chianucci <dan.chianucci@gmail.com>
// Created   :  September 28 2024
//
// Description:
//    Utilities to configure the ESP Uart for use with LX16A style SERVOS
//
//------------------------------------------------------------------------

#ifndef SERVO_BUS__H
#define SERVO_BUS__H

#include <cstdint>
#include <span>

#include "serial_servo/base/servo_utils.h"

namespace esp_serial_servo {
class ServoBus {
 public:
  static constexpr char LOG_TAG[] = "ServoBus";

  /** Enables/Disables the reciever
   *  @param enable True to enable, false to disable
   *  @return SRV_OK on success.
   *  @return SRV_PARAM_ERR on failure.
   */
  virtual srv_stat_t enable_rx(bool enable)=0;

  /** Discards all buffered rx data
   * @return SRV_OK on success.
   * @return SRV_PARAM_ERR on failure.
   */
  virtual srv_stat_t discard_input()=0;

  /** Block until all tx data has been transmitted
   *  @param timeout Timeout in ticks
   * @return SRV_OK on success.
   * @return SRV_TIMEOUT on timeout.
   */
  virtual srv_stat_t flush_output(timeout_duration_t timeout)=0;

  /** Read bytes from the bus, block until pattern is found or timeout expires.
   *  @param pattern buffer containing the the sync pattern
   *  @param timeout Timeout in ticks to wait for the sync operation to complete.
   *  @return SRV_OK on success.
   *  @return SRV_TIMEOUT if sync not found.
   */
  virtual srv_stat_t read_sync(const buffer_const_t pattern, timeout_duration_t timeout)=0;

  /** Read bytes from the bus, block until the bytes are read or timeout expires.
   *  @param data The buffer where the read data will be stored.
   *  @param timeout Timeout in ticks to wait for the read operation to complete.
   *  @return The number of bytes read.
   *  @return SRV_READ_FAIL on error.
   */
  virtual srv_result_t read_bytes(const buffer_t data, timeout_duration_t timeout)=0;

  /** Write bytes to the bus, block until the bytes are written or timeout expires.
   *  @param data buffer to write.
   *  @param timeout Timeout in ticks or 0 for non-blocking
   *  @return The number of bytes written.
   *  @return SRV_WRITE_FAIL on error.
   *  @return SRV_TIMEOUT on timeout.
   */
  virtual srv_result_t write_bytes(const buffer_const_t, timeout_duration_t timeout = timeout_duration_t::max())=0;

};
}  // namespace esp_serial_servo
#endif  // SERVO_BUS__H