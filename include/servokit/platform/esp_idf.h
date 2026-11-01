#ifndef SERIALSERVO_PLATFORM_ESPIDF__H
#define SERIALSERVO_PLATFORM_ESPIDF__H

#include <hal/gpio_types.h>
#include <hal/uart_types.h>

#include "servokit/serial/serial_port.h"

namespace servokit {
class ESPSerialPort : public SerialPort {
 public:
  ESPSerialPort(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud_rate = 115200,
              int rx_buffer_size = -1, int tx_buffer_size = 0, int intr_alloc_flags = 0, bool tx_od = true,
              bool tx_pu = true, bool rx_pu = true);

  /** Sets up GPIO, installs UART Driver
   *  @return SRV_OK on success.
   *  @return SRV_PARAM_ERR on parameter errors.
   *  @return SRV_FAIL if already initialized or uart is already installed.
   */
  srv_stat_t initialize();

  /** Resets GPIO and uninstall uart driver
   * @return SRV_OK on success
   * @return SRV_PARAM_ERR on parameter error
   */
  srv_stat_t close();

  /** Enables/Disables the reciever
   *  @param enable True to enable, false to disable
   *  @return SRV_OK on success.
   *  @return SRV_PARAM_ERR on failure.
   */
  srv_stat_t enable_rx(bool enable);

  /** Discards all buffered rx data
   * @return SRV_OK on success.
   * @return SRV_PARAM_ERR on failure.
   */
  srv_stat_t discard_input();

  /** Block until all tx data has been transmitted.
   *  @param timeout Longest time to wait, as a duration. Zero or negative
   *         still checks once without blocking.
   *  @return SRV_OK when transmission is complete.
   *  @return SRV_TIMEOUT if it is not complete when the time has passed.
   *  @return SRV_FAIL on a driver error.
   */
  srv_stat_t flush_output(timeout_duration_t timeout);

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
  srv_stat_t read_sync(const buffer_const_t pattern, timeout_duration_t timeout);

  /** Read bytes from the bus, block until the buffer is full or the time
   *  expires.
   *  @param data The buffer where the read data will be stored.
   *  @param timeout Longest total time to wait, as a duration. Zero or
   *         negative still makes one non-blocking read attempt.
   *  @return The number of bytes read (fewer than requested, possibly 0, if
   *          the time expired first).
   *  @return SRV_READ_FAIL on error.
   */
  srv_result_t read_bytes(const buffer_t data, timeout_duration_t timeout);

  /** Write bytes to the bus.
   *  The bytes are first handed to the UART driver with uart_write_bytes(),
   *  which has no timeout; that step is not bounded by @p timeout (with a TX
   *  buffer size of 0 it returns once the bytes are in the UART FIFO, so it
   *  can block for the duration of the frame). The deadline bounds only the
   *  wait for transmission to finish, which happens only when @p timeout is
   *  positive; the deadline starts when the call begins.
   *  @param data buffer to write.
   *  @param timeout Longest time to wait for transmission to finish, as a
   *         duration. Zero or negative returns right after the hand-off
   *         without waiting for transmission.
   *  @return The number of bytes handed to the driver.
   *  @return SRV_WRITE_FAIL on error.
   *  @return SRV_TIMEOUT if transmission is not done in time (the bytes
   *          remain queued and are still sent).
   *  @return SRV_FAIL on a driver error while waiting.
   */
  srv_result_t write_bytes(const buffer_const_t, timeout_duration_t timeout = timeout_duration_t::max());

  ~ESPSerialPort();

  uart_port_t get_uart_num() { return m_uart_num; }
  gpio_num_t get_tx_pin() { return m_tx_pin; }
  gpio_num_t get_rx_pin() { return m_rx_pin; }
  int get_baud_rate() { return m_baud_rate; }
  int get_rx_buffer_size() { return m_rx_buffer_size; }
  int get_tx_buffer_size() { return m_tx_buffer_size; }
  int get_intr_alloc_flags() { return m_intr_alloc_flags; }
  bool get_tx_od() { return m_tx_od; }
  bool get_tx_pu() { return m_tx_pu; }
  bool get_rx_pu() { return m_rx_pu; }

 protected:
  srv_stat_t initialize_uart();
  srv_stat_t initialize_gpio();

  bool m_initialized;  //!< @brief True when the Bus has been initialized.

  const uart_port_t m_uart_num;  //!< The UART peripheral id
  const gpio_num_t m_tx_pin;     //!< The TX GPIO Pin
  const gpio_num_t m_rx_pin;     //!< The RX GPIO Pin

  const int m_baud_rate;         //!< Baud Rate
  const int m_rx_buffer_size;    //!< RX Buffer Size
  const int m_tx_buffer_size;    //!< TX Buffer Size
  const int m_intr_alloc_flags;  //!< UART INTR Alloc Flags
  const int m_tx_od;             //!< TX Pin Direction / Mode

  const bool m_tx_pu;  //!< TX Pullup Enabled
  const bool m_rx_pu;  //!< RX Pullup Enabled
};

}  // namespace servokit

#endif  // SERIALSERVO_PLATFORM_ESPIDF__H