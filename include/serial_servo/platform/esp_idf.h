#ifndef SERIALSERVO_PLATFORM_ESPIDF__H
#define SERIALSERVO_PLATFORM_ESPIDF__H

#include <hal/gpio_types.h>
#include <hal/uart_types.h>

#include "serial_servo/base/servo_bus.h"

namespace esp_serial_servo {
class ESPServoBus : public ServoBus {
 public:
  ESPServoBus(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud_rate = 115200,
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

  /** Block until all tx data has been transmitted
   *  @param timeout Timeout in ticks
   * @return SRV_OK on success.
   * @return SRV_TIMEOUT on timeout.
   */
  srv_stat_t flush_output(timeout_duration_t timeout);

  /** Read bytes from the bus, block until pattern is found or timeout expires.
   *  @param pattern buffer containing the the sync pattern
   *  @param timeout Timeout in ticks to wait for the sync operation to complete.
   *  @return SRV_OK on success.
   *  @return SRV_TIMEOUT if sync not found.
   */
  srv_stat_t read_sync(const buffer_const_t pattern, timeout_duration_t timeout);

  /** Read bytes from the bus, block until the bytes are read or timeout expires.
   *  @param data The buffer where the read data will be stored.
   *  @param timeout Timeout in ticks to wait for the read operation to complete.
   *  @return The number of bytes read.
   *  @return SRV_READ_FAIL on error.
   */
  srv_result_t read_bytes(const buffer_t data, timeout_duration_t timeout);

  /** Write bytes to the bus, block until the bytes are written or timeout expires.
   *  @param data buffer to write.
   *  @param timeout Timeout in ticks or 0 for non-blocking
   *  @return The number of bytes written.
   *  @return SRV_WRITE_FAIL on error.
   *  @return SRV_TIMEOUT on timeout.
   */
  srv_result_t write_bytes(const buffer_const_t, timeout_duration_t timeout = timeout_duration_t::max());

  ~ESPServoBus();

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

}  // namespace esp_serial_servo

#endif  // SERIALSERVO_PLATFORM_ESPIDF__H