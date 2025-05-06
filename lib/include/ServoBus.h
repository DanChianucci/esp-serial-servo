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

#include <freertos/FreeRTOS.h>
#include <hal/gpio_types.h>
#include <hal/uart_types.h>

#include <span>
#include <vector>

class ServoBus {
 private:
  static constexpr const char* LOG_TAG = "ServoBus";
  bool m_initialized;  // True when the Bus has been initialized.

  const uart_port_t m_uart_num;  // The UART peripheral id
  const gpio_num_t m_tx_pin;     // The TX GPIO Pin
  const gpio_num_t m_rx_pin;     // The RX GPIO Pin

  const int m_baud_rate;         // Baud Rate
  const int m_rx_buffer_size;    // RX Buffer Size
  const int m_tx_buffer_size;    // TX Buffer Size
  const int m_intr_alloc_flags;  // UART INTR Alloc Flags

  const gpio_mode_t   m_tx_mode;  // TX Pin Direction / Mode
  const gpio_pullup_t m_tx_pu;    // TX Pullup Enabled
  const gpio_pullup_t m_rx_pu;    // RX Pullup Enabled

 public:
  ServoBus(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin,
           int baud_rate = 115200, int rx_buffer_size = -1,
           int tx_buffer_size = 0, int intr_alloc_flags = 0,
           gpio_mode_t tx_mode = GPIO_MODE_OUTPUT_OD,
           gpio_pullup_t tx_pu = GPIO_PULLUP_ENABLE,
           gpio_pullup_t rx_pu = GPIO_PULLUP_ENABLE);

  int initialize();
  int close();


  int enable_rx(bool enable);

  int flush_output(TickType_t timeout);

  // Write bytes to the bus, block until the bytes are written or timeout expires.
  //@param data Pointer to the buffer containing the data to be written.
  //@param size Number of bytes to write.
  //@param timeout Timeout in ticks or 0 for non-blocking
  //@return The number of bytes written or ESP_FAIL on error.
  int write_bytes(const uint8_t * const data, size_t size, TickType_t timeout=0);

  // Write bytes to the bus, block until the bytes are written or timeout expires.
  //@param begin Pointer/iterator to the start of the buffer containing the data to be written.
  //@param end   Pointer/iterator to the end of the buffer containing the data to be written.
  //@param timeout Timeout in ticks or 0 for non-blocking
  //@return The number of bytes written or ESP_FAIL on error.
  template<std::contiguous_iterator T>
  int write_bytes(T begin, T end, TickType_t timeout=0) {
    return write_bytes(&*begin, std::distance(begin, end), timeout);
  }

  // Write bytes to the bus, block until the bytes are written or timeout expires.
  //@param data Contiguous range containing the data to be written.
  //@param timeout Timeout in ticks or 0 for non-blocking
  //@return The number of bytes written or ESP_FAIL on error.
  template<std::ranges::contiguous_range T>
  int write_bytes(const T& data, TickType_t timeout) {
    return write_bytes(std::ranges::begin(data),std::ranges::end(data), timeout);
  }

  int discard_input();

  // Read bytes from the bus, block until the sync pattern is found or timeout expires.
  //@param data buffercontaining the the sync pattern
  //@param timeout Timeout in ticks to wait for the sync operation to complete.
  //@return ESP_OK on success, ESP_FAIL on error.
  int read_sync(const std::vector<uint8_t> & data, TickType_t timeout);

  // Read bytes from the bus, block until the bytes are read or timeout expires.
  //@param data Pointer to the buffer where the read data will be stored.
  //@param size Number of bytes to read.
  //@param timeout Timeout in ticks to wait for the read operation to complete.
  //@return The number of bytes read or ESP_FAIL on error.
  int read_bytes(uint8_t *data, size_t size, TickType_t timeout);

  //Read bytes from the bus, block until the bytes are read or timeout expires.
  //@param begin Pointer/iterator to the start of the buffer where the read data will be stored.
  //@param end   Pointer/iterator to the end of the buffer where the read data will be stored.
  //@param timeout Timeout in ticks to wait for the read operation to complete.
  //@return The number of bytes read or ESP_FAIL on error.
  template<std::contiguous_iterator T>
  int read_bytes(T begin, T end, TickType_t timeout) {
    return read_bytes(&*begin, std::distance(begin, end), timeout);
  }

  // Read bytes from the bus, block until the bytes are read or timeout expires.
  //@param data Contiguous range where the read data will be stored.
  //@param timeout Timeout in ticks to wait for the read operation to complete.
  //@return The number of bytes read or ESP_FAIL on error.
  template<std::ranges::contiguous_range T>
  int read_bytes(T& data, TickType_t timeout) {
    return read_bytes(std::ranges::begin(data), std::ranges::end(data), timeout);
  }


  ~ServoBus();

  uart_port_t get_uart_num() { return m_uart_num; }
  gpio_num_t get_tx_pin() { return m_tx_pin; }
  gpio_num_t get_rx_pin() { return m_rx_pin; }
  int get_baud_rate() { return m_baud_rate; }
  int get_rx_buffer_size() { return m_rx_buffer_size; }
  int get_tx_buffer_size() { return m_tx_buffer_size; }
  int get_intr_alloc_flags() { return m_intr_alloc_flags; }
  gpio_mode_t get_tx_mode() { return m_tx_mode; }
  gpio_pullup_t get_tx_pu() { return m_tx_pu; }
  gpio_pullup_t get_rx_pu() { return m_rx_pu; }

 protected:
  int initialize_uart();
  int initialize_gpio();

  struct SyncState {
    const std::vector<uint8_t> pattern;
    uint8_t match_index;
    size_t read_bytes;
  };

  bool read_sync_raw(SyncState& sync_statem, TickType_t timeout);
};

#endif  // SERVO_BUS__H