


//------------------------------------------------------------------------
//                  Copyright 2024 - Dan Chianucci
//------------------------------------------------------------------------
//       Use of this source code is governed by an MIT-style
//       license that can be found in the LICENSE file or at
//              https://opensource.org/licenses/MIT
//------------------------------------------------------------------------
//
// Filename  :  servo_bus.c
// Project   :  ESP LX16A SERVO
// Author    :  Dan Chianucci <dan.chianucci@gmail.com>
// Created   :  September 28 2024
//
// Description:
//
//
//------------------------------------------------------------------------

#include "ServoBus.h"

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_check.h>
#include <esp_log.h>
#include <soc/gpio_periph.h>
#include <soc/uart_periph.h>


int default_rx_buffer_size(int uart_num, int rx_buffer_size) {
  if (rx_buffer_size <= 0)
    return UART_HW_FIFO_LEN(uart_num) + 1;
  else
    return rx_buffer_size;
}

ServoBus::ServoBus(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin,
                   int baud_rate, int rx_buffer_size, int tx_buffer_size,
                   int intr_alloc_flags, gpio_mode_t tx_mode,
                   gpio_pullup_t tx_pu, gpio_pullup_t rx_pu)
    : m_initialized(false),
      m_uart_num(uart_num),
      m_tx_pin(tx_pin),
      m_rx_pin(rx_pin),
      m_baud_rate(baud_rate),
      m_rx_buffer_size(default_rx_buffer_size(uart_num, rx_buffer_size)),
      m_tx_buffer_size(tx_buffer_size),
      m_intr_alloc_flags(intr_alloc_flags),
      m_tx_mode(tx_pin!=rx_pin ? tx_mode : (gpio_mode_t)(tx_mode|GPIO_MODE_INPUT) ),
      m_tx_pu(tx_pu),
      m_rx_pu(rx_pu) {}

int ServoBus::initialize() {
  ESP_RETURN_ON_ERROR(uart_is_driver_installed(m_uart_num), LOG_TAG,
                      "Uart Driver already installed on m_uart_num");
  ESP_RETURN_ON_FALSE(!m_initialized, ESP_FAIL, LOG_TAG,
                      "ServoBus has already been initialized");

  initialize_gpio();
  initialize_uart();

  m_initialized = true;
  return ESP_OK;
}

int ServoBus::initialize_uart() {
  if (m_initialized) return ESP_FAIL;
  uart_config_t uart_config = {.baud_rate = m_baud_rate,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                               .rx_flow_ctrl_thresh = 0,
                               .source_clk = UART_SCLK_DEFAULT};

  // Configure the UART Peripheral
  ESP_ERROR_CHECK(uart_param_config(m_uart_num, &uart_config));
  ESP_ERROR_CHECK(uart_driver_install(m_uart_num, m_rx_buffer_size,
                                      m_tx_buffer_size, 0, NULL,
                                      m_intr_alloc_flags));
  return ESP_OK;
}

int ServoBus::initialize_gpio() {
  if (m_initialized) return ESP_FAIL;
  gpio_config_t tx_config = {
      .pin_bit_mask = BIT64(m_tx_pin), .mode = m_tx_mode, .pull_up_en = m_tx_pu,
      .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&tx_config);
  ESP_ERROR_CHECK(gpio_set_level(m_tx_pin, 1));

  if (m_tx_pin != m_rx_pin){
    gpio_config_t rx_config = {
        .pin_bit_mask = BIT64(m_rx_pin), .mode = GPIO_MODE_INPUT, .pull_up_en = m_rx_pu,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE };
    gpio_config(&rx_config);
  }

  uint32_t tx_signal_id = UART_PERIPH_SIGNAL(m_uart_num, SOC_UART_TX_PIN_IDX);
  esp_rom_gpio_connect_out_signal(m_tx_pin, tx_signal_id, 0, 0);  // GPIO->UART

  uint32_t rx_signal_id = UART_PERIPH_SIGNAL(m_uart_num, SOC_UART_RX_PIN_IDX);
  esp_rom_gpio_connect_in_signal(m_rx_pin, rx_signal_id, 0);   // GPIO->UART

  if(esp_log_level_get(LOG_TAG) >= ESP_LOG_DEBUG) {
    ESP_LOGD(LOG_TAG, "TX Pin: %d RX Pin: %d", m_tx_pin, m_rx_pin);
    gpio_dump_io_configuration(stdout, BIT64(m_tx_pin) | BIT64(m_rx_pin));
  }



  return ESP_OK;
}

int ServoBus::close() {
  if (m_initialized) {
    ESP_ERROR_CHECK(uart_driver_delete(m_uart_num));
    ESP_ERROR_CHECK(gpio_reset_pin(m_tx_pin));
    ESP_ERROR_CHECK(gpio_reset_pin(m_rx_pin));
    m_initialized = false;
  }
  return ESP_OK;
}

int ServoBus::enable_rx(bool enable) {
  if (enable)
    return uart_enable_rx_intr(m_uart_num);
  else
    return uart_disable_rx_intr(m_uart_num);
}



int ServoBus::flush_output(TickType_t timeout) {
  int result = uart_wait_tx_done(m_uart_num, timeout);
  if (result != ESP_OK) ESP_LOGE(LOG_TAG, "Failed to flush output");
  return result;
}

int ServoBus::discard_input() {
  int result = uart_flush_input(m_uart_num);
  if (result != ESP_OK) ESP_LOGE(LOG_TAG, "Failed to flush input");
  return result;
}



int ServoBus::read_bytes(uint8_t *data, size_t size, TickType_t timeout) {
  int result = uart_read_bytes(m_uart_num, data, size, timeout);

  ESP_LOGV(LOG_TAG, "read_bytes(%p, %u, %lu)", data, size, timeout);
  ESP_LOG_BUFFER_HEXDUMP(LOG_TAG, data, result, ESP_LOG_VERBOSE);

  ESP_RETURN_ON_FALSE(result >= 0, ESP_FAIL, LOG_TAG, "Failed to read bytes");
  return result;
}

int ServoBus::write_bytes(const uint8_t * const data, size_t size, TickType_t timeout) {
  ESP_LOGV(LOG_TAG, "write_bytes(%p, %u, %lu)", data, size, timeout);
  ESP_LOG_BUFFER_HEXDUMP(LOG_TAG, data, size, ESP_LOG_VERBOSE);

  int result = uart_write_bytes(m_uart_num, (const char *)data, size);
  ESP_RETURN_ON_FALSE(result>=0,ESP_FAIL, LOG_TAG, "Failed to write bytes");

  if(timeout> 0) {
    int wait_res = uart_wait_tx_done(m_uart_num, timeout);
    ESP_RETURN_ON_FALSE(wait_res==ESP_OK,ESP_FAIL, LOG_TAG, "Failed to wait for tx done");
  }
  return result;
}


bool ServoBus::read_sync_raw(SyncState& sync_state, TickType_t timeout) {
  uint8_t byte;
  int len = read_bytes(&byte, 1, timeout);
  sync_state.read_bytes += len;
  if (len > 0) {
    if (byte == sync_state.pattern[sync_state.match_index]) {
      sync_state.match_index++;
    } else {
      sync_state.match_index = 0;  // Reset if mismatch
    }
  }
  return sync_state.match_index >= sync_state.pattern.size();
}


int ServoBus::read_sync(const std::vector<uint8_t> & data, TickType_t timeout) {
  ESP_RETURN_ON_FALSE(data.size() != 0, ESP_FAIL, LOG_TAG, "Data size is zero");
  ESP_RETURN_ON_FALSE(timeout != 0, ESP_FAIL, LOG_TAG, "Timeout is zero");

  TimeOut_t xTimeOut;
  TickType_t ticks_to_wait = timeout;
  vTaskSetTimeOutState( &xTimeOut );
  SyncState sync_state = {.pattern=data, .match_index=0, .read_bytes=0};
  while(!read_sync_raw(sync_state, ticks_to_wait)) {
    bool timed_out = xTaskCheckForTimeOut(&xTimeOut, &ticks_to_wait) == pdTRUE;
    ESP_RETURN_ON_FALSE(!timed_out, ESP_FAIL, LOG_TAG, "Timeout waiting for sync pattern");
  }
  return ESP_OK;
}



ServoBus::~ServoBus() { this->close(); }