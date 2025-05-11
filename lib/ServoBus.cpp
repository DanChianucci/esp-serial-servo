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

ServoBus::ServoBus(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud_rate, int rx_buffer_size,
                   int tx_buffer_size, int intr_alloc_flags, gpio_mode_t tx_mode, gpio_pullup_t tx_pu,
                   gpio_pullup_t rx_pu)
    : m_initialized(false),
      m_uart_num(uart_num),
      m_tx_pin(tx_pin),
      m_rx_pin(rx_pin),
      m_baud_rate(baud_rate),
      m_rx_buffer_size(default_rx_buffer_size(uart_num, rx_buffer_size)),
      m_tx_buffer_size(tx_buffer_size),
      m_intr_alloc_flags(intr_alloc_flags),
      m_tx_mode(tx_pin != rx_pin ? tx_mode : (gpio_mode_t)(tx_mode | GPIO_MODE_INPUT)),
      m_tx_pu(tx_pu),
      m_rx_pu(rx_pu) {}

int ServoBus::initialize() {
  ESP_RETURN_ON_ERROR(uart_is_driver_installed(m_uart_num), LOG_TAG, "Uart Driver already installed on m_uart_num");
  ESP_RETURN_ON_FALSE(!m_initialized, ESP_FAIL, LOG_TAG, "ServoBus has already been initialized");

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
  ESP_ERROR_CHECK(uart_driver_install(m_uart_num, m_rx_buffer_size, m_tx_buffer_size, 0, NULL, m_intr_alloc_flags));
  return ESP_OK;
}

int ServoBus::initialize_gpio() {
  if (m_initialized) return ESP_FAIL;
  gpio_config_t tx_config = {.pin_bit_mask = BIT64(m_tx_pin),
                             .mode = m_tx_mode,
                             .pull_up_en = m_tx_pu,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&tx_config);
  ESP_ERROR_CHECK(gpio_set_level(m_tx_pin, 1));

  if (m_tx_pin != m_rx_pin) {
    gpio_config_t rx_config = {.pin_bit_mask = BIT64(m_rx_pin),
                               .mode = GPIO_MODE_INPUT,
                               .pull_up_en = m_rx_pu,
                               .pull_down_en = GPIO_PULLDOWN_DISABLE,
                               .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&rx_config);
  }

  uint32_t tx_signal_id = UART_PERIPH_SIGNAL(m_uart_num, SOC_UART_TX_PIN_IDX);
  esp_rom_gpio_connect_out_signal(m_tx_pin, tx_signal_id, 0, 0);  // GPIO->UART

  uint32_t rx_signal_id = UART_PERIPH_SIGNAL(m_uart_num, SOC_UART_RX_PIN_IDX);
  esp_rom_gpio_connect_in_signal(m_rx_pin, rx_signal_id, 0);  // GPIO->UART

  if (esp_log_level_get(LOG_TAG) >= ESP_LOG_DEBUG) {
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

int ServoBus::discard_input() {
  int result = uart_flush_input(m_uart_num);
  if (result != ESP_OK) ESP_LOGE(LOG_TAG, "Failed to flush input");
  return result;
}

int ServoBus::read_sync(const buffer_const_t pattern, timeout_duration_t timeout) {
  size_t match_index = 0;
  int bytes_read = 0;
  timeout_duration_t remaining=timeout;

  timeout_ctrl_t timeout_ctrl;
  init_timeout_ctrl(&timeout_ctrl);

  uint8_t byte[1];
  while (match_index < pattern.size()) {
    int len = read_bytes(buffer_t(byte), remaining);
    bytes_read += len;
    if (len > 0) {
      if (*byte == pattern[match_index])
        match_index++;
      else
        match_index = 0;
    }
    bool timed_out = update_timeout(&timeout_ctrl, &remaining);
    ESP_RETURN_ON_FALSE(!timed_out, ESP_FAIL, LOG_TAG, "Timeout waiting for sync pattern");
  }
  return ESP_OK;
}

int ServoBus::read_bytes(const buffer_t data, timeout_duration_t timeout) {
  uint32_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
  int result = uart_read_bytes(m_uart_num, data.data(), data.size(), pdMS_TO_TICKS(ms));
  ESP_RETURN_ON_FALSE(result >= 0, ESP_FAIL, LOG_TAG, "Failed to read bytes");
  return result;
}

int ServoBus::write_bytes(const buffer_const_t data, timeout_duration_t timeout) {
  int result = uart_write_bytes(m_uart_num, data.data(), data.size());
  ESP_RETURN_ON_FALSE(result >= 0, ESP_FAIL, LOG_TAG, "Failed to write bytes");

  if (timeout > timeout_duration_t::zero()) {
    int wait_res = flush_output(timeout);
    ESP_RETURN_ON_FALSE(wait_res == ESP_OK, ESP_FAIL, LOG_TAG, "Timeout waiting for tx flush");
  }
  return result;
}

int ServoBus::flush_output(timeout_duration_t timeout) {
  int ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
  int result = uart_wait_tx_done(m_uart_num, pdMS_TO_TICKS(ms));
  return result;
}

ServoBus::~ServoBus() { this->close(); }