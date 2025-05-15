#include "serial_servo/platform/esp_idf.h"

#include <driver/gpio.h>
#include <driver/uart.h>
#include <soc/gpio_periph.h>
#include <soc/uart_periph.h>

#include "serial_servo/base/servo_internal.h"

namespace esp_serial_servo {
using namespace timeout;

int def_rx_buffer_size(int uart_num, int rx_buffer_size) {
  if (rx_buffer_size <= 0)
    return UART_HW_FIFO_LEN(uart_num) + 1;
  else
    return rx_buffer_size;
}

gpio_mode_t def_tx_gpio_mode(int tx_pin, int rx_pin, bool od) {
  int result = GPIO_MODE_DEF_OUTPUT;
  if (od) result |= GPIO_MODE_DEF_OD;
  if (tx_pin == rx_pin) result |= GPIO_MODE_DEF_INPUT;
  return (gpio_mode_t)result;
}

ESPServoBus::ESPServoBus(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud_rate, int rx_buffer_size,
                   int tx_buffer_size, int intr_alloc_flags, bool tx_od, bool tx_pu, bool rx_pu)
    : m_initialized(false),
      m_uart_num(uart_num),
      m_tx_pin(tx_pin),
      m_rx_pin(rx_pin),
      m_baud_rate(baud_rate),
      m_rx_buffer_size(def_rx_buffer_size(uart_num, rx_buffer_size)),
      m_tx_buffer_size(tx_buffer_size),
      m_intr_alloc_flags(intr_alloc_flags),
      m_tx_od(tx_od),
      m_tx_pu(tx_pu),
      m_rx_pu(rx_pu) {}

srv_stat_t ESPServoBus::initialize() {
  SRV_RETURN_IF(m_initialized, SRV_FAIL);
  SRV_RETURN_IF(uart_is_driver_installed(m_uart_num), SRV_FAIL);

  SRV_RETURN_ON_ERR(initialize_gpio());
  SRV_RETURN_ON_ERR(initialize_uart());

  m_initialized = true;
  return SRV_OK;
}

srv_stat_t ESPServoBus::initialize_uart() {
  uart_config_t uart_config = {.baud_rate = m_baud_rate,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                               .rx_flow_ctrl_thresh = 0,
                               .source_clk = UART_SCLK_DEFAULT,
                               .flags = {}
                              };

  // Configure the UART Peripheral
  SRV_RETURN_IF(uart_param_config(m_uart_num, &uart_config) != ESP_OK, SRV_PARAM_ERR);
  SRV_RETURN_IF(
      uart_driver_install(m_uart_num, m_rx_buffer_size, m_tx_buffer_size, 0, NULL, m_intr_alloc_flags) != ESP_OK,
      SRV_PARAM_ERR);

  return SRV_OK;
}

srv_stat_t ESPServoBus::initialize_gpio() {
  gpio_config_t tx_config = {.pin_bit_mask = BIT64(m_tx_pin),
                             .mode = def_tx_gpio_mode(m_tx_pin, m_rx_pin, m_tx_od),
                             .pull_up_en = m_tx_pu ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE};

  SRV_RETURN_IF(gpio_config(&tx_config) != ESP_OK, SRV_PARAM_ERR);
  SRV_RETURN_IF(gpio_set_level(m_tx_pin, 1) != ESP_OK, SRV_PARAM_ERR);

  if (m_tx_pin != m_rx_pin) {
    gpio_config_t rx_config = {.pin_bit_mask = BIT64(m_rx_pin),
                               .mode = GPIO_MODE_INPUT,
                               .pull_up_en = m_rx_pu ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
                               .pull_down_en = GPIO_PULLDOWN_DISABLE,
                               .intr_type = GPIO_INTR_DISABLE};
    SRV_RETURN_IF(gpio_config(&rx_config) != ESP_OK, SRV_PARAM_ERR);
  }

  uint32_t tx_signal_id = UART_PERIPH_SIGNAL(m_uart_num, SOC_UART_TX_PIN_IDX);
  esp_rom_gpio_connect_out_signal(m_tx_pin, tx_signal_id, 0, 0);  // GPIO->UART

  uint32_t rx_signal_id = UART_PERIPH_SIGNAL(m_uart_num, SOC_UART_RX_PIN_IDX);
  esp_rom_gpio_connect_in_signal(m_rx_pin, rx_signal_id, 0);  // GPIO->UART

  return SRV_OK;
}

srv_stat_t ESPServoBus::close() {
  if (m_initialized) {
    m_initialized = false;
    SRV_RETURN_IF(uart_driver_delete(m_uart_num) != ESP_OK, SRV_PARAM_ERR);
    SRV_RETURN_IF(gpio_reset_pin(m_tx_pin) != ESP_OK, SRV_PARAM_ERR);
    SRV_RETURN_IF(gpio_reset_pin(m_rx_pin) != ESP_OK, SRV_PARAM_ERR);
  }
  return SRV_OK;
}

srv_stat_t ESPServoBus::enable_rx(bool enable) {
  if (enable) {
    SRV_RETURN_IF(uart_enable_rx_intr(m_uart_num) != ESP_OK, SRV_PARAM_ERR);
  } else {
    SRV_RETURN_IF(uart_disable_rx_intr(m_uart_num) != ESP_OK, SRV_PARAM_ERR);
  }
  return SRV_OK;
}

srv_stat_t ESPServoBus::discard_input() {
  SRV_RETURN_IF(uart_flush_input(m_uart_num) != ESP_OK, SRV_PARAM_ERR);
  return SRV_OK;
}

srv_stat_t ESPServoBus::flush_output(timeout_duration_t timeout) {
  int ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
  int result = uart_wait_tx_done(m_uart_num, pdMS_TO_TICKS(ms));

  SRV_RETURN_IF(result == ESP_ERR_TIMEOUT, SRV_TIMEOUT);
  SRV_RETURN_IF(result != ESP_OK, SRV_FAIL);
  return SRV_OK;
}

srv_stat_t ESPServoBus::read_sync(const buffer_const_t pattern, timeout_duration_t timeout) {
  size_t match_index = 0;
  int bytes_read = 0;
  timeout_duration_t remaining = timeout;

  timeout_ctrl_t timeout_ctrl;
  init_timeout_ctrl(&timeout_ctrl);

  uint8_t byte[1];
  while (match_index < pattern.size()) {
    srv_result_t result = read_bytes(buffer_t(byte), remaining);  // TODO do I need to check for error
    SRV_RETURN_ON_ERR(result);
    int len = get_value(result);
    bytes_read += len;
    if (len > 0) {
      if (*byte == pattern[match_index])
        match_index++;
      else
        match_index = 0;
    }
    bool timed_out = update_timeout(&timeout_ctrl, &remaining);
    SRV_RETURN_IF(timed_out, SRV_TIMEOUT);
  }
  return SRV_OK;
}

srv_result_t ESPServoBus::read_bytes(const buffer_t data, timeout_duration_t timeout) {
  uint32_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
  int result = uart_read_bytes(m_uart_num, data.data(), data.size(), pdMS_TO_TICKS(ms));
  SRV_RETURN_IF(result == ESP_FAIL, SRV_READ_FAIL);
  return result;
}

srv_result_t ESPServoBus::write_bytes(const buffer_const_t data, timeout_duration_t timeout) {
  int result = uart_write_bytes(m_uart_num, data.data(), data.size());
  SRV_RETURN_IF(result == ESP_FAIL, SRV_WRITE_FAIL);

  if (timeout > timeout_duration_t::zero()) {
    SRV_RETURN_ON_ERR(flush_output(timeout));
  }
  return result;
}

ESPServoBus::~ESPServoBus() { close(); }

}  // namespace esp_serial_servo