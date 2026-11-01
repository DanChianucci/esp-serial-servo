// Host-side fake SerialPort for unit tests. No ESP-IDF/Arduino dependency.
#ifndef FAKE_SERIAL_PORT__H
#define FAKE_SERIAL_PORT__H

#include <algorithm>
#include <cstdint>
#include <deque>
#include <vector>

#include "servokit/serial/serial_port.h"

namespace servokit_test {

using servokit::buffer_const_t;
using servokit::buffer_t;
using servokit::srv_result_t;
using servokit::srv_stat_t;
using servokit::timeout_duration_t;
using Bytes = std::vector<uint8_t>;

// Scripted fake of the serial bus.
//  - Every write_bytes() call is captured in tx_log() (one entry per call).
//  - Each write consumes the next scripted reply (if any) and makes its bytes
//    available to the reader. Replies may be well-formed frames, malformed
//    frames, wrong-ID frames (just build the bytes) or empty (= no answer,
//    so reads time out).
//  - add_stale_rx() pre-loads bytes that were on the bus before a command.
//    discard_input() drops everything currently buffered.
//  - Reads never block: if fewer bytes than requested are buffered, the
//    partial count is returned (0 when empty), modelling an expired timeout.
class FakeSerialPort : public servokit::SerialPort {
 public:
  // --- scripting ---
  void script_reply(Bytes reply) { m_replies.push_back(std::move(reply)); }
  void script_timeout() { m_replies.emplace_back(); }
  void add_stale_rx(const Bytes& bytes) { m_rx.insert(m_rx.end(), bytes.begin(), bytes.end()); }
  void fail_next_write() { m_fail_write = true; }
  void fail_next_read() { m_fail_read = true; }

  // --- inspection ---
  const std::vector<Bytes>& tx_log() const { return m_tx; }
  size_t rx_pending() const { return m_rx.size(); }
  bool rx_enabled() const { return m_rx_enabled; }
  int discard_count() const { return m_discards; }
  int flush_count() const { return m_flushes; }
  timeout_duration_t last_write_timeout() const { return m_last_write_timeout; }

  // --- SerialPort ---
  servokit::srv_stat_t enable_rx(bool enable) override {
    m_rx_enabled = enable;
    return servokit::SRV_OK;
  }

  servokit::srv_stat_t discard_input() override {
    m_rx.clear();
    ++m_discards;
    return servokit::SRV_OK;
  }

  servokit::srv_stat_t flush_output(timeout_duration_t) override {
    ++m_flushes;
    return servokit::SRV_OK;
  }

  servokit::srv_stat_t read_sync(const buffer_const_t pattern, timeout_duration_t) override {
    // Mirrors the ESP implementation: consume bytes up to and including the pattern.
    size_t match = 0;
    while (match < pattern.size()) {
      if (m_rx.empty()) return servokit::SRV_TIMEOUT;
      uint8_t b = m_rx.front();
      m_rx.pop_front();
      match = (b == pattern[match]) ? match + 1 : 0;
    }
    return servokit::SRV_OK;
  }

  servokit::srv_result_t read_bytes(const buffer_t data, timeout_duration_t) override {
    if (m_fail_read) {
      m_fail_read = false;
      return servokit::srv_result_t(std::unexpected(servokit::SRV_READ_FAIL.error()));
    }
    size_t n = std::min(data.size(), m_rx.size());
    for (size_t i = 0; i < n; ++i) {
      data[i] = m_rx.front();
      m_rx.pop_front();
    }
    return static_cast<int>(n);
  }

  servokit::srv_result_t write_bytes(const buffer_const_t data, timeout_duration_t timeout) override {
    m_last_write_timeout = timeout;
    if (m_fail_write) {
      m_fail_write = false;
      return servokit::srv_result_t(std::unexpected(servokit::SRV_WRITE_FAIL.error()));
    }
    m_tx.emplace_back(data.begin(), data.end());
    if (!m_replies.empty()) {
      const Bytes& r = m_replies.front();
      m_rx.insert(m_rx.end(), r.begin(), r.end());
      m_replies.pop_front();
    }
    return static_cast<int>(data.size());
  }

 private:
  std::vector<Bytes> m_tx;
  std::deque<Bytes> m_replies;
  std::deque<uint8_t> m_rx;
  bool m_rx_enabled = true;
  bool m_fail_write = false;
  bool m_fail_read = false;
  int m_discards = 0;
  int m_flushes = 0;
  timeout_duration_t m_last_write_timeout = timeout_duration_t::zero();
};

}  // namespace servokit_test
#endif  // FAKE_SERIAL_PORT__H
