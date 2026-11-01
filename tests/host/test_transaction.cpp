// Card 03: transaction hardening tests (flush, timeouts, checksum option, Result mapping).
#include <chrono>
#include <cstdio>

#include "fake_serial_port.h"
#include "servokit/serial/servo_controller.h"
#include "servokit/stseries/stseries_packet.h"

static int g_failures = 0;
#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                               \
    }                                                             \
  } while (0)

using namespace servokit;
using namespace servokit_test;
using std::chrono::milliseconds;

// Builds an STS status frame: FF FF id len err params... checksum.
static Bytes frame(uint8_t id, uint8_t err, Bytes params, bool good_checksum = true, int len_delta = 0) {
  Bytes f = {0xFF, 0xFF, id, (uint8_t)(params.size() + 2 + len_delta), err};
  f.insert(f.end(), params.begin(), params.end());
  unsigned sum = 0;
  for (size_t i = 2; i < f.size(); ++i) sum += f[i];
  uint8_t cs = (uint8_t)~sum;
  f.push_back(good_checksum ? cs : (uint8_t)(cs ^ 0x5A));
  return f;
}

struct Rig {
  FakeSerialPort port;
  ServoController ctl{&port, milliseconds(10), milliseconds(20)};
  uint8_t cbuf[32];
  uint8_t rbuf[32];
  STSeriesPacket cmd{buffer_t(cbuf)};
  STSeriesPacket rsp{buffer_t(rbuf)};
  Rig() { cmd.init(1, STS::CMD_PING, buffer_const_t()); }
  srv_result_t run() { return ctl.command_response(&cmd, &rsp); }
};

static void test_flush_and_stale() {
  Rig r;
  // Stale garbage including a partial frame and a fake sync pattern.
  r.port.add_stale_rx({0x12, 0xFF, 0xFF, 0x01, 0x09, 0x00});
  r.port.script_reply(frame(1, 0, {0x10, 0x20}));
  auto res = r.run();
  CHECK(res.has_value());
  CHECK(r.port.discard_count() == 1);
  CHECK(r.rsp.is_well_formed());
  CHECK(r.rsp.get_device_id() == 1);
  CHECK(r.rsp.get_data_byte(0) == 0x10 && r.rsp.get_data_byte(1) == 0x20);

  // Flush happens before every command, even with no reply scripted.
  r.port.add_stale_rx({0xFF, 0xFF});
  (void)r.ctl.send_command(&r.cmd);
  CHECK(r.port.discard_count() == 2);
  CHECK(r.port.rx_pending() == 0);
  CHECK(r.ctl.flush_rx().error() == SRV_OK.error());
  CHECK(r.port.discard_count() == 3);
}

static void test_timeouts() {
  Rig r;
  CHECK(r.ctl.transaction_timeout() == milliseconds(30));
  CHECK(r.ctl.set_transaction_timeout(milliseconds(0)).error() == SRV_PARAM_ERR.error());
  CHECK(r.ctl.transaction_timeout() == milliseconds(30));
  CHECK(r.ctl.set_transaction_timeout(milliseconds(50)).error() == SRV_OK.error());
  CHECK(r.ctl.transaction_timeout() == milliseconds(50));

  // No answer: finite TIMEOUT.
  r.port.script_timeout();
  auto res = r.run();
  CHECK(!res.has_value() && res.error() == SRV_TIMEOUT.error());
  CHECK(to_result(res) == Result::TIMEOUT);

  // Truncated reply (header only, body never arrives): TIMEOUT, not garbage.
  Bytes f = frame(1, 0, {1, 2, 3});
  f.resize(5);
  r.port.script_reply(f);
  res = r.run();
  CHECK(!res.has_value() && res.error() == SRV_TIMEOUT.error());

  // Malformed reply is distinguishable: length byte 0 -> BAD_FRAME.
  r.port.script_reply({0xFF, 0xFF, 0x01, 0x00});
  res = r.run();
  CHECK(!res.has_value() && res.error() == SRV_BAD_FRAME.error());
  CHECK(to_result(res) == Result::BAD_RESPONSE);

  // Length byte larger than the buffer -> BAD_FRAME, no overflow.
  r.port.script_reply({0xFF, 0xFF, 0x01, 0xF0});
  res = r.run();
  CHECK(!res.has_value() && res.error() == SRV_BAD_FRAME.error());

  // Write failure is a bus error.
  r.port.fail_next_write();
  res = r.run();
  CHECK(!res.has_value() && to_result(res) == Result::BUS);
}

static void test_checksum_option() {
  Rig r;
  CHECK(r.ctl.verify_checksum());
  CHECK(r.ctl.checksum_errors() == 0);

  // Default: bad checksum rejected and counted.
  r.port.script_reply(frame(1, 0, {7}, false));
  auto res = r.run();
  CHECK(!res.has_value() && res.error() == SRV_BAD_CHECKSUM.error());
  CHECK(to_result(res) == Result::BAD_RESPONSE);
  CHECK(r.ctl.checksum_errors() == 1);

  // Good frame: accepted, no count.
  r.port.script_reply(frame(1, 0, {7}));
  CHECK(r.run().has_value());
  CHECK(r.ctl.checksum_errors() == 1);

  // Disabled: accepted, still counted.
  r.ctl.set_verify_checksum(false);
  CHECK(!r.ctl.verify_checksum());
  r.port.script_reply(frame(1, 0, {7}, false));
  res = r.run();
  CHECK(res.has_value());
  CHECK(r.rsp.get_data_byte(0) == 7);
  CHECK(r.ctl.checksum_errors() == 2);

  // Disabled: bad ID still rejected.
  r.port.script_reply(frame(2, 0, {7}, false));
  res = r.run();
  CHECK(!res.has_value() && res.error() == SRV_BAD_ID.error());

  // Disabled: bad length still rejected (len byte 0, and length claiming more than was sent).
  r.port.script_reply({0xFF, 0xFF, 0x01, 0x00, 0x00});
  res = r.run();
  CHECK(!res.has_value() && res.error() == SRV_BAD_FRAME.error());
  r.port.script_reply(frame(1, 0, {7}, false, 1));
  res = r.run();
  CHECK(!res.has_value() && res.error() == SRV_TIMEOUT.error());

  // Disabled: bad sync is not accepted either.
  Bytes bad = frame(1, 0, {7});
  bad[0] = 0xAA;
  r.port.script_reply(bad);
  res = r.run();
  CHECK(!res.has_value());

  // Re-enable at runtime.
  r.ctl.set_verify_checksum(true);
  r.port.script_reply(frame(1, 0, {7}, false));
  CHECK(!r.run().has_value());

  // Transmitted frames always carry a correct checksum, in both modes.
  for (const Bytes& tx : r.port.tx_log()) {
    unsigned sum = 0;
    for (size_t i = 2; i + 1 < tx.size(); ++i) sum += tx[i];
    CHECK(tx.back() == (uint8_t)~sum);
  }
}

static void test_result_mapping() {
  CHECK(to_result(SRV_OK) == Result::OK);
  CHECK(to_result(SRV_TIMEOUT) == Result::TIMEOUT);
  CHECK(to_result(SRV_READ_FAIL) == Result::BUS);
  CHECK(to_result(SRV_WRITE_FAIL) == Result::BUS);
  CHECK(to_result(SRV_FAIL) == Result::BUS);
  CHECK(to_result(SRV_BAD_FRAME) == Result::BAD_RESPONSE);
  CHECK(to_result(SRV_BAD_CHECKSUM) == Result::BAD_RESPONSE);
  CHECK(to_result(SRV_BAD_ID) == Result::BAD_RESPONSE);
  CHECK(to_result(SRV_PARAM_ERR) == Result::PARAM);
  CHECK(to_result(SRV_OVERFLOW) == Result::PARAM);
  CHECK(to_result(-999) == Result::BUS);
  CHECK(to_result(srv_result_t(5)) == Result::OK);
  CHECK(to_result(srv_result_t(std::unexpected(SRV_TIMEOUT.error()))) == Result::TIMEOUT);
}

int main() {
  test_flush_and_stale();
  test_timeouts();
  test_checksum_option();
  test_result_mapping();
  if (g_failures) {
    std::printf("%d failure(s)\n", g_failures);
    return 1;
  }
  std::printf("test_transaction: all passed\n");
  return 0;
}
