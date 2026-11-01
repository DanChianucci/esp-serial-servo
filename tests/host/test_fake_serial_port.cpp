// Smoke tests for FakeSerialPort. Minimal self-contained harness.
#include <cstdio>

#include "fake_serial_port.h"

static int g_failures = 0;
#define CHECK(cond)                                                  \
  do {                                                               \
    if (!(cond)) {                                                   \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
      ++g_failures;                                                  \
    }                                                                \
  } while (0)

using namespace servokit_test;
using std::chrono::milliseconds;

static void test_command_and_scripted_reply() {
  FakeSerialPort port;
  const Bytes cmd = {0x55, 0x55, 0x01, 0x03, 0x0E, 0xED};
  const Bytes reply = {0x55, 0x55, 0x01, 0x05, 0x0E, 0x10, 0x00, 0xDB};
  port.script_reply(reply);

  auto w = port.write_bytes(cmd, milliseconds(10));
  CHECK(w.has_value() && *w == (int)cmd.size());
  CHECK(port.tx_log().size() == 1);
  CHECK(port.tx_log()[0] == cmd);

  const Bytes sync = {0x55, 0x55};
  CHECK(port.read_sync(sync, milliseconds(10)).error() == servokit::SRV_OK.error());
  Bytes rest(6);
  auto r = port.read_bytes(rest, milliseconds(10));
  CHECK(r.has_value() && *r == 6);
  CHECK(rest == Bytes(reply.begin() + 2, reply.end()));
}

static void test_timeout_and_stale() {
  FakeSerialPort port;
  port.add_stale_rx({0xAA, 0xBB});
  CHECK(port.rx_pending() == 2);
  CHECK(port.discard_input().error() == servokit::SRV_OK.error());
  CHECK(port.rx_pending() == 0);

  port.script_timeout();
  CHECK(port.write_bytes(Bytes{0x01}, milliseconds(1)).has_value());
  Bytes buf(4);
  auto r = port.read_bytes(buf, milliseconds(1));
  CHECK(r.has_value() && *r == 0);
  const Bytes sync = {0x55, 0x55};
  CHECK(port.read_sync(sync, milliseconds(1)).error() == servokit::SRV_TIMEOUT.error());
}

int main() {
  test_command_and_scripted_reply();
  test_timeout_and_stale();
  if (g_failures) {
    std::printf("%d failure(s)\n", g_failures);
    return 1;
  }
  std::printf("all tests passed\n");
  return 0;
}
