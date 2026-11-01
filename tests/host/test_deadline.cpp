// Tests for servokit::timeout::BasicDeadline and the tick conversion helpers,
// using an injected fake clock.
#include <cstdint>
#include <cstdio>

#include "servokit/serial/deadline.h"

static int g_failures = 0;
#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                               \
    }                                                             \
  } while (0)

using namespace std::chrono;
using namespace servokit::timeout;

struct FakeClock {
  using duration = nanoseconds;
  using rep = duration::rep;
  using period = duration::period;
  using time_point = std::chrono::time_point<FakeClock>;
  static inline time_point current{};
  static time_point now() { return current; }
  static void set(duration d) { current = time_point(d); }
};
using FakeDeadline = BasicDeadline<FakeClock>;

static void test_basic() {
  FakeClock::set(seconds(100));
  FakeDeadline d(milliseconds(50));
  CHECK(!d.expired());
  CHECK(d.remaining() == milliseconds(50));
  FakeClock::set(seconds(100) + milliseconds(20));
  CHECK(!d.expired());
  CHECK(d.remaining() == milliseconds(30));
  FakeClock::set(seconds(100) + milliseconds(50));
  CHECK(d.expired());
  CHECK(d.remaining() == nanoseconds::zero());
  FakeClock::set(seconds(200));
  CHECK(d.expired());
  CHECK(d.remaining() == nanoseconds::zero());
}

static void test_zero_and_negative() {
  FakeClock::set(seconds(5));
  FakeDeadline z(nanoseconds::zero());
  CHECK(z.expired());
  CHECK(z.remaining() == nanoseconds::zero());
  FakeDeadline n(milliseconds(-10));
  CHECK(n.expired());
  CHECK(n.remaining() == nanoseconds::zero());
}

static void test_saturation() {
  FakeClock::set(hours(1));
  FakeDeadline huge(nanoseconds::max());
  CHECK(!huge.expired());
  CHECK(huge.remaining() > hours(1000));
  CHECK(huge.remaining() >= nanoseconds::zero());
  FakeClock::set(hours(24 * 365));
  CHECK(!huge.expired());

  FakeClock::set(nanoseconds::max() - seconds(1));
  FakeDeadline edge(seconds(10));
  CHECK(!edge.expired());
  CHECK(edge.remaining() == seconds(1));
}

static void test_real_clock() {
  Deadline zero{steady_clock::duration::zero()};
  CHECK(zero.expired());
  Deadline later(hours(1));
  CHECK(!later.expired());
  CHECK(later.remaining() > minutes(59));
  Deadline huge(steady_clock::duration::max());
  CHECK(!huge.expired());
  Deadline soon(milliseconds(5));
  while (!soon.expired()) {
  }
  CHECK(soon.remaining() == steady_clock::duration::zero());
}

static void test_conversions() {
  CHECK(ceil_ms(nanoseconds(-5)) == 0);
  CHECK(ceil_ms(nanoseconds(0)) == 0);
  CHECK(ceil_ms(nanoseconds(1)) == 1);
  CHECK(ceil_ms(microseconds(1500)) == 2);
  CHECK(ceil_ms(milliseconds(7)) == 7);
  CHECK(ceil_ms(hours(24 * 365)) == UINT32_MAX);
  CHECK(ceil_ms(nanoseconds::max()) == UINT32_MAX);

  CHECK(ceil_ticks(0, 10, 1000) == 0);
  CHECK(ceil_ticks(1, 10, 1000) == 1);
  CHECK(ceil_ticks(10, 10, 1000) == 1);
  CHECK(ceil_ticks(11, 10, 1000) == 2);
  CHECK(ceil_ticks(25, 1, 1000) == 25);
  CHECK(ceil_ticks(UINT32_MAX, 1, 1000) == 1000);
  CHECK(ceil_ticks(UINT32_MAX, 10, UINT32_MAX - 1) == 429496730u);
  CHECK(ceil_ticks(5, 0, 100) == 5);
}

int main() {
  test_basic();
  test_zero_and_negative();
  test_saturation();
  test_real_clock();
  test_conversions();
  if (g_failures == 0) std::printf("test_deadline: all passed\n");
  return g_failures == 0 ? 0 : 1;
}
