// Tests for servokit::SyncMatcher.
#include <cstdint>
#include <cstdio>
#include <vector>

#include "servokit/serial/sync_matcher.h"

static int g_failures = 0;
#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                               \
    }                                                             \
  } while (0)

using servokit::SyncMatcher;
using Bytes = std::vector<uint8_t>;

// Number of bytes consumed when the pattern first completes, or 0 when it
// never does (or the pattern is rejected).
static size_t first_match(const Bytes& pattern, const Bytes& stream) {
  SyncMatcher m;
  if (!m.reset(pattern)) return 0;
  for (size_t i = 0; i < stream.size(); ++i)
    if (m.feed(stream[i])) return i + 1;
  return 0;
}

static void test_overlaps() {
  CHECK(first_match({0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}) == 2);
  CHECK(first_match({0xFF, 0xFF}, {0xFF, 0x12, 0xFF, 0xFF}) == 4);
  CHECK(first_match({0xAA, 0xAB}, {0xAA, 0xAA, 0xAB}) == 3);
  CHECK(first_match({0xAA, 0xAA, 0xAB}, {0xAA, 0xAA, 0xAA, 0xAB}) == 4);
  CHECK(first_match({1, 2, 1, 2, 3}, {1, 2, 1, 2, 1, 2, 3}) == 7);
  CHECK(first_match({1, 1, 2}, {1, 1, 1, 1, 2}) == 5);
}

static void test_no_match() {
  CHECK(first_match({0xFF, 0xFF}, {}) == 0);
  CHECK(first_match({0xFF, 0xFF}, {0xFF, 0x00, 0xFF, 0x00, 0xFF}) == 0);
  CHECK(first_match({1, 2, 3}, {1, 2, 4, 1, 2, 4}) == 0);
  CHECK(first_match({1, 2, 3}, {3, 2, 1}) == 0);
}

static void test_split_across_reads() {
  SyncMatcher m;
  const Bytes p{0xFF, 0xFF, 0x01};
  CHECK(m.reset(p));
  const Bytes a{0x00, 0xFF}, b{0xFF}, c{0x05, 0xFF, 0xFF, 0x01};
  bool hit = false;
  for (uint8_t x : a) hit |= m.feed(x);
  for (uint8_t x : b) hit |= m.feed(x);
  CHECK(!hit);
  CHECK(!m.complete());
  for (uint8_t x : c) hit |= m.feed(x);
  CHECK(hit);
  CHECK(m.complete());
}

static void test_repeated_after_reset() {
  SyncMatcher m;
  const Bytes p{0xFF, 0xFF};
  CHECK(m.reset(p));
  CHECK(!m.feed(0xFF));
  CHECK(m.feed(0xFF));
  m.restart();
  CHECK(!m.complete());
  CHECK(!m.feed(0xFF));
  CHECK(m.feed(0xFF));
  CHECK(m.reset(p));
  CHECK(!m.feed(0x00));
  CHECK(!m.feed(0xFF));
  CHECK(m.feed(0xFF));
  // Without restart, the overlap continues the search.
  CHECK(m.feed(0xFF));
  CHECK(!m.feed(0x00));
}

static void test_limits() {
  SyncMatcher m;
  const Bytes empty;
  CHECK(m.reset(empty));
  CHECK(m.complete());
  CHECK(m.feed(0x00));

  Bytes max(SyncMatcher::MAX_PATTERN, 0xAA);
  max.back() = 0xAB;
  Bytes s(5, 0xAA);
  s.insert(s.end(), max.begin(), max.end());
  CHECK(first_match(max, s) == s.size());

  Bytes over(SyncMatcher::MAX_PATTERN + 1, 0x01);
  CHECK(!m.reset(over));
  CHECK(first_match(over, over) == 0);
}

int main() {
  test_overlaps();
  test_no_match();
  test_split_across_reads();
  test_repeated_after_reset();
  test_limits();
  if (g_failures == 0) std::printf("test_sync_matcher: all passed\n");
  return g_failures == 0 ? 0 : 1;
}
