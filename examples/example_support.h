// Shared helpers for the host examples: frame builders for scripted replies.
// The examples talk to a scripted fake SerialPort instead of a UART, so they
// build and run on a host with no hardware and no platform SDK.
#ifndef EXAMPLE_SUPPORT__H
#define EXAMPLE_SUPPORT__H

#include <cstdint>
#include <cstdio>

#include "fake_serial_port.h"

namespace example {

using servokit_test::Bytes;

// Appends the checksum: the low byte of the inverted sum of everything after
// the two sync bytes.
inline Bytes with_checksum(Bytes frame) {
  unsigned sum = 0;
  for (size_t i = 2; i < frame.size(); ++i) sum += frame[i];
  frame.push_back(static_cast<uint8_t>(~sum));
  return frame;
}

// Reply of servo `id` carrying `data` (empty for a write acknowledgement).
inline Bytes reply(uint8_t id, Bytes data = {}, uint8_t status = 0) {
  Bytes frame = {0xFF, 0xFF, id, static_cast<uint8_t>(data.size() + 2), status};
  frame.insert(frame.end(), data.begin(), data.end());
  return with_checksum(frame);
}

// Counts failed expectations; main() returns non-zero if any failed.
// One counter per program: the function-local static is shared by every
// translation unit that includes this header.
inline int& failures() {
  static int count = 0;
  return count;
}

// Checks a condition; on failure prints file, line and the expression and counts it.
#define EXPECT(cond)                                                \
  do {                                                              \
    if (!(cond)) {                                                  \
      std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++example::failures();                                        \
    }                                                               \
  } while (0)

}  // namespace example

#endif  // EXAMPLE_SUPPORT__H
