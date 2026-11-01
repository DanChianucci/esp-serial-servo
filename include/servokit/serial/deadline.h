//------------------------------------------------------------------------
// Filename  :  deadline.h
// Project   :  servokit
//
// Description:
//    Deadline arithmetic for platform serial ports: a fixed point in time on
//    a monotonic clock, and conversions from durations to the whole
//    milliseconds and RTOS ticks that drivers accept. Header-only, pure
//    functions and values: no I/O, never blocks, safe from any thread. No
//    platform dependencies, so it is tested on a host with an injected clock.
//------------------------------------------------------------------------

#ifndef SERVOKIT_DEADLINE__H
#define SERVOKIT_DEADLINE__H

#include <chrono>
#include <cstdint>

namespace servokit {
namespace timeout {

/** A point in time after which an operation must stop waiting.
 *
 *  The deadline is fixed when the object is constructed, measured on a
 *  monotonic clock, and independent of how many attempts are made before it.
 *  Arithmetic saturates: a timeout too large to represent never expires.
 *  A zero or negative timeout is expired at construction; callers that must
 *  attempt an operation once regardless use a do/while loop that tests
 *  expired() after the attempt.
 *
 *  @tparam Clock A clock type with static now(), time_point and duration
 *                (std::chrono::steady_clock by default; tests inject a fake).
 */
template <class Clock = std::chrono::steady_clock>
class BasicDeadline {
 public:
  using clock = Clock;                             ///< The clock type.
  using duration = typename Clock::duration;       ///< The clock's duration type.
  using time_point = typename Clock::time_point;   ///< The clock's time point type.

  /** @param timeout Time allowed, counted from @p start.
   *  @param start Reference instant; defaults to the current time. */
  explicit BasicDeadline(duration timeout, time_point start = Clock::now())
      : m_deadline(compute(timeout, start)) {}

  /** @return Time left at @p now, never negative. */
  duration remaining(time_point now = Clock::now()) const {
    return now >= m_deadline ? duration::zero() : m_deadline - now;
  }

  /** @return true when @p now is at or past the deadline. */
  bool expired(time_point now = Clock::now()) const { return now >= m_deadline; }

 private:
  // Deadline = start + timeout, saturating at the clock's maximum so an
  // unrepresentably large timeout means "never expires" instead of wrapping
  // into the past. A non-positive timeout is expired at construction.
  static time_point compute(duration timeout, time_point start) {
    if (timeout <= duration::zero()) return start;
    // For a start before the clock epoch, start + timeout cannot overflow
    // (the sum lies between start and timeout), whereas max() - start could.
    if (start >= time_point{} && timeout > time_point::max() - start) return time_point::max();
    return start + timeout;
  }

  time_point m_deadline;
};

/** Deadline on the monotonic std::chrono::steady_clock, the one used in production. */
using Deadline = BasicDeadline<std::chrono::steady_clock>;

/** Converts a duration to whole milliseconds, rounding up.
 *  Negative durations give 0; durations above UINT32_MAX ms saturate. */
template <class Duration>
constexpr uint32_t ceil_ms(Duration d) {
  using namespace std::chrono;
  if (d <= Duration::zero()) return 0;
  constexpr uint32_t max_ms = UINT32_MAX;
  if (d > milliseconds(max_ms)) return max_ms;
  return static_cast<uint32_t>(ceil<milliseconds>(d).count());
}

/** Converts milliseconds to RTOS ticks, rounding up so a wait never ends
 *  before the requested time (it can overrun by less than one tick).
 *  The result is ceil(ms * tick_rate_hz / 1000), computed in 64 bits.
 *  @param ms Milliseconds to convert (see ceil_ms()).
 *  @param tick_rate_hz RTOS tick rate in Hz; 0 is treated as 1.
 *  @param max_ticks Saturation limit for the result.
 */
constexpr uint32_t ceil_ticks(uint32_t ms, uint32_t tick_rate_hz, uint32_t max_ticks) {
  if (tick_rate_hz == 0) tick_rate_hz = 1;
  uint64_t ticks = (static_cast<uint64_t>(ms) * tick_rate_hz + 999) / 1000;
  return ticks > max_ticks ? max_ticks : static_cast<uint32_t>(ticks);
}

}  // namespace timeout
}  // namespace servokit

#endif  // SERVOKIT_DEADLINE__H
