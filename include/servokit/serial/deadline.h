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
  using clock = Clock;
  using duration = typename Clock::duration;
  using time_point = typename Clock::time_point;

  /** @param timeout Time allowed, counted from @p start. */
  explicit BasicDeadline(duration timeout, time_point start = Clock::now())
      : m_deadline(compute(timeout, start)) {}

  /** @return Time left at @p now, never negative. */
  duration remaining(time_point now = Clock::now()) const {
    return now >= m_deadline ? duration::zero() : m_deadline - now;
  }

  /** @return true when @p now is at or past the deadline. */
  bool expired(time_point now = Clock::now()) const { return now >= m_deadline; }

 private:
  static time_point compute(duration timeout, time_point start) {
    if (timeout <= duration::zero()) return start;
    if (timeout > time_point::max() - start) return time_point::max();
    return start + timeout;
  }

  time_point m_deadline;
};

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

/** Converts a duration to RTOS ticks, rounding up so a wait never ends before
 *  the requested time (it can overrun by less than one tick).
 *  @param ms Milliseconds to convert (see ceil_ms()).
 *  @param tick_period_ms Length of one tick in milliseconds, at least 1.
 *  @param max_ticks Saturation limit for the result.
 */
constexpr uint32_t ceil_ticks(uint32_t ms, uint32_t tick_period_ms, uint32_t max_ticks) {
  if (tick_period_ms == 0) tick_period_ms = 1;
  uint64_t ticks = (static_cast<uint64_t>(ms) + tick_period_ms - 1) / tick_period_ms;
  return ticks > max_ticks ? max_ticks : static_cast<uint32_t>(ticks);
}

}  // namespace timeout
}  // namespace servokit

#endif  // SERVOKIT_DEADLINE__H
