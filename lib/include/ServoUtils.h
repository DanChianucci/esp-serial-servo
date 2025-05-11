#ifndef SERVOUTILS__H
#define SERVOUTILS__H

#define SRV_RETURN_IF(cond, val) \
  do {                           \
    if ((cond)) {                \
      return (val);              \
    }                            \
  } while (0)

#define SRV_RETURN_IF_NOT(cond, val) \
  do {                               \
    if (!(cond)) {                   \
      return (val);                  \
    }                                \
  } while (0)

#define SRV_RETURN_ON_ERR(cond) \
  do {                          \
    int _stat = (cond);         \
    if (_stat < 0) {            \
      return _stat;             \
    }                           \
  } while (0)

const int SRV_OK = 0;
const int SRV_FAIL = -1;
const int SRV_TIMEOUT = -2;
const int SRV_OVERFLOW = -3;

#include <span>
typedef std::span<uint8_t> buffer_t;
typedef std::span<const uint8_t> buffer_const_t;

#include <chrono>
typedef std::chrono::steady_clock::time_point timeout_ctrl_t;
typedef timeout_ctrl_t::duration timeout_duration_t;

inline void init_timeout_ctrl(timeout_ctrl_t* ctrl) { *ctrl = std::chrono::steady_clock::now(); }

inline bool update_timeout(timeout_ctrl_t* ctrl, timeout_duration_t* remaining) {
  timeout_ctrl_t now = std::chrono::steady_clock::now();
  timeout_ctrl_t::duration elapsed = now - (*ctrl);

  if (elapsed < *remaining) {
    *remaining -= elapsed;
    *ctrl = now;
    return false;
  } else {
    *remaining = timeout_ctrl_t::duration::zero();
    return true;
  }
}

inline timeout_duration_t timeout_ms(uint32_t ms) { return std::chrono::milliseconds(ms); }

#endif  // SERVODUTILS__H