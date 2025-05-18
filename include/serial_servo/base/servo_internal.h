#ifndef SERVO_INTERNAL__H
#define SERVO_INTERNAL__H

#include "serial_servo/base/servo_utils.h"

#define SRV_RETURN_IF(cond, val) \
  do {                           \
    if ((cond)) [[unlikely]] {   \
      return (val);              \
    }                            \
  } while (0)

#define SRV_RETURN_IF_NOT(cond, val) \
  do {                               \
    if (!(cond)) [[unlikely]] {      \
      return (val);                  \
    }                                \
  } while (0)

#define SRV_RETURN_ON_ERR(cond)               \
  do {                                        \
    srv_stat_t _stat = get_status(cond);        \
    if (_stat != SRV_OK) [[unlikely]] { \
      return _stat;                           \
    }                                         \
  } while (0)

namespace esp_serial_servo {
namespace timeout {
/** Initializes the timeout control to start counting down from the time of the call */
inline void init_timeout_ctrl(timeout_ctrl_t* ctrl) { *ctrl = std::chrono::steady_clock::now(); }

/** Updates the timeout control and remaining time.
 * @return TRUE if the timeout has expired since last call.
 * @return FALSE if the timeout still has remaining time
 */
inline bool update_timeout(timeout_ctrl_t* ctrl, timeout_duration_t* remaining) {
  #ifdef DISABLE_SRV_TIMEOUTS
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
  #else
    return false;
  #endif
}
}  // namespace timeout
}  // namespace esp_serial_servo

#endif //SERVO_INTERNAL__H