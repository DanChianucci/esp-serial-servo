#ifndef SERVO_INTERNAL__H
#define SERVO_INTERNAL__H

#include "servokit/core/servo_utils.h"
#include "servokit/serial/deadline.h"

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

#endif //SERVO_INTERNAL__H