//------------------------------------------------------------------------
// Filename  :  servo_internal.h
// Project   :  servokit
//
// Description:
//    Early-return macros used inside the library's own sources. They are
//    implementation helpers, not part of the public API; applications
//    should not include this header.
//------------------------------------------------------------------------

#ifndef SERVO_INTERNAL__H
#define SERVO_INTERNAL__H

#include "servokit/core/servo_utils.h"
#include "servokit/serial/deadline.h"

/** Returns @p val from the enclosing function when @p cond is true. */
#define SRV_RETURN_IF(cond, val) \
  do {                           \
    if ((cond)) [[unlikely]] {   \
      return (val);              \
    }                            \
  } while (0)

/** Returns @p val from the enclosing function when @p cond is false. */
#define SRV_RETURN_IF_NOT(cond, val) \
  do {                               \
    if (!(cond)) [[unlikely]] {      \
      return (val);                  \
    }                                \
  } while (0)

/** Evaluates @p cond (an srv_stat_t or srv_result_t expression) once and
 *  returns its status from the enclosing function when it is an error.
 *  The enclosing function must return srv_stat_t or srv_result_t. */
#define SRV_RETURN_ON_ERR(cond)               \
  do {                                        \
    srv_stat_t _stat = get_status(cond);        \
    if (_stat != SRV_OK) [[unlikely]] { \
      return _stat;                           \
    }                                         \
  } while (0)

#endif //SERVO_INTERNAL__H
