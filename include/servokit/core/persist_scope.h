//------------------------------------------------------------------------
// Filename  :  persist_scope.h
// Project   :  servokit
//
// Description:
//    Scope guard around Servo::set_persist(): makes configuration writes
//    persistent for the lifetime of the guard. Header-only.
//------------------------------------------------------------------------

#ifndef PERSIST_SCOPE__H
#define PERSIST_SCOPE__H

#include "servokit/core/servo.h"
#include "servokit/core/servo_types.h"

namespace servokit {

/** Keeps configuration writes persistent for the lifetime of the object.
 *
 *  Construction calls set_persist(true). Unlocking is explicit: check ok() or
 *  result() before relying on persistence. The relock (set_persist(false)) is
 *  made at destruction or by close(), and only when the unlock returned OK, so
 *  every exit path of the enclosing scope relocks after a confirmed unlock. An
 *  unlock that returned an error (for example a timeout) may still have taken
 *  effect; in that case call set_persist(false) explicitly.
 *
 *  Blocking, timeouts and errors are those of Servo::set_persist(). Not
 *  thread-safe; serialize with the other calls on the same bus. The Servo is
 *  borrowed and must outlive the guard. Not copyable and not movable.
 *
 *  A relock failure in the destructor cannot be reported. Call close() to
 *  relock early and obtain the result. */
class PersistScope {
 public:
  /** @param servo Servo to unlock; must outlive this object. */
  explicit PersistScope(Servo& servo)
      : m_servo(servo), m_result(servo.set_persist(true)), m_open(m_result == Result::OK) {}

  PersistScope(const PersistScope&) = delete;
  PersistScope& operator=(const PersistScope&) = delete;
  PersistScope(PersistScope&&) = delete;
  PersistScope& operator=(PersistScope&&) = delete;

  /** Relocks if the unlock succeeded and close() was not called; the result
   *  is discarded. */
  ~PersistScope() {
    if (m_open) (void)m_servo.set_persist(false);
  }

  /** @return true when the unlock succeeded. */
  bool ok() const { return m_result == Result::OK; }

  /** @return Result of the unlock performed by the constructor. */
  Result result() const { return m_result; }

  /** Relocks immediately. After it the destructor does nothing. Without a successful
   *  unlock, or when already closed, sends nothing.
   *  @return Result of the relock; OK when there was nothing to relock. */
  Result close() {
    if (!m_open) return Result::OK;
    m_open = false;
    return m_servo.set_persist(false);
  }

 private:
  Servo& m_servo;
  Result m_result;
  bool m_open;
};

}  // namespace servokit

#endif  // PERSIST_SCOPE__H
