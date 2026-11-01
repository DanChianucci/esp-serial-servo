//------------------------------------------------------------------------
// Filename  :  servo_utils.h
// Project   :  servokit
//
// Description:
//    Transport-level types shared by SerialPort, ServoController and the
//    packet classes: byte views, timeout types and the integer status codes
//    (srv_stat_t / srv_result_t). The generic Servo API reports the
//    family-neutral Result instead; to_result() (servo_controller.h) maps
//    one to the other.
//
//    Everything here is a pure value or function: no I/O, never blocks, safe
//    to use from any thread.
//------------------------------------------------------------------------

#ifndef SERVO_UTILS__H
#define SERVO_UTILS__H

#include <chrono>
#include <span>
#include <expected>

// TODO(servokit): a C++11 build would need replacements for <expected> and
// <span> (for example the Boost equivalents); the library requires C++23.

namespace servokit {

/** Non-owning writable view of bytes. The viewed storage is owned by the
 *  caller and must outlive every use of the view. */
typedef std::span<uint8_t> buffer_t;

/** Non-owning read-only view of bytes; same lifetime rule as buffer_t. */
typedef std::span<const uint8_t> buffer_const_t;

/** Point on the monotonic clock used for deadlines. */
typedef std::chrono::steady_clock::time_point timeout_ctrl_t;

/** Time span used for every timeout argument (std::chrono, never RTOS ticks).
 *  Zero or negative durations are "already expired" unless a function says
 *  otherwise (see ServoController::DEF_TIMEOUT). */
typedef timeout_ctrl_t::duration timeout_duration_t;

/** A transport status code. It is the error half of std::expected, so a
 *  function returning srv_stat_t returns SRV_OK on success and one of the
 *  negative SRV_* codes on failure. Read the code with error(); compare a
 *  status with SRV_OK through get_status(), not with ==. */
typedef std::unexpected<int> srv_stat_t;

/** Success. The only status code that is not an error (value 0). */
constexpr srv_stat_t SRV_OK = srv_stat_t(0);
/** Generic transport or driver failure (value -1). Maps to Result::BUS. */
constexpr srv_stat_t SRV_FAIL = srv_stat_t(-1);
/** No (complete) data arrived within the time allowed (value -2). Maps to Result::TIMEOUT. */
constexpr srv_stat_t SRV_TIMEOUT = srv_stat_t(-2);
/** The caller's buffer or the packet capacity is too small (value -3). Maps to Result::PARAM. */
constexpr srv_stat_t SRV_OVERFLOW = srv_stat_t(-3);
/** An argument or configuration was invalid (value -4). Maps to Result::PARAM. */
constexpr srv_stat_t SRV_PARAM_ERR = srv_stat_t(-4);
/** The transport reported an error while reading (value -5). Maps to Result::BUS. */
constexpr srv_stat_t SRV_READ_FAIL = srv_stat_t(-5);
/** The transport reported an error while writing (value -6). Maps to Result::BUS. */
constexpr srv_stat_t SRV_WRITE_FAIL = srv_stat_t(-6);

// Response validation failures. All map to Result::BAD_RESPONSE.

/** The received frame has an inconsistent length or a wrong sync header (value -7). */
constexpr srv_stat_t SRV_BAD_FRAME = srv_stat_t(-7);
/** The received frame's checksum does not match; returned only while checksum
 *  verification is enabled (value -8). */
constexpr srv_stat_t SRV_BAD_CHECKSUM = srv_stat_t(-8);
/** The response came from a different device ID than the command's (value -9). */
constexpr srv_stat_t SRV_BAD_ID = srv_stat_t(-9);

/** Return type of functions that give either a value (for example a byte
 *  count, never negative) or a status code. Test it with has_value(); the
 *  error() member is the SRV_* code. */
typedef std::expected<int, int> srv_result_t;

/** @return true if @p result holds a value. */
inline bool is_value(srv_result_t result){
  return result.has_value();
}

/** @return true if @p result holds a status code. */
inline bool is_status(srv_result_t result){
  return !result.has_value();
}

/** @return SRV_OK if @p result holds a value, otherwise its status code. */
inline srv_stat_t get_status(srv_result_t result) {
  return result.has_value() ? SRV_OK : srv_stat_t(result.error());
}

/** @return The value held by @p result, or @p default_val if it holds a status code. */
inline int get_value(srv_result_t result, int default_val = 0) {
  return result.value_or(default_val);
}

}  // namespace servokit

#endif  // SERVO_UTILS__H
