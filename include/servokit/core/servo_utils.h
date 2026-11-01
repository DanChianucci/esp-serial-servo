#ifndef SERVO_UTILS__H
#define SERVO_UTILS__H

#include <chrono>
#include <span>
#include <expected>


//TODO Use boost <expected> fopr c++11
//TODO Use boost <span> for c++11

namespace servokit {
typedef std::span<uint8_t> buffer_t;
typedef std::span<const uint8_t> buffer_const_t;

typedef std::chrono::steady_clock::time_point timeout_ctrl_t;
typedef timeout_ctrl_t::duration timeout_duration_t;


typedef  std::unexpected<int> srv_stat_t;

constexpr srv_stat_t SRV_OK         = srv_stat_t( 0);
constexpr srv_stat_t SRV_FAIL       = srv_stat_t(-1);
constexpr srv_stat_t SRV_TIMEOUT    = srv_stat_t(-2);
constexpr srv_stat_t SRV_OVERFLOW   = srv_stat_t(-3);
constexpr srv_stat_t SRV_PARAM_ERR  = srv_stat_t(-4);
constexpr srv_stat_t SRV_READ_FAIL  = srv_stat_t(-5);
constexpr srv_stat_t SRV_WRITE_FAIL = srv_stat_t(-6);
// Response validation failures (Card 03). Map to Result::BAD_RESPONSE.
constexpr srv_stat_t SRV_BAD_FRAME    = srv_stat_t(-7);  // short/inconsistent length or bad sync
constexpr srv_stat_t SRV_BAD_CHECKSUM = srv_stat_t(-8);  // checksum mismatch (verification enabled)
constexpr srv_stat_t SRV_BAD_ID       = srv_stat_t(-9);  // response from a different device ID







//Return type of servokit functions which may either return a result or a status
typedef std::expected<int, int> srv_result_t;


// Returns true if the result holds a value
inline bool is_value(srv_result_t result){
  return result.has_value();
}


//Returns true if the result holds a status code
inline bool is_status(srv_result_t result){
  return !result.has_value();
}

//Returns the status, or OK if the result was not a satatus flag
inline srv_stat_t get_status(srv_result_t result) {
  return result.has_value() ? SRV_OK : srv_stat_t(result.error());
}

// Returns the result, or the default value if result was a status flag
inline int get_value(srv_result_t result, int default_val = 0) {
  return result.value_or(default_val);
}



}  // namespace servokit

#endif  // SERVO_UTILS__H