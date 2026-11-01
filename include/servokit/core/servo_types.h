//------------------------------------------------------------------------
// Filename  :  servo_types.h
// Project   :  ESP Serial Servo
//
// Description:
//    Family-neutral value types for the generic Servo API: results,
//    capabilities, alarms, identity, telemetry, protection and profile.
//
//    Units used everywhere in the generic API:
//      angle          millidegrees (mdeg), int32_t
//      speed          mdeg/s
//      acceleration   mdeg/s^2
//      current        mA
//      temperature    degrees C
//      voltage        mV
//      torque / load  permille (1/1000 of full scale)
//      time           milliseconds
//
//    All types are plain values: copyable, no heap, no references to
//    caller memory, safe to pass between threads by value. Nothing here
//    blocks or performs I/O.
//------------------------------------------------------------------------

#ifndef SERVO_TYPES__H
#define SERVO_TYPES__H

#include <cstddef>
#include <cstdint>

namespace servokit {

/** Result of a generic Servo operation. No exceptions are thrown. */
enum class Result : uint8_t {
  OK = 0,        ///< Operation completed.
  TIMEOUT,       ///< No (complete) response within the bounded timeout.
  BAD_RESPONSE,  ///< Framing, checksum, ID or servo error flag mismatch.
  UNSUPPORTED,   ///< The servo/family does not provide this operation.
  OUT_OF_RANGE,  ///< Argument outside the servo's domain/limits.
  PARAM,         ///< Invalid argument (null, bad length, bad value).
  BUS            ///< Transport failure (UART/driver), not a servo fault.
};

/** Generic fault bits reported by a servo. Family drivers translate their
 *  native status encodings into these. */
enum class Alarm : uint16_t {
  OVER_VOLTAGE = 1u << 0,      ///< Supply above the servo's limit
  UNDER_VOLTAGE = 1u << 1,     ///< Supply below the servo's limit
  OVER_TEMPERATURE = 1u << 2,  ///< Temperature above the servo's limit
  OVER_CURRENT = 1u << 3,      ///< Current above the limit for the set time
  OVERLOAD = 1u << 4,          ///< Load/torque above the limit for the set time
  STALLED = 1u << 5,           ///< Commanded to move but not moving
  ENCODER = 1u << 6,           ///< Angle sensor fault or invalid reading
  ANGLE_LIMIT = 1u << 7,       ///< Position outside the servo's own limits
  COMMUNICATION = 1u << 8,     ///< Servo-side bus or checksum error
  UNLOADED = 1u << 9,          ///< Torque released because of a protection trip
  OTHER = 1u << 10             ///< Native bit with no generic mapping
};

/** Bitmask of Alarm values. */
class AlarmSet {
 public:
  using bits_t = uint16_t;

  constexpr AlarmSet() = default;
  constexpr AlarmSet(Alarm a) : m_bits(static_cast<bits_t>(a)) {}
  static constexpr AlarmSet from_bits(bits_t bits) {
    AlarmSet s;
    s.m_bits = bits;
    return s;
  }

  constexpr bits_t bits() const { return m_bits; }
  constexpr bool none() const { return m_bits == 0; }
  constexpr bool any() const { return m_bits != 0; }
  constexpr bool has(Alarm a) const { return (m_bits & static_cast<bits_t>(a)) != 0; }
  /** True if every alarm in @p o is set. */
  constexpr bool has_all(AlarmSet o) const { return (m_bits & o.m_bits) == o.m_bits; }
  /** True if at least one alarm in @p o is set. */
  constexpr bool has_any(AlarmSet o) const { return (m_bits & o.m_bits) != 0; }

  constexpr AlarmSet& set(Alarm a) {
    m_bits |= static_cast<bits_t>(a);
    return *this;
  }
  constexpr AlarmSet& clear(Alarm a) {
    m_bits &= static_cast<bits_t>(~static_cast<bits_t>(a));
    return *this;
  }

  constexpr AlarmSet operator|(AlarmSet o) const { return from_bits(m_bits | o.m_bits); }
  constexpr AlarmSet operator&(AlarmSet o) const { return from_bits(m_bits & o.m_bits); }
  constexpr AlarmSet& operator|=(AlarmSet o) {
    m_bits |= o.m_bits;
    return *this;
  }
  constexpr bool operator==(const AlarmSet&) const = default;

  /** Short upper-case name of a single alarm; "UNKNOWN" if not a valid alarm.
   *  Returns a pointer to static storage. */
  static constexpr const char* name(Alarm a) {
    switch (a) {
      case Alarm::OVER_VOLTAGE: return "OVER_VOLTAGE";
      case Alarm::UNDER_VOLTAGE: return "UNDER_VOLTAGE";
      case Alarm::OVER_TEMPERATURE: return "OVER_TEMPERATURE";
      case Alarm::OVER_CURRENT: return "OVER_CURRENT";
      case Alarm::OVERLOAD: return "OVERLOAD";
      case Alarm::STALLED: return "STALLED";
      case Alarm::ENCODER: return "ENCODER";
      case Alarm::ANGLE_LIMIT: return "ANGLE_LIMIT";
      case Alarm::COMMUNICATION: return "COMMUNICATION";
      case Alarm::UNLOADED: return "UNLOADED";
      case Alarm::OTHER: return "OTHER";
    }
    return "UNKNOWN";
  }

  /** Writes the set alarm names separated by '|' into @p buf (NUL-terminated,
   *  truncated to fit). An empty set yields "NONE". Non-blocking, no heap,
   *  thread-safe on distinct buffers.
   *  @param buf Caller-owned output buffer.
   *  @param len Size of @p buf in bytes.
   *  @return Length written excluding the NUL; 0 if buf is null or len is 0. */
  size_t to_text(char* buf, size_t len) const {
    if (buf == nullptr || len == 0) return 0;
    size_t n = 0;
    auto put = [&](const char* s) {
      for (; *s != '\0' && n + 1 < len; ++s) buf[n++] = *s;
    };
    if (m_bits == 0) {
      put("NONE");
    } else {
      bool first = true;
      constexpr bits_t last = static_cast<bits_t>(Alarm::OTHER);
      for (bits_t bit = 1; bit <= last; bit = static_cast<bits_t>(bit << 1)) {
        if ((m_bits & bit) == 0) continue;
        if (!first) put("|");
        first = false;
        put(name(static_cast<Alarm>(bit)));
      }
    }
    buf[n] = '\0';
    return n;
  }

 private:
  bits_t m_bits = 0;
};

/** Capabilities a servo can declare. A flag is set only if the servo itself
 *  performs the feature; the library never emulates missing features. */
enum class Feature : uint64_t {
  // Motion modes
  ABSOLUTE_POSITION = 1ull << 0,
  MULTI_TURN_POS = 1ull << 1,
  RELATIVE_POSITION = 1ull << 2,
  CONTINUOUS_ROTATION = 1ull << 3,
  PWM_DRIVE = 1ull << 4,
  // Motion parameters
  SET_SPEED = 1ull << 5,
  SET_ACCEL = 1ull << 6,
  TORQUE_LIMIT = 1ull << 7,
  CURRENT_LIMIT = 1ull << 8,
  START_TORQUE = 1ull << 9,
  // Control
  TORQUE_ENABLE = 1ull << 10,
  HOLD = 1ull << 11,
  POSITION_GAINS = 1ull << 12,
  DEADZONE = 1ull << 13,
  ANGLE_LIMITS = 1ull << 14,
  POSITION_OFFSET = 1ull << 15,
  // Feedback
  READ_POSITION = 1ull << 16,
  READ_SPEED = 1ull << 17,
  READ_CURRENT = 1ull << 18,
  READ_LOAD = 1ull << 19,
  READ_VOLTAGE = 1ull << 20,
  READ_TEMPERATURE = 1ull << 21,
  READ_MOVING = 1ull << 22,
  READ_ALARMS = 1ull << 23,
  // Protection (performed by the servo itself)
  PROTECT_CURRENT = 1ull << 24,
  PROTECT_OVERLOAD = 1ull << 25,
  PROTECT_TEMP = 1ull << 26,
  PROTECT_VOLTAGE = 1ull << 27,
  PROTECT_RELEASE_ON_ALARM = 1ull << 28,
  // Bus and admin
  PING = 1ull << 29,
  READ_IDENTITY = 1ull << 30,
  SET_ID = 1ull << 31,
  SET_BAUD = 1ull << 32,
  SYNC_WRITE = 1ull << 33,
  SYNC_READ = 1ull << 34,
  ASYNC_WRITE = 1ull << 35,
  BROADCAST = 1ull << 36,
  RAW_REGISTERS = 1ull << 37,
  // Behaviour
  RETAINS_POSITION_UNPOWERED = 1ull << 38
};

/** Bitmask of Feature values. */
class FeatureSet {
 public:
  using bits_t = uint64_t;

  constexpr FeatureSet() = default;
  constexpr FeatureSet(Feature f) : m_bits(static_cast<bits_t>(f)) {}
  static constexpr FeatureSet from_bits(bits_t bits) {
    FeatureSet s;
    s.m_bits = bits;
    return s;
  }

  constexpr bits_t bits() const { return m_bits; }
  constexpr bool none() const { return m_bits == 0; }
  /** True if feature @p f is set. */
  constexpr bool has(Feature f) const { return (m_bits & static_cast<bits_t>(f)) != 0; }
  /** True if every feature in @p o is set (mask check). */
  constexpr bool has_all(FeatureSet o) const { return (m_bits & o.m_bits) == o.m_bits; }
  /** True if at least one feature in @p o is set. */
  constexpr bool has_any(FeatureSet o) const { return (m_bits & o.m_bits) != 0; }

  constexpr FeatureSet& set(Feature f) {
    m_bits |= static_cast<bits_t>(f);
    return *this;
  }
  constexpr FeatureSet& clear(Feature f) {
    m_bits &= ~static_cast<bits_t>(f);
    return *this;
  }

  constexpr FeatureSet operator|(FeatureSet o) const { return from_bits(m_bits | o.m_bits); }
  constexpr FeatureSet operator&(FeatureSet o) const { return from_bits(m_bits & o.m_bits); }
  constexpr FeatureSet& operator|=(FeatureSet o) {
    m_bits |= o.m_bits;
    return *this;
  }
  constexpr bool operator==(const FeatureSet&) const = default;

 private:
  bits_t m_bits = 0;
};

constexpr FeatureSet operator|(Feature a, Feature b) { return FeatureSet(a) | FeatureSet(b); }
constexpr AlarmSet operator|(Alarm a, Alarm b) { return AlarmSet(a) | AlarmSet(b); }

/** Numeric limits of a servo. Each value is valid only if the matching
 *  Feature is set; otherwise it is 0. Static per model. */
struct Limits {
  int32_t angle_min_mdeg = 0;        ///< Lowest commandable angle
  int32_t angle_max_mdeg = 0;        ///< Highest commandable angle
  uint32_t resolution_mdeg = 0;      ///< Angle per native step (mdeg/step)
  uint32_t max_speed_mdeg_s = 0;     ///< Maximum speed (SET_SPEED)
  uint32_t max_accel_mdeg_s2 = 0;    ///< Maximum acceleration (SET_ACCEL)
  uint16_t max_torque_permille = 0;  ///< Maximum torque limit (TORQUE_LIMIT), 1000 = full
  int32_t current_min_ma = 0;        ///< Lowest reportable/settable current (READ_CURRENT)
  int32_t current_max_ma = 0;        ///< Highest reportable/settable current
  AlarmSet reportable_alarms;        ///< Alarms this servo can actually report
};

/** What the connected servo does. Static per model; may depend on the result
 *  of Servo::read_identity(). */
struct Capabilities {
  FeatureSet features;  ///< What this servo supports
  Limits limits;        ///< Numbers; each valid only if its feature is set
};

/** Identity of a servo. Strings are fixed-size, NUL-terminated and owned by
 *  the struct (no lifetime dependency on the library). */
struct Identity {
  char family[16] = {};         ///< Family name, e.g. "STS"
  uint16_t model_major = 0;     ///< Model number, major
  uint16_t model_minor = 0;     ///< Model number, minor
  uint16_t firmware_major = 0;  ///< Firmware version, major
  uint16_t firmware_minor = 0;  ///< Firmware version, minor
  uint8_t device_id = 0;        ///< Bus ID of the servo
};

/** One atomic sample: the whole sample is returned or the call fails.
 *  Optional fields carry a validity flag; read them only when it is true. */
struct Telemetry {
  int32_t position_mdeg = 0;  ///< Position (multi-turn aware when supported)
  int32_t current_ma = 0;     ///< Motor current
  int16_t temperature_c = 0;  ///< Temperature in degrees C
  AlarmSet alarms;            ///< Active alarms

  bool load_valid = false;    ///< True if load_permille is valid (READ_LOAD)
  int16_t load_permille = 0;  ///< Load, signed permille of full scale

  bool voltage_valid = false;  ///< True if voltage_mv is valid (READ_VOLTAGE)
  uint16_t voltage_mv = 0;     ///< Supply voltage in mV

  bool moving_valid = false;  ///< True if moving is valid (READ_MOVING)
  bool moving = false;        ///< Servo reports motion in progress
};

/** Servo-side protection thresholds. The servo enforces them itself, even
 *  if the host stops. */
struct Protection {
  uint16_t stall_current_ma = 0;  ///< Current above which the stall timer runs
  uint16_t stall_time_ms = 0;     ///< Time above stall current before trip
  int16_t max_temperature_c = 0;  ///< Temperature trip threshold
  AlarmSet release_on_alarm;      ///< Alarms that make the servo release torque
};

/** Motion profile applied to following moves. Values are clamped to the
 *  servo's Limits. */
struct Profile {
  uint32_t speed_mdeg_s = 0;           ///< Target speed
  uint32_t accel_mdeg_s2 = 0;          ///< Acceleration
  uint16_t torque_permille = 0;        ///< Torque limit, 1000 = full
  uint16_t start_torque_permille = 0;  ///< Start torque; ignored without START_TORQUE
};

}  // namespace servokit

#endif  // SERVO_TYPES__H
