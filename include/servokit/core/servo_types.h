//------------------------------------------------------------------------
// Filename  :  servo_types.h
// Project   :  servokit
//
// Description:
//    Family-neutral value types for the generic Servo API: results,
//    capabilities, alarms, identity, telemetry and motion mode.
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
  BUS,           ///< Transport failure (UART/driver), not a servo fault.
  VERIFY_FAILED  ///< A verified write read back different from the bytes written.
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

/** Bitmask of Alarm values. A plain value: copyable, no heap, safe to pass
 *  between threads by value. */
class AlarmSet {
 public:
  using bits_t = uint16_t;  ///< Integer type holding the mask.

  /** The empty set. */
  constexpr AlarmSet() = default;
  /** A set holding the single alarm @p a (implicit, so an Alarm converts). */
  constexpr AlarmSet(Alarm a) : m_bits(static_cast<bits_t>(a)) {}
  /** Builds a set from raw mask bits (Alarm bit values); no validation. */
  static constexpr AlarmSet from_bits(bits_t bits) {
    AlarmSet s;
    s.m_bits = bits;
    return s;
  }

  /** @return The raw mask. */
  constexpr bits_t bits() const { return m_bits; }
  /** @return true when no alarm is set. */
  constexpr bool none() const { return m_bits == 0; }
  /** @return true when at least one alarm is set. */
  constexpr bool any() const { return m_bits != 0; }
  /** @return true if alarm @p a is set. */
  constexpr bool has(Alarm a) const { return (m_bits & static_cast<bits_t>(a)) != 0; }
  /** True if every alarm in @p o is set. */
  constexpr bool has_all(AlarmSet o) const { return (m_bits & o.m_bits) == o.m_bits; }
  /** True if at least one alarm in @p o is set. */
  constexpr bool has_any(AlarmSet o) const { return (m_bits & o.m_bits) != 0; }

  /** Adds @p a to the set. @return *this. */
  constexpr AlarmSet& set(Alarm a) {
    m_bits |= static_cast<bits_t>(a);
    return *this;
  }
  /** Removes @p a from the set. @return *this. */
  constexpr AlarmSet& clear(Alarm a) {
    m_bits &= static_cast<bits_t>(~static_cast<bits_t>(a));
    return *this;
  }

  /** Union of two sets. */
  constexpr AlarmSet operator|(AlarmSet o) const { return from_bits(m_bits | o.m_bits); }
  /** Intersection of two sets. */
  constexpr AlarmSet operator&(AlarmSet o) const { return from_bits(m_bits & o.m_bits); }
  /** Adds every alarm of @p o. @return *this. */
  constexpr AlarmSet& operator|=(AlarmSet o) {
    m_bits |= o.m_bits;
    return *this;
  }
  /** Sets compare equal when they hold the same alarms. */
  constexpr bool operator==(const AlarmSet&) const = default;

  /** Short upper-case name of a single alarm; "UNKNOWN" if not a valid alarm.
   *  @return A pointer to static storage; never null. */
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
    // Appends s, leaving room for the terminating NUL; truncates silently.
    auto put = [&](const char* s) {
      for (; *s != '\0' && n + 1 < len; ++s) buf[n++] = *s;
    };
    if (m_bits == 0) {
      put("NONE");
    } else {
      bool first = true;
      // Walk the bits from lowest to highest so names come out in enum order.
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
 *  performs the feature; the library never emulates missing features. The
 *  operation each flag gates returns Result::UNSUPPORTED when it is absent.
 *  Flags that name no Servo method yet (for example MULTI_TURN_POS,
 *  CONTINUOUS_ROTATION, SYNC_WRITE) are reserved for operations the interface
 *  does not provide; no family sets them. */
enum class Feature : uint64_t {
  // Motion modes
  ABSOLUTE_POSITION = 1ull << 0,   ///< Drives to an absolute target angle (Servo::move_to()).
  MULTI_TURN_POS = 1ull << 1,      ///< Absolute targets beyond one revolution.
  RELATIVE_POSITION = 1ull << 2,   ///< Moves relative to the current position.
  CONTINUOUS_ROTATION = 1ull << 3, ///< Constant-speed rotation without a target.
  PWM_DRIVE = 1ull << 4,           ///< Open-loop drive by PWM duty.
  // Motion parameters
  SET_SPEED = 1ull << 5,           ///< Servo::set_speed().
  SET_ACCEL = 1ull << 6,           ///< Servo::set_acceleration().
  TORQUE_LIMIT = 1ull << 7,        ///< Servo::set_torque_limit().
  CURRENT_LIMIT = 1ull << 8,       ///< A settable motor current limit.
  START_TORQUE = 1ull << 9,        ///< Servo::set_start_torque().
  // Control
  TORQUE_ENABLE = 1ull << 10,      ///< Servo::set_torque_enabled() and read_torque_enabled().
  CANCEL_MOTION = 1ull << 11,      ///< Servo::cancel_motion()
  POSITION_GAINS = 1ull << 12,     ///< Adjustable position control-loop gains.
  DEADZONE = 1ull << 13,           ///< Adjustable position dead zone.
  ANGLE_LIMITS = 1ull << 14,       ///< Adjustable angle limits stored in the servo.
  POSITION_OFFSET = 1ull << 15,    ///< Adjustable position correction offset.
  // Feedback
  READ_POSITION = 1ull << 16,      ///< Telemetry::position_mdeg is reported.
  READ_SPEED = 1ull << 17,         ///< The servo can report its speed (no generic accessor reads it yet).
  READ_CURRENT = 1ull << 18,       ///< Telemetry::current_ma is reported.
  READ_LOAD = 1ull << 19,          ///< Telemetry::load_permille is reported.
  READ_VOLTAGE = 1ull << 20,       ///< Telemetry::voltage_mv is reported.
  READ_TEMPERATURE = 1ull << 21,   ///< Telemetry::temperature_c is reported.
  READ_MOVING = 1ull << 22,        ///< Telemetry::moving is reported.
  READ_ALARMS = 1ull << 23,        ///< Servo::read_alarms() and Telemetry::alarms are reported.
  // Protection (performed by the servo itself)
  PROTECT_CURRENT = 1ull << 24,    ///< Servo::set_protect_current()
  PROTECT_OVERLOAD = 1ull << 25,   ///< Servo::set_protect_overload()
  PROTECT_TEMP = 1ull << 26,       ///< Servo::set_protect_temperature()
  PROTECT_VOLTAGE = 1ull << 27,    ///< Servo::set_protect_voltage()
  PROTECT_RELEASE_ON_ALARM = 1ull << 28,  ///< Servo::set_protect_release_alarms()
  // Bus and admin
  PROBE = 1ull << 29,     ///< Servo::probe()
  IDENTIFY = 1ull << 30,  ///< Servo::identify()
  SET_ID = 1ull << 31,    ///< The servo's bus ID can be changed.
  SET_BAUD = 1ull << 32,  ///< The servo's baud rate can be changed.
  SYNC_WRITE = 1ull << 33,   ///< Bus-level write to several servos in one frame.
  SYNC_READ = 1ull << 34,    ///< Bus-level read from several servos in one command.
  ASYNC_WRITE = 1ull << 35,  ///< Deferred write executed by a later action command.
  BROADCAST = 1ull << 36,    ///< Commands addressed to every servo on the bus.
  REGISTER_ACCESS = 1ull << 37,  ///< Servo::get_registers() returns a RegisterAccess
  // Behaviour
  RETAINS_POSITION_UNPOWERED = 1ull << 38,  ///< The servo keeps its position count while unpowered.
  // Configuration storage
  PERSIST_CONFIG = 1ull << 39,  ///< set_persist() controls whether configuration writes survive power loss
  // Write checking
  READBACK_CHECK = 1ull << 40  ///< RegisterAccess::set_readback_check() makes writes read back and compare
};

/** Bitmask of Feature values. A plain value: copyable, no heap, safe to pass
 *  between threads by value. */
class FeatureSet {
 public:
  using bits_t = uint64_t;  ///< Integer type holding the mask.

  /** The empty set. */
  constexpr FeatureSet() = default;
  /** A set holding the single feature @p f (implicit, so a Feature converts). */
  constexpr FeatureSet(Feature f) : m_bits(static_cast<bits_t>(f)) {}
  /** Builds a set from raw mask bits (Feature bit values); no validation. */
  static constexpr FeatureSet from_bits(bits_t bits) {
    FeatureSet s;
    s.m_bits = bits;
    return s;
  }

  /** @return The raw mask. */
  constexpr bits_t bits() const { return m_bits; }
  /** @return true when no feature is set. */
  constexpr bool none() const { return m_bits == 0; }
  /** True if feature @p f is set. */
  constexpr bool has(Feature f) const { return (m_bits & static_cast<bits_t>(f)) != 0; }
  /** True if every feature in @p o is set (mask check). */
  constexpr bool has_all(FeatureSet o) const { return (m_bits & o.m_bits) == o.m_bits; }
  /** True if at least one feature in @p o is set. */
  constexpr bool has_any(FeatureSet o) const { return (m_bits & o.m_bits) != 0; }

  /** Adds @p f to the set. @return *this. */
  constexpr FeatureSet& set(Feature f) {
    m_bits |= static_cast<bits_t>(f);
    return *this;
  }
  /** Removes @p f from the set. @return *this. */
  constexpr FeatureSet& clear(Feature f) {
    m_bits &= ~static_cast<bits_t>(f);
    return *this;
  }

  /** Union of two sets. */
  constexpr FeatureSet operator|(FeatureSet o) const { return from_bits(m_bits | o.m_bits); }
  /** Intersection of two sets. */
  constexpr FeatureSet operator&(FeatureSet o) const { return from_bits(m_bits & o.m_bits); }
  /** Adds every feature of @p o. @return *this. */
  constexpr FeatureSet& operator|=(FeatureSet o) {
    m_bits |= o.m_bits;
    return *this;
  }
  /** Sets compare equal when they hold the same features. */
  constexpr bool operator==(const FeatureSet&) const = default;

 private:
  bits_t m_bits = 0;
};

/** Combines two features into a set. */
constexpr FeatureSet operator|(Feature a, Feature b) { return FeatureSet(a) | FeatureSet(b); }
/** Combines two alarms into a set. */
constexpr AlarmSet operator|(Alarm a, Alarm b) { return AlarmSet(a) | AlarmSet(b); }

/** Numeric limits of a servo. Each value is valid only if the matching
 *  Feature is set (otherwise it is 0), except max_torque_mnm, which is
 *  informational and has no feature. Static per model. */
struct Limits {
  int32_t angle_min_mdeg = 0;        ///< Lowest commandable angle, mdeg
  int32_t angle_max_mdeg = 0;        ///< Highest commandable angle, mdeg
  uint32_t resolution_mdeg = 0;      ///< Angle per native step (mdeg/step)
  uint32_t max_speed_mdeg_s = 0;     ///< Maximum speed, mdeg/s (SET_SPEED)
  uint32_t max_accel_mdeg_s2 = 0;    ///< Maximum acceleration, mdeg/s^2 (SET_ACCEL)
  uint16_t max_torque_permille = 0;  ///< Maximum torque limit (TORQUE_LIMIT), 1000 = full
  uint32_t max_torque_mnm = 0;       ///< Informational stall torque in mN*m at the voltage the model's documentation states; 0 = unknown. Commands use permille, never this value
  int32_t current_min_ma = 0;        ///< Lowest reportable current, mA (READ_CURRENT); a settable threshold may span less
  int32_t current_max_ma = 0;        ///< Highest reportable current, mA
  AlarmSet reportable_alarms;        ///< Alarms this servo can actually report
};

/** What the connected servo does. Static per model; may depend on the result
 *  of Servo::identify(). */
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
  int32_t position_mdeg = 0;  ///< Position in mdeg (multi-turn aware when supported)
  int32_t current_ma = 0;     ///< Motor current, mA
  int16_t temperature_c = 0;  ///< Temperature in degrees C
  AlarmSet alarms;            ///< Active alarms at the time of the sample

  bool load_valid = false;    ///< True if load_permille is valid (READ_LOAD)
  int16_t load_permille = 0;  ///< Load, signed permille of full scale

  bool voltage_valid = false;  ///< True if voltage_mv is valid (READ_VOLTAGE)
  uint16_t voltage_mv = 0;     ///< Supply voltage in mV

  bool moving_valid = false;  ///< True if moving is valid (READ_MOVING)
  bool moving = false;        ///< True while the servo reports motion in progress
};

/** Motion mode selected with Servo::set_mode(). */
enum class MotionMode : uint8_t {
  ABSOLUTE_POSITION  ///< The servo drives to an absolute target angle over its full domain.
};

}  // namespace servokit

#endif  // SERVO_TYPES__H
