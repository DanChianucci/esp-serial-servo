#include "servokit/stseries/sts_servo.h"

#include <algorithm>
#include <utility>

#include "servokit/serial/servo_internal.h"
#include "servokit/stseries/stseries_packet.h"

namespace servokit {

namespace {
// Largest frame: 6 meta bytes + address byte + MAX_SPAN payload (writes), or
// 6 meta bytes + MAX_SPAN data (read responses).
constexpr size_t FRAME_BUF = STS::NUM_META_BYTES + 1 + STSServo::MAX_SPAN;

struct TableEntry {
  uint8_t addr;
  uint8_t bytes;
  STSServo::VerifyClass kind;
  uint8_t mask;
};

constexpr auto STABLE = STSServo::VerifyClass::STABLE;
constexpr auto MASKED = STSServo::VerifyClass::STABLE_MASKED;
constexpr auto VOLATILE = STSServo::VerifyClass::VOLATILE;

// Writable registers of the STS control table; documented in sts_servo.h.
constexpr TableEntry VERIFY_TABLE[] = {
    {STS::REG_DEVICE_ID, 1, STABLE, 0xFF},
    {STS::REG_BAUD_RATE, 1, VOLATILE, 0x00},
    {STS::REG_RESPONSE_DELAY, 1, STABLE, 0xFF},
    {STS::REG_RESPONSE_MODE, 1, STABLE, 0xFF},
    {STS::REG_MIN_ANGLE_THRESH, 2, STABLE, 0xFF},
    {STS::REG_MAX_ANGLE_THRESH, 2, STABLE, 0xFF},
    {STS::REG_OVERTEMP_THRESH, 1, STABLE, 0xFF},
    {STS::REG_OVERVOLT_THRESH, 1, STABLE, 0xFF},
    {STS::REG_UNDERVOLT_THRESH, 1, STABLE, 0xFF},
    {STS::REG_CONTROL_TORQUE_NV, 2, STABLE, 0xFF},
    {STS::REG_PHASE, 1, MASKED, STSServo::PHASE_VERIFY_MASK},
    {STS::REG_UNLOAD_ALARM_MASK, 1, STABLE, 0xFF},
    {STS::REG_LED_ALARM_MASK, 1, STABLE, 0xFF},
    {STS::REG_POS_COEF_P, 1, STABLE, 0xFF},
    {STS::REG_POS_COEF_D, 1, STABLE, 0xFF},
    {STS::REG_POS_COEF_I, 1, STABLE, 0xFF},
    {STS::REG_START_TORQUE, 2, STABLE, 0xFF},
    {STS::REG_CW_DEADZONE, 1, STABLE, 0xFF},
    {STS::REG_CCW_DEADZONE, 1, STABLE, 0xFF},
    {STS::REG_OVERCURR_THRESH, 2, STABLE, 0xFF},
    {STS::REG_ANGLE_RESOLUTION, 1, STABLE, 0xFF},
    {STS::REG_POS_OFFSET, 2, STABLE, 0xFF},
    {STS::REG_OPERATION_MODE, 1, STABLE, 0xFF},
    {STS::REG_UNLOAD_CTRL_TORQUE, 1, STABLE, 0xFF},
    {STS::REG_OVERLOAD_TIME, 1, STABLE, 0xFF},
    {STS::REG_OVERLOAD_THRESH, 1, STABLE, 0xFF},
    {STS::REG_SPEED_COEF_P, 1, STABLE, 0xFF},
    {STS::REG_OVERCURR_TIME, 1, STABLE, 0xFF},
    {STS::REG_SPEED_COEF_I, 1, STABLE, 0xFF},
    {STS::REG_SERVO_EN, 1, VOLATILE, 0x00},
    {STS::REG_CTRL_ACCEL, 1, VOLATILE, 0x00},
    {STS::REG_CTRL_POS, 2, VOLATILE, 0x00},
    {STS::REG_CTRL_PWM, 2, VOLATILE, 0x00},
    {STS::REG_CTRL_SPEED, 2, VOLATILE, 0x00},
    {STS::REG_CTRL_TORQUE, 2, VOLATILE, 0x00},
    {STS::REG_EEPROM_LOCK, 1, STABLE, 0xFF},
};
}  // namespace

STSServo::VerifyRule STSServo::verify_rule(uint16_t addr) {
  for (const TableEntry& e : VERIFY_TABLE) {
    if (addr >= e.addr && addr < e.addr + e.bytes) return {e.kind, e.mask};
  }
  return {VerifyClass::VOLATILE, 0x00};
}

STSServo::STSServo(ServoController* controller, uint8_t id) : m_ctl(controller), m_id(id) {}

STSServo::STSServo(std::unique_ptr<ServoController> controller, uint8_t id)
    : m_owned(std::move(controller)), m_ctl(m_owned.get()), m_id(id) {}

Result STSServo::transact(uint8_t id, uint8_t instruction, const uint8_t* params, size_t param_len, uint8_t* out,
                          size_t out_len, bool tolerate_status) {
  m_last_status = 0;
  if (m_ctl == nullptr || id >= STS::BROADCAST_ID) return Result::PARAM;

  uint8_t cbuf[FRAME_BUF];
  uint8_t rbuf[FRAME_BUF];
  STSeriesPacket cmd{buffer_t(cbuf)};
  STSeriesPacket rsp{buffer_t(rbuf)};
  const Result built = to_result(cmd.init(id, instruction, buffer_const_t(params, param_len)));
  if (built != Result::OK) return built;

  const srv_result_t io = m_ctl->command_response(&cmd, &rsp);
  if (!io.has_value()) return to_result(io);

  // Sync, length and (when enabled) checksum were validated by the controller.
  // The ID is compared again so a reply from another servo is never accepted.
  if (rsp.packet_device_id() != id) return Result::BAD_RESPONSE;
  m_last_status = rsp.get_code();
  if (m_last_status != STS::STAT_OK && !tolerate_status) return Result::BAD_RESPONSE;
  if (rsp.get_length() != out_len + STS::NUM_CODE_BYTES + STS::NUM_CHKSUM_BYTES) return Result::BAD_RESPONSE;

  if (out_len > 0) {
    const buffer_const_t data = rsp.get_data();
    if (data.size() != out_len) return Result::BAD_RESPONSE;
    std::copy(data.begin(), data.end(), out);
  }
  return Result::OK;
}

Result STSServo::ping(uint8_t id) { return transact(id, STS::CMD_PING, nullptr, 0, nullptr, 0); }

Result STSServo::read_register(uint8_t id, uint16_t addr, uint8_t* data, size_t len) {
  if (data == nullptr || len == 0) return Result::PARAM;
  if (len > MAX_SPAN || addr + len > REGISTER_SPACE) return Result::OUT_OF_RANGE;
  const uint8_t params[2] = {static_cast<uint8_t>(addr), static_cast<uint8_t>(len)};
  // Read into a scratch copy so the caller's buffer is untouched on failure.
  uint8_t tmp[MAX_SPAN];
  const Result r = transact(id, STS::CMD_READ, params, sizeof(params), tmp, len);
  if (r == Result::OK) std::copy(tmp, tmp + len, data);
  return r;
}

Result STSServo::read_with_status(uint16_t addr, uint8_t* data, size_t len) {
  if (data == nullptr || len == 0) return Result::PARAM;
  if (len > MAX_SPAN || addr + len > REGISTER_SPACE) return Result::OUT_OF_RANGE;
  const uint8_t params[2] = {static_cast<uint8_t>(addr), static_cast<uint8_t>(len)};
  uint8_t tmp[MAX_SPAN];
  const Result r = transact(m_id, STS::CMD_READ, params, sizeof(params), tmp, len, true);
  if (r == Result::OK) std::copy(tmp, tmp + len, data);
  return r;
}

Result STSServo::write_register(uint8_t id, uint16_t addr, const uint8_t* data, size_t len) {
  if (data == nullptr || len == 0) return Result::PARAM;
  if (len > MAX_SPAN || addr + len > REGISTER_SPACE) return Result::OUT_OF_RANGE;
  uint8_t params[1 + MAX_SPAN];
  params[0] = static_cast<uint8_t>(addr);
  std::copy(data, data + len, params + 1);
  const Result w = transact(id, STS::CMD_WRITE, params, 1 + len, nullptr, 0);
  if (w != Result::OK || !m_verify_writes) return w;

  bool any_compared = false;
  for (size_t i = 0; i < len; ++i) any_compared = any_compared || verify_rule(addr + i).mask != 0;
  if (!any_compared) return Result::OK;

  // A write covering REG_DEVICE_ID moves the servo to the new ID.
  uint8_t read_id = id;
  if (addr <= STS::REG_DEVICE_ID && STS::REG_DEVICE_ID < addr + len) {
    read_id = data[STS::REG_DEVICE_ID - addr];
    if (read_id > STS::MAX_ID) return Result::OK;  // not addressable, nothing to compare
  }
  uint8_t back[MAX_SPAN];
  const Result r = read_register(read_id, addr, back, len);
  if (r != Result::OK) return r;
  for (size_t i = 0; i < len; ++i) {
    if (((back[i] ^ data[i]) & verify_rule(addr + i).mask) != 0) return Result::VERIFY_FAILED;
  }
  return Result::OK;
}

Result STSServo::read_u8(uint8_t id, uint16_t addr, uint8_t* out) { return read_register(id, addr, out, 1); }

Result STSServo::read_u16(uint8_t id, uint16_t addr, uint16_t* out) {
  if (out == nullptr) return Result::PARAM;
  uint8_t b[2];
  const Result r = read_register(id, addr, b, 2);
  if (r == Result::OK) *out = static_cast<uint16_t>(b[0] | (b[1] << 8));
  return r;
}

Result STSServo::write_u8(uint8_t id, uint16_t addr, uint8_t value) { return write_register(id, addr, &value, 1); }

Result STSServo::write_u16(uint8_t id, uint16_t addr, uint16_t value) {
  const uint8_t b[2] = {static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>(value >> 8)};
  return write_register(id, addr, b, 2);
}

Result STSServo::ping() { return ping(m_id); }
Result STSServo::read_raw(uint16_t addr, uint8_t* data, size_t len) { return read_register(m_id, addr, data, len); }
Result STSServo::write_raw(uint16_t addr, const uint8_t* data, size_t len) {
  return write_register(m_id, addr, data, len);
}

Result STSServo::set_verify_checksum(bool enable) {
  if (m_ctl == nullptr) return Result::PARAM;
  m_ctl->set_verify_checksum(enable);
  return Result::OK;
}
bool STSServo::verify_checksum() const { return m_ctl == nullptr ? true : m_ctl->verify_checksum(); }
uint32_t STSServo::checksum_errors() const { return m_ctl == nullptr ? 0 : m_ctl->checksum_errors(); }

Result STSServo::set_verify_writes(bool enable) {
  m_verify_writes = enable;
  return Result::OK;
}

Result STSServo::set_persist(bool persist) { return persist ? unlock_eeprom() : lock_eeprom(); }

}  // namespace servokit
