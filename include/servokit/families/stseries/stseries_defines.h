//------------------------------------------------------------------------
// Filename  :  stseries_defines.h
// Project   :  servokit
//
// Description:
//    Protocol constants, enumerations and the control-table register map of
//    the STS series (for example ST3215). Constants only: no code, no I/O.
//
//    Sources: the ST3215 memory table and communication manual (doc/STS3215),
//    and the vendor sample code in that folder. Items the repo has not
//    confirmed on a physical servo are marked "unverified".
//
//    Frame layout (see stseries_packet.h):
//      FF FF id length code data... checksum
//      length   = number of bytes after the length byte (code + data + checksum)
//      code     instruction (cmd_e) in a command, status (err_code_t) in a reply
//      checksum = ~(id + length + code + data...), low byte
//
//    Register map conventions: addresses are single bytes; a 2-byte register
//    is little endian (low byte at the stated address, high byte at the next).
//    "EEPROM" registers keep their value across a power cycle unless
//    REG_EEPROM_LOCK is 1; "SRAM" registers reset at power-up.
//------------------------------------------------------------------------

#ifndef STSERIES_DEFINES__H
#define STSERIES_DEFINES__H

#include <cstdint>
#include <cstdio>

namespace servokit {

/** Constants of the STS series protocol and control table. */
namespace STS {

/** Servo firmware version the register map was written against (3.7), as
 *  major in the high byte and minor in the low byte (0x03_07). Informational;
 *  the library never compares it with anything. */
constexpr uint16_t LIB_FW_VERSION = 0x0307;
/** Frame sync pattern: two 0xFF bytes opening every command and reply. */
constexpr uint16_t SYNC_PATTERN = 0xFFFF;
/** Highest device ID the library treats as addressable by the readback
 *  check. The memory table allows IDs up to 253 (0xFD) in REG_DEVICE_ID;
 *  0xFD can still be addressed for ordinary calls, but a write that moves a
 *  servo to an ID above this value is not read back. */
constexpr uint8_t MAX_ID = 0xFC;
/** Broadcast device ID: every servo executes the command and none replies. */
constexpr uint8_t BROADCAST_ID = 0xFE;

constexpr uint8_t SYNC_IDX   = 0;  ///< Frame index of the first sync byte.
constexpr uint8_t DEVID_IDX  = 2;  ///< Frame index of the device ID byte.
constexpr uint8_t LENGTH_IDX = 3;  ///< Frame index of the length byte.
constexpr uint8_t CODE_IDX   = 4;  ///< Frame index of the instruction/status byte.
constexpr uint8_t DATA_IDX   = 5;  ///< Frame index of the first data byte.

constexpr uint8_t NUM_SYNC_BYTES   = 2;  ///< Size of the sync pattern, bytes.
constexpr uint8_t NUM_DEVID_BYTES  = 1;  ///< Size of the device ID field, bytes.
constexpr uint8_t NUM_LENGTH_BYTES = 1;  ///< Size of the length field, bytes.
constexpr uint8_t NUM_CODE_BYTES  = 1;   ///< Size of the instruction/status field, bytes.
constexpr uint8_t NUM_CHKSUM_BYTES = 1;  ///< Size of the checksum field, bytes.
constexpr uint8_t NUM_META_BYTES = 6;    ///< Frame bytes besides the data: sync (2), ID, length, code, checksum.


/** STS command instructions (the code byte of a command frame). */
typedef enum {
  CMD_PING = 0x01,       ///< Ask a servo to answer; no data.
  CMD_READ = 0x02,       ///< Read registers: data = address, byte count.
  CMD_WRITE = 0x03,      ///< Write registers: data = address, bytes.
  CMD_REG_WRITE = 0x04,  ///< Deferred write, executed by CMD_ACTION. Not used by the library.
  CMD_ACTION = 0x05,     ///< Execute deferred writes. Not used by the library.
  CMD_RESET = 0x06,      ///< Protocol reset instruction. Not used by the library.
  CMD_SYNC_READ = 0x82,  ///< Read from several servos. Not in the protocol manual; appears in the vendor sample code. Not used by the library.
  CMD_SYNC_WRITE = 0x83  ///< Write to several servos in one frame. Not used by the library.
} cmd_e;

/** STS alarm / status codes. One bit per fault; the same layout is used by the
 *  status byte of every response, REG_STATUS, REG_LED_ALARM_MASK (bit set =
 *  that fault flashes the LED) and REG_UNLOAD_ALARM_MASK (bit set = that fault
 *  releases torque). Source: ST3215 memory table, registers 0x13, 0x14 and
 *  0x41. A status byte is the OR of the bits that apply. */
typedef enum {
  STAT_OK = 0x00,             ///< No error.
  STAT_VOLTAGE = 1 << 0,      ///< Voltage error (supply outside the voltage thresholds).
  STAT_SENSOR = 1 << 1,       ///< Sensor error.
  STAT_TEMPERATURE = 1 << 2,  ///< Temperature error.
  STAT_CURRENT = 1 << 3,      ///< Current error.
  STAT_ANGLE = 1 << 4,        ///< Angle error.
  STAT_OVERLOAD = 1 << 5      ///< Overload error.
} err_code_t;
/** Alternative name of err_code_t used in documentation and call sites. */
typedef err_code_t sts_err_code_t;

/** Values of REG_RESPONSE_MODE: which commands the servo answers. */
typedef enum {
  RSP_READ_PING = 0,  ///< Respond to READ and PING commands only (writes get no reply).
  RSP_ALL = 1         ///< Respond to all commands.
} rsp_mode_t;

/** Values of REG_OPERATION_MODE: how the servo interprets its control registers. */
typedef enum {
  OP_SERVO = 0,  ///< Position servo mode, controlled by REG_CTRL_POS.
  OP_SPEED = 1,  ///< Constant speed mode, controlled by REG_CTRL_SPEED (bit 15 = direction).
  OP_PWM = 2,    ///< Open-loop PWM mode, controlled by REG_CTRL_PWM.
  OP_STEP = 3,   ///< Stepper mode, REG_CTRL_POS is a step count (bit 15 = direction). Listed in the memory table text; its range column stops at 2, so unverified.
} op_mode_t;

/** Values of REG_BAUD_RATE (the comment of each value is the bus speed). */
typedef enum {
  BAUD_1000000 = 0,  ///< 1,000,000 baud.
  BAUD_500000 = 1,   ///< 500,000 baud.
  BAUD_250000 = 2,   ///< 250,000 baud.
  BAUD_128000 = 3,   ///< 128,000 baud.
  BAUD_115200 = 4,   ///< 115,200 baud.
  BAUD_76800 = 5,    ///< 76,800 baud.
  BAUD_57600 = 6,    ///< 57,600 baud.
  BAUD_38400 = 7     ///< 38,400 baud.
} baud_rate_e;

/** Values of REG_SERVO_EN (torque switch). The register also accepts 128, which
 *  calibrates the current position to 2048; the library never writes it. */
typedef enum {
  SERVO_DISABLE = 0,  ///< Torque output off.
  SERVO_ENABLE = 1    ///< Torque output on.
} servo_en_t;

/** Values of REG_EEPROM_LOCK. The meaning is from the register map and is
 *  unverified on hardware. */
typedef enum {
  EEPROM_UNLOCKED = 0,  ///< Writes to EEPROM-region registers are saved across a power cycle.
  EEPROM_LOCKED = 1     ///< EEPROM-region writes take effect but are not saved across a power cycle.
} eeprom_lock_t;

/** Values of REG_MOVE_FLAG. */
typedef enum {
  SERVO_STOPPED = 0,  ///< The servo is not moving.
  SERVO_MOVING = 1    ///< The servo is moving.
} move_flag_t;

// TODO(multi-turn): REG_PHASE bit 4 has to be set during multi-turn control to
// enable angle resolution feedback; the library does not enable multi-turn
// control itself (see STSServo::set_multiturn_feedback()).

/** STS control table. Each entry's comment gives: name, then
 *  `address (decimal) | width | access (RW/RO) | region`, then the unit and the
 *  value range of the memory table. Units named "step" are 1/4096 of a
 *  revolution. */
typedef enum {
  // clang-format off
  // Manufacturing Data
  REG_VER_MAJ            = 0x00, // Firmware Major Version                            @details `0x00 (0)  | 1B | RO | EEPROM`
  REG_VER_SUB            = 0x01, // Firmware Minor Version                            @details `0x01 (1)  | 1B | RO | EEPROM`
  REG_MODEL_MAJ          = 0x03, // Servo model major (memory table: "main version") @details `0x03 (3)  | 1B | RO | EEPROM`
  REG_MODEL_SUB          = 0x04, // Servo model minor (memory table: "sub version")  @details `0x04 (4)  | 1B | RO | EEPROM`
  // Communications
  REG_DEVICE_ID          = 0x05, // Device ID, unique on the bus; 0xFE is broadcast   @details `0x05 (5)  | 1B | RW | EEPROM` @note values: [0, 0xFD] per the memory table; the library treats IDs above 0xFC (MAX_ID) as not readable back
  REG_BAUD_RATE          = 0x06, // Baud Rate Setting                                 @details `0x06 (6)  | 1B | RW | EEPROM` @note values: `baud_rate_e`      @note unit:  enum
  REG_RESPONSE_DELAY     = 0x07, // Response (return) delay before the servo replies  @details `0x07 (7)  | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  2us
  REG_RESPONSE_MODE      = 0x08, // Response Mode: which commands are acknowledged    @details `0x08 (8)  | 1B | RW | EEPROM` @note values: `rsp_mode_t`       @note unit:  enum
  // Tuning Controls
  REG_CONTROL_TORQUE_NV  = 0x10, // Stored Maximum Control Torque                     @details `0x10 (16) | 2B | RW | EEPROM` @note values: [0,1000]             @note unit:  0.1%               @attention value copied to `REG_CTRL_TORQUE` at startup
  REG_PHASE              = 0x12, // SPECIAL FUNCTION BYTE                             @details `0x12 (18) | 1B | RW | EEPROM` @attention only write if other register description says to; bit meanings are not in the memory table (bit 4 = multi-turn feedback is used by the library, unverified)
  REG_CW_DEADZONE        = 0x1A, // Clockwise Deadzone                                @details `0x1A (26) | 1B | RW | EEPROM` @note values: [0,32]               @note unit:  1step
  REG_CCW_DEADZONE       = 0x1B, // Counter Clockwise Deadzone                        @details `0x1B (27) | 1B | RW | EEPROM` @note values: [0,32]               @note unit:  1step
  REG_ANGLE_RESOLUTION   = 0x1E, // Angle Resolution Amplification Factor             @details `0x1E (30) | 1B | RW | EEPROM` @note values: [1,100] per the memory table @note unit:  factor on the sensor's minimum resolution angle; larger values extend the number of turns
  REG_POS_OFFSET         = 0x1F, // Position Correction Offset                        @details `0x1F (31) | 2B | RW | EEPROM` @note values: [+-2047]             @note unit:  1step               @note bit[11] = direction   @see `REG_SERVO_EN` value 128
  // PID Control
  REG_POS_COEF_P         = 0x15, // Position Loop P Coefficient                       @details `0x15 (21) | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  1x
  REG_POS_COEF_D         = 0x16, // Position Loop D Coefficient                       @details `0x16 (22) | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  1x
  REG_POS_COEF_I         = 0x17, // Position Loop I Coefficient                       @details `0x17 (23) | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  1x
  REG_SPEED_COEF_P       = 0x25, // Speed Closed Loop P coefficient (speed mode)      @details `0x25 (37) | 1B | RW | EEPROM` @note values: [0,254] per the memory table @note unit:  1x
  REG_SPEED_COEF_I       = 0x27, // Speed Closed Loop I coefficient (speed mode)      @details `0x27 (39) | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  none stated in the memory table
  //ALARM Control
  REG_UNLOAD_ALARM_MASK  = 0x13, // Unloading Condition Alarm Mask                    @details `0x13 (19) | 1B | RW | EEPROM` @note values: `err_code_t` bits    @note unit:  bitmask (bit set = that fault releases torque)
  REG_LED_ALARM_MASK     = 0x14, // LED Indicator Alarm Mask                          @details `0x14 (20) | 1B | RW | EEPROM` @note values: `err_code_t` bits    @note unit:  bitmask (bit set = that fault flashes the LED)
  REG_MIN_ANGLE_THRESH    = 0x09, // Min Angle Limit (stroke minimum)                 @details `0x09 (9)  | 2B | RW | EEPROM` @note values: [0,4094] per the memory table @note unit:  1step   @note must be below the maximum; the memory table says 0 for multi-turn absolute control
  REG_MAX_ANGLE_THRESH    = 0x0B, // Max Angle Limit (stroke maximum)                 @details `0x0B (11) | 2B | RW | EEPROM` @note values: [1,4095] per the memory table @note unit:  1step   @note must be above the minimum; the memory table says 0 for multi-turn absolute control
  REG_OVERTEMP_THRESH    = 0x0D, // Maximum Temperature for alarm                     @details `0x0D (13) | 1B | RW | EEPROM` @note values: [0,100]              @note unit:  °C
  REG_OVERVOLT_THRESH    = 0x0E, // Maximum Voltage for alarm                         @details `0x0E (14) | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  0.1V
  REG_UNDERVOLT_THRESH   = 0x0F, // Minimum Voltage for alarm                         @details `0x0F (15) | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  0.1V
  REG_OVERCURR_THRESH    = 0x1C, // Maximum Current for alarm                         @details `0x1C (28) | 2B | RW | EEPROM` @note values: [0,511]              @note unit:  6.5mA
  REG_OVERCURR_TIME      = 0x26, // Duration for Protection Current to cause Alarm    @details `0x26 (38) | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  10ms
  REG_OVERLOAD_THRESH    = 0x24, // Maximum Load for alarm                            @details `0x24 (36) | 1B | RW | EEPROM` @note values: [0,100] meaningful (table range [0,254]) @note unit:  1.0% of maximum torque
  REG_OVERLOAD_TIME      = 0x23, // Duration for Overload Torque to cause alarm       @details `0x23 (35) | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  10 ms
  // Operation Control
  REG_START_TORQUE       = 0x18, // Minimum starting control Torque                   @details `0x18 (24) | 2B | RW | EEPROM` @note values: [0,1000]             @note unit:  0.1%
  REG_OPERATION_MODE     = 0x21, // Operating Mode Control                            @details `0x21 (33) | 1B | RW | EEPROM` @note values: `op_mode_t`         @note unit:  enum
  REG_UNLOAD_CTRL_TORQUE = 0x22, // Protective torque: output torque after an overload protection trip     @details `0x22 (34) | 1B | RW | EEPROM` @note values: [0,100] meaningful (table range [0,254]) @note unit:  1.0% of maximum torque
  REG_SERVO_EN           = 0x28, // Servo Enable / Disable (torque switch)            @details `0x28 (40) | 1B | RW | SRAM`   @note values: `servo_en_t`       @note unit: enum                @note value 128 calibrates the current position to 2048 (`REG_POS_OFFSET` correction)
  REG_CTRL_ACCEL         = 0x29, // Acceleration Control                              @details `0x29 (41) | 1B | RW | SRAM`   @note values: [0,254]              @note unit:  100 steps/sec^2
  REG_CTRL_POS           = 0x2A, // Position / STEP Control (target)                  @details `0x2A (42) | 2B | RW | SRAM`   @note values: [+-32766] per the memory table (the library limits itself to +-30719) @note unit:  1step @note bit[15] = direction (sign-magnitude)
  REG_CTRL_PWM           = 0x2C, // PWM Duty Cycle Control ("running time" in the memory table) @details `0x2C (44) | 2B | RW | SRAM` @note values: [0,1000]  @note unit:  0.1%               @note direction bit: bit[11] per the memory table, unverified
  REG_CTRL_SPEED         = 0x2E, // Speed Control                                     @details `0x2E (46) | 2B | RW | SRAM`   @note values: [0,3400] (no-load speed; the memory table range column says 254) @note unit:  steps/sec @note bit[15] = direction when `REG_OPERATION_MODE` == `OP_SPEED`
  REG_CTRL_TORQUE        = 0x30, // Maximum Control Torque                            @details `0x30 (48) | 2B | RW | SRAM`   @note values: [0,1000]             @note unit:  0.1%               @attention Takes value from `REG_CONTROL_TORQUE_NV` at startup
  // EEPROM Control
  REG_EEPROM_LOCK        = 0x37, // EEPROM Lock Flag                                  @details `0x37 (55) | 1B | RW | SRAM`   @note values: `eeprom_lock_t`     @note unit: enum  @note semantics unverified on hardware
  // Feedback / Status
  REG_POSITION           = 0x38, // Sensed Position                                   @details `0x38 (56) | 2B | RO | SRAM`                                     @note unit: 1step  @note bit[15] = sign (unverified)
  REG_SPEED              = 0x3A, // Sensed Speed                                      @details `0x3A (58) | 2B | RO | SRAM`                                     @note unit: steps/sec
  REG_LOAD               = 0x3C, // Sensed Load (drive duty cycle)                    @details `0x3C (60) | 2B | RO | SRAM`                                     @note unit: 0.1%  @note bit[10] = sign, magnitude 0..1000 (unverified)
  REG_VOLTAGE            = 0x3E, // Sensed Voltage                                    @details `0x3E (62) | 1B | RO | SRAM`                                     @note unit: 0.1V
  REG_TEMPERATURE        = 0x3F, // Sensed Temperature                                @details `0x3F (63) | 1B | RO | SRAM`                                     @note unit: °C
  REG_ASYNC_WRITE_FLAG   = 0x40, // Flag set when a deferred (async) write is pending @details `0x40 (64) | 1B | RO | SRAM`                                     @note unit: N/A
  REG_STATUS             = 0x41, // Servo Alarm Status                                @details `0x41 (65) | 1B | RO | SRAM`   @note values: `err_code_t` bits    @note unit: bitmask
  REG_MOVE_FLAG          = 0x42, // Movement Flag                                     @details `0x42 (66) | 1B | RO | SRAM`   @note values: `move_flag_t`       @note unit: enum
  REG_CURRENT            = 0x45, // Sensed Current                                    @details `0x45 (69) | 2B | RO | SRAM`                                     @note unit: 6.5mA  @note bit[15] = sign (unverified)
  // clang-format on
} reg_addr_e;

}  // namespace STS
}  // namespace servokit
#endif  // STSERIES_DEFINES__H
