#ifndef STSERIES_DEFINES__H
#define STSERIES_DEFINES__H

#include <cstdint>
#include <cstdio>

namespace esp_serial_servo {

namespace STS {

constexpr uint16_t LIB_FW_VERSION = 0x0307;  // FW 3.7 (0x03_07)
constexpr uint16_t SYNC_PATTERN = 0xFFFF;    // Sync Pattern for ST servos
constexpr uint8_t MAX_ID = 0xFC;             // Maximum usable Device ID
constexpr uint8_t BROADCAST_ID = 0xFE;       // Broadcast ID (no response)

constexpr uint8_t SYNC_IDX = 0;
constexpr uint8_t DEVID_IDX = 2;
constexpr uint8_t LENGTH_IDX = 3;
constexpr uint8_t CODE_IDX = 4;
constexpr uint8_t DATA_IDX = 5;

constexpr uint8_t NUM_SYNC_BYTES   = 2;
constexpr uint8_t NUM_DEVID_BYTES  = 1;
constexpr uint8_t NUM_LENGTH_BYTES = 1;
constexpr uint8_t NUM_CODE_BYTES  = 1;
constexpr uint8_t NUM_CHKSUM_BYTES = 1;
constexpr uint8_t NUM_META_BYTES = 6; // Sum of all metadata bytes


// STSeries Command Instructions
typedef enum {
  CMD_PING = 0x01,
  CMD_READ = 0x02,
  CMD_WRITE = 0x03,
  CMD_REG_WRITE = 0x04,
  CMD_ACTION = 0x05,
  CMD_RESET = 0x06,
  CMD_SYNC_READ = 0x82,  // Not documented, but in python lib
  CMD_SYNC_WRITE = 0x83
} cmd_e;

// STSeries Alarm / Status Codes
typedef enum {
  STAT_OK = 0x00,             // No Error
  STAT_VOLTAGE = 1 << 0,      // Voltage Error
  STAT_SENSOR = 1 << 1,       // Sensor Error
  STAT_TEMPERATURE = 1 << 2,  // Temperature Error
  STAT_CURRENT = 1 << 3,      // Current Error
  STAT_ANGLE = 1 << 3,        // Angle Error
  STAT_OVERLOAD = 1 << 5      // Overload Error
} err_code_t;

// Constants for `REG_RESPONSE_MODE`
typedef enum {
  RSP_READ_PING = 0,  /// Respond to READ and PING commands only
  RSP_ALL = 1         /// Respond to all commands
} rsp_mode_t;

// Constants for `REG_OPERATION_MODE`
typedef enum {
  OP_SERVO = 0,  // Normal Servo Mode      @note controlled by `REG_CTRL_POS`
  OP_SPEED = 1,  // Constant Speed Mode    @note controlled by `REG_CTRL_SPEED`
  OP_PWM = 2,    // Constant PWM Mode      @note controlled by `REG_CTRL_PWM`
  OP_STEP = 3,   // Stepper Mode           @note controlled by `REG_CTRL_POS`
} op_mode_t;

// Constants for `REG_BAUD_RATE`
typedef enum {
  BAUD_1000000 = 0,  // 1000  Kbps
  BAUD_500000 = 1,   // 500   Kbps
  BAUD_250000 = 2,   // 250   Kbps
  BAUD_128000 = 3,   // 128   Kbps
  BAUD_115200 = 4,   // 115.2 Kbps
  BAUD_76800 = 5,    // 76.8  Kbps
  BAUD_57600 = 6,    // 57.6  Kbps
  BAUD_38400 = 7     // 38.4  Kbps
} baud_rate_e;

// Constants for `REG_SERVO_EN`
typedef enum { SERVO_DISABLE = 0, SERVO_ENABLE = 1 } servo_en_t;

// Constants for `REG_EEPROM_LOCK`
typedef enum { EEPROM_UNLOCKED = 0, EEPROM_LOCKED = 1 } eeprom_lock_t;

// Constants for `REG_MOVE_FLAG`
typedef enum { SERVO_STOPPED = 0, SERVO_MOVING = 1 } move_flag_t;

// STS Register Map
// @todo REG_PHASE bit 4 should be set when performing multi turn contorl to enable angle resolution feedback
typedef enum {
  // clang-format off
  // Manufacturing Data
  REG_VER_MAJ            = 0x00, // Firmware Major Version                            @details `0x00 (0)  | 1B | RO | EEPROM`
  REG_VER_SUB            = 0x01, // Firmware Minor Version                            @details `0x01 (1)  | 1B | RO | EEPROM`
  REG_MODEL_MAJ          = 0x03, // HW Major Version                                  @details `0x03 (3)  | 1B | RO | EEPROM`
  REG_MODEL_SUB          = 0x04, // HW Minor Version                                  @details `0x04 (4)  | 1B | RO | EEPROM`
  // Communications
  REG_DEVICE_ID          = 0x05, // Device ID                                         @details `0x05 (5)  | 1B | RW | EEPROM` @note values: [0, 0xFC]
  REG_BAUD_RATE          = 0x06, // Baud Rate Setting                                 @details `0x06 (6)  | 1B | RW | EEPROM` @note values: `sts_baud_rate_t`    @note unit:  enum
  REG_RESPONSE_DELAY     = 0x07, // Response Delay                                    @details `0x07 (7)  | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  2us
  REG_RESPONSE_MODE      = 0x08, // Response Mode                                     @details `0x08 (8)  | 1B | RW | EEPROM` @note values: `sts_rsp_mode_t`     @note unit:  enum
  // Tuning Controls
  REG_CONTROL_TORQUE_NV  = 0x10, // Stored Maximum Control Torque                     @details `0x10 (16) | 2B | RW | EEPROM` @note values: [0,1000]             @note unit:  0.1%               @attention value copied to `REG_CONTROL_TORQUE` at startup
  REG_PHASE              = 0x12, // SPECIAL FUNCTION BYTE                             @details `0x12 (18) | 1B | RW | EEPROM` @attention only write if other register description says to
  REG_CW_DEADZONE        = 0x1A, // Clockwise Deadzone                                @details `0x1A (26) | 1B | RW | EEPROM` @note values: [0,32]               @note unit:  1step
  REG_CCW_DEADZONE       = 0x1B, // Counter Clockwise Deadzone                        @details `0x1B (27) | 1B | RW | EEPROM` @note values: [0,32]               @note unit:  1step
  REG_ANGLE_RESOLUTION   = 0x1E, // Angle Resolution Amplification Factor             @details `0x1E (30) | 1B | RW | EEPROM` @note values: [1,3]                @note unit:  N/A    @note
  REG_POS_OFFSET         = 0x1F, // Position Correction Offset                        @details `0x1F (31) | 2B | RW | EEPROM` @note values: [+-2047]             @note unit:  1step               @note bit[11] = direction   @see `REG_TORQUE_EN` bit7
  // PID Control
  REG_POS_COEF_P         = 0x15, // Position Loop P Coefficient                       @details `0x15 (21) | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  1x
  REG_POS_COEF_D         = 0x16, // Position Loop D Coefficient                       @details `0x16 (22) | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  1x
  REG_POS_COEF_I         = 0x17, // Position Loop I Coefficient                       @details `0x17 (23) | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  1x
  REG_SPEED_COEF_P       = 0x25, // Speed Closed Loop P coefficient                   @details `0x25 (37) | 1B | RW | EEPROM` @note values: [0,100]              @note unit:  1x
  REG_SPEED_COEF_I       = 0x27, // Speed Closed Loop I coefficient                   @details `0x27 (39) | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  1/10x
  //ALARM Control
  REG_UNLOAD_ALARM_MASK  = 0x13, // Unloading Condition Alarm Mask                    @details `0x13 (19) | 1B | RW | EEPROM` @note values: `sts_err_code_t`     @note unit:  bitmask
  REG_LED_ALARM_MASK     = 0x14, // LED Indicator Alarm Mask                          @details `0x14 (20) | 1B | RW | EEPROM` @note values: `sts_err_code_t`     @note unit:  bitmask
  REG_MIN_ANGLE_THRESH    = 0x09, // Min Angle Range for alarm                        @details `0x09 (9)  | 2B | RW | EEPROM` @note values: (+-32767]            @note unit:  1step                     @note 0 = no limit
  REG_MAX_ANGLE_THRESH    = 0x0B, // Max Angle Range for alarm                        @details `0x0B (11) | 2B | RW | EEPROM` @note values: (+-32767]            @note unit:  1step                     @note 0 = no limit
  REG_OVERTEMP_THRESH    = 0x0D, // Maximum Temperature for alarm                     @details `0x0D (13) | 1B | RW | EEPROM` @note values: [0,100]              @note unit:  °C
  REG_OVERVOLT_THRESH    = 0x0E, // Maximum Voltage for alarm                         @details `0x0E (14) | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  0.1V
  REG_UNDERVOLT_THRESH   = 0x0F, // Maximum Voltage for alarm                         @details `0x0F (15) | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  0.1V
  REG_OVERCURR_THRESH    = 0x1C, // Maximum Current for alarm                         @details `0x1C (28) | 2B | RW | EEPROM` @note values: [0,511]              @note unit:  6.5mA
  REG_OVERCURR_TIME      = 0x26, // Duration for Protection Current to cause Alarm    @details `0x26 (38) | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  10ms
  REG_OVERLOAD_THRESH    = 0x24, // Maximum Load for alarm                            @details `0x24 (36) | 1B | RW | EEPROM` @note values: [0,100]              @note unit:  1.0%
  REG_OVERLOAD_TIME      = 0x23, // Duration for Overload Torque to cause alarm       @details `0x23 (35) | 1B | RW | EEPROM` @note values: [0,254]              @note unit:  10 ms
  // Operation Control
  REG_START_TORQUE       = 0x18, // Minimum starting control Torque                   @details `0x18 (24) | 2B | RW | EEPROM` @note values: [0,1000]             @note unit:  0.1%
  REG_OPERATION_MODE     = 0x21, // Operating Mode Control                            @details `0x21 (33) | 1B | RW | EEPROM` @note values: `sts_op_mode_t`      @note unit:  enum
  REG_UNLOAD_CTRL_TORQUE = 0x22, // Maximum Control Torque for Unloaded condition     @details `0x22 (34) | 1B | RW | EEPROM` @note values: [0,100]              @note unit:  1.0%
  REG_SERVO_EN           = 0x28, // Servo Enable / Disable                            @details `0x28 (40) | 1B | RW | SRAM`   @note values: `sts_servo_en_t`     @note unit: enum                @note bit[7]: arbitrary `REG_POS_OFFSET` correction to 2048.
  REG_CTRL_ACCEL         = 0x29, // Accelleration Control                             @details `0x29 (41) | 1B | RW | SRAM`   @note values: [0,254]              @note unit:  100 steps/sec^2
  REG_CTRL_POS           = 0x2A, // Position / STEP Control                           @details `0x2A (42) | 2B | RW | SRAM`   @note values: [+-30719]            @note unit:  1step              @note bit[15] = direction when `REG_OP_MODE` == `OP_STEP`
  REG_CTRL_PWM           = 0x2C, // PWM Duty Cycle Control                            @details `0x2C (44) | 2B | RW | SRAM`   @note values: [0,1000]             @note unit:  0.1%               @note bit[10] = direction
  REG_CTRL_SPEED         = 0x2E, // Speed Control                                     @details `0x2E (46) | 2B | RW | SRAM`   @note values: [0,3400]             @note unit:  steps/sec          @note bit[15] = direction when `REG_OP_MODE` == `OP_SPEED`
  REG_CTRL_TORQUE        = 0x30, // Maximum Control Torque                            @details `0x30 (48) | 2B | RW | SRAM`   @note values: [0,1000]             @note unit:  0.1%               @attention Takes value from `REG_CONTROL_TORQUE_NV` at startup
  // EEPROM Control
  REG_EEPROM_LOCK        = 0x37, // EEPROM Lock Flag                                  @details `0x37 (55) | 1B | RW | SRAM`   @note values: `sts_eeprom_lock_t`  @note unit: enum
  // Feedback / Status
  REG_POSITION           = 0x38, // Sensed Position                                   @details `0x38 (56) | 2B | RO | SRAM`                                     @note unit: 1step
  REG_SPEED              = 0x3A, // Sensed Speed                                      @details `0x3A (58) | 2B | RO | SRAM`                                     @note unit: steps/sec
  REG_LOAD               = 0x3C, // Sensed Load                                       @details `0x3C (60) | 2B | RO | SRAM`                                     @note unit: 0.1%
  REG_VOLTAGE            = 0x3E, // Sensed Voltage                                    @details `0x3E (62) | 1B | RO | SRAM`                                     @note unit: 0.1V
  REG_TEMPERATURE        = 0x3F, // Sensed Temperature                                @details `0x3F (63) | 1B | RO | SRAM`                                     @note unit: °C
  REG_ASYNC_WRITE_FLAG   = 0x40, // Async Write Flag??                                @details `0x40 (64) | 1B | RO | SRAM`                                     @note unit: N/A
  REG_STATUS             = 0x41, // Servo Alarm Status                                @details `0x41 (65) | 1B | RO | SRAM`   @note values: `sts_err_code_t`    @note unit: bitmask
  REG_MOVE_FLAG          = 0x42, // Movement Flag                                     @details `0x42 (66) | 1B | RO | SRAM`   @note values: `sts_moving_flag_t` @note unit: enum
  REG_CURRENT            = 0x45, // Sensed Current                                    @details `0x45 (69) | 2B | RO | SRAM`                                     @note unit: 6.5mA
  // clang-format on
} reg_addr_e;

}  // namespace STS
}  // namespace esp_serial_servo
#endif  // STSERIES_DEFINES__H