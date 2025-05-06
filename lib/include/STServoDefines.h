#ifndef STSERVODEFINES__H
#define STSERVODEFINES__H

static const uint8_t STS_MAX_ID = 0xFC;
static const uint8_t STS_BROADCAST_ID = 0xFE;
static const uint16_t STS_SYNC_PATTERN = 0xFFFF;
static const std::endian STS_ENDIANNESS = std::endian::little;

// STS Command Instructions
typedef enum {
  STS_CMD_PING = 0x01,
  STS_CMD_READ = 0x02,
  STS_CMD_WRITE = 0x03,
  STS_CMD_REG_WRITE = 0x04,
  STS_CMD_ACTION = 0x05,
  STS_CMD_RESET = 0x06,
  STS_CMD_SYNC_READ = 0x82,  // Not documented, but in python lib
  STS_CMD_SYNC_WRITE = 0x83
} sts_cmd_e;

// Python code says:   VOLT, ANGLE, TEMP, CURRENT, OVERLOAD
// Memory Table Says:  VOLT, SENSE, TEMP, CURRENT, ANGLE, OVERLOAD
// LED_ALARM says:     TEMP, CURRENT, ANGLER, OVERLOAD, VOLTAGE

// STS Response Error codes
typedef enum {
  STS_ERR_NONE = 0x00,           // No Error
  STS_ERR_VOLTAGE = 1 << 0,      // Input voltage Error
  STS_ERR_ANGLE = 1 << 1,        // Angle Sensor Error
  STS_ERR_OVERHEAT = 1 << 2,     // Overheat Error
  STS_ERR_OVERCURRENT = 1 << 3,  // Overcurrent Error
  STS_ERR_OVERLOAD = 1 << 5      // Overload Error
} sts_err_code_t;

// Baud Rate Definitions for STS_BAUD_RATE register
typedef enum {
  STS_1000000 = 0,  // 1M
  STS_500000 = 1,   // 500K
  STS_250000 = 2,   // 250K
  STS_128000 = 3,   // 128K
  STS_115200 = 4,   // 115.2K
  STS_76800 = 5,    // 76.8K
  STS_57600 = 6,    // 57.6K
  STS_38400 = 7     // 38.4K
} sts_baud_rate_e;

// STS Register Map

//| Address | Name                    | Type    | Size   | Desc
//|:-------:|-------------------------|:-------:|:------:|---------------------
//| 0x00 (0)  | STS_VER_MAJ           | EEPROM  | 1      |
//| 0x01 (1)  | STS_VER_SUB           | EEPROM  | 1      |
//| 0x03 (3)  | STS_MODEL_MAJ         | EEPROM  | 1      |
//| 0x04 (4)  | STS_MODEL_SUB         | EEPROM  | 1      |
//| 0x05 (5)  | STS_DEVID             | EEPROM  | 1      |
//| 0x06 (6)  | STS_BAUD_RATE         | EEPROM  | 1      |
//| 0x07 (7)  | STS_RSP_DELAY         | EEPROM  | 1      |
//| 0x08 (8)  | STS_RSP_MODE          | EEPROM  | 1      |
//| 0x09 (9)  | STS_MIN_ANGLE         | EEPROM  | 2      |
//| 0x0B (11) | STS_MAX_ANGLE         | EEPROM  | 2      |
//| 0x0D (13) | STS_MAX_TEMP          | EEPROM  | 1      |
//| 0x0E (14) | STS_MAX_VOLT          | EEPROM  | 1      |
//| 0x0F (15) | STS_MIN_VOLT          | EEPROM  | 1      |
//| 0x10 (16) | STS_MAX_TORQUE        | EEPROM  | 2      |
//| 0x12 (18) | PHASE                 | EEPROM  | 1      |
//| 0x13 (19) | STS_UNLOAD_CONDITION  | EEPROM  | 1      |
//| 0x14 (20) | STS_ALARM_CONDITION   | EEPROM  | 1      |
//| 0x15 (21) | STS_COEF_P            | EEPROM  | 1      |
//| 0x16 (22) | STS_COEF_D            | EEPROM  | 1      |
//| 0x17 (23) | STS_COEF_I            | EEPROM  | 1      |
//| 0x18 (24) | STS_MIN_START_FORCE   | EEPROM  | 2      |
//| 0x1A (26) | STS_CW_DEADBAND       | EEPROM  | 1      |
//| 0x1B (27) | STS_CCW_DEADBAND      | EEPROM  | 1      |
//| 0x1C (28) | STS_CURRENT_LIMIT     | EEPROM  | 2      |
//| 0x1E (30) | STS_ANGLE_RES         | EEPROM  | 1      |
//| 0x1F (31) | STS_POS_OFFSET        | EEPROM  | 2      |
//| 0x21 (33) | STS_MOVE_MODE         | EEPROM  | 1      |
//| 0x22 (34) | STS_PROTECTION_TORQUE | EEPROM  | 1      |
//| 0x23 (35) | STS_PROTECTION_TIME   | EEPROM  | 1      |
//| 0x24 (36) | STS_OVERLOAD_TORQUE   | EEPROM  | 1      |
//| 0x25 (37) | STS_COEF_P_CLOSED     | EEPROM  | 1      |
//| 0x26 (38) | STS_OVERCURRENT_TIME  | EEPROM  | 1      |
//| 0x27 (39) | STS_COEF_I_CLOSED     | EEPROM  | 1      |
//| 0x28 (40) | STS_TORQUE_EN         | SRAM    | 1      |
//| 0x29 (41) | STS_TARGET_ACCEL      | SRAM    | 1      |
//| 0x2A (42) | STS_TARGET_POS        | SRAM    | 2      | Controls position/steps in position/step control mode
//| 0x2C (44) | STS_TARGET_TIME       | SRAM    | 2      | Controls time param in open loop pwm mode
//| 0x2E (46) | STS_TARGET_SPEED      | SRAM    | 2      | Controls speed in constant speed mode
//| 0x30 (48) | STS_TORQUE_LIMIT      | SRAM    | 2      |
//| 0x37 (55) | STS_EEPROM_LOCK       | SRAM    | 1      |
//| 0x38 (56) | STS_POSITION          | SRAM    | 2      |
//| 0x3A (58) | STS_SPEED             | SRAM    | 2      |
//| 0x3C (60) | STS_LOAD              | SRAM    | 2      |
//| 0x3E (62) | STS_VOLTAGE           | SRAM    | 1      |
//| 0x3F (63) | STS_TEMPERATURE       | SRAM    | 1      |
//| 0x40 (64) | STS_ASYNC_WRITE_FLAG  | SRAM    | 1      |
//| 0x41 (65) | STS_STATUS            | SRAM    | 1      |
//| 0x42 (66) | STS_MOVING_FLAG       | SRAM    | 1      |
//| 0x45 (69) | STS_CURRENT           | SRAM    | 2      |
//|:---------:|-----------------------|:-------:|:------:|---------------------
typedef enum {
  STS_VER_MAJ             = 0x00, // Firmware Major Version
  STS_VER_SUB             = 0x01, // Firmware Minor Version
  STS_MODEL_MAJ           = 0x03, // Model Version Major
  STS_MODEL_SUB           = 0x04, // Model Version Minor
  STS_DEVID               = 0x05, // Device ID
  STS_BAUD_RATE           = 0x06, // Baud Rate (See `sts_baud_rate_e`)
  STS_RSP_DELAY           = 0x07,
  STS_RSP_MODE            = 0x08,
  STS_MIN_ANGLE           = 0x09,
  STS_MAX_ANGLE           = 0x0B,
  STS_MAX_TEMP            = 0x0D,
  STS_MAX_VOLT            = 0x0E,
  STS_MIN_VOLT            = 0x0F,
  STS_MAX_TORQUE          = 0x10,
  PHASE                   = 0x12,
  STS_UNLOAD_CONDITION    = 0x13,
  STS_ALARM_CONDITION     = 0x14,
  STS_COEF_P              = 0x15,
  STS_COEF_D              = 0x16,
  STS_COEF_I              = 0x17,
  STS_STARTING_FORCE      = 0x18,
  STS_CW_DEADBAND         = 0x1A,
  STS_CCW_DEADBAND        = 0x1B,
  STS_CURRENT_LIMIT       = 0x1C,
  STS_ANGLE_RES           = 0x1E,
  STS_POS_OFFSET          = 0x1F,
  STS_MOVE_MODE           = 0x21,
  STS_PROTECTION_TORQUE   = 0x22,
  STS_PROTECTION_TIME     = 0x23,
  STS_OVERLOAD_TORQUE     = 0x24,
  STS_COEF_P_CLOSED       = 0x25,
  STS_OVERCURRENT_TIME    = 0x26,
  STS_COEF_I_CLOSED       = 0x27,
  STS_TORQUE_EN           = 0x28,
  STS_TARGET_ACCEL        = 0x29,
  STS_TARGET_POS          = 0x2A,   //controls position/steps in position/step control mode
  STS_TARGET_TIME         = 0x2C,   //controlls time param in open loop pwm mode
  STS_TARGET_SPEED        = 0x2E,   //Controls speed in constant speed mode
  STS_TORQUE_LIMIT        = 0x30,
  STS_EEPROM_LOCK         = 0x37,
  STS_POSITION            = 0x38,
  STS_SPEED               = 0x3A,
  STS_LOAD                = 0x3C,
  STS_VOLTAGE             = 0x3E,
  STS_TEMPERATURE         = 0x3F,
  STS_ASYNC_WRITE_FLAG    = 0x40,
  STS_STATUS              = 0x41,
  STS_MOVING_FLAG         = 0x42,
  STS_CURRENT             = 0x45
} sts_reg_addr_e;

#endif  // STSERVODEFINES__H