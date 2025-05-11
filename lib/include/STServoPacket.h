#ifndef STSERVOPACKET__H
#define STSERVOPACKET__H

#include <cstdint>

#include "ServoPacket.h"

class STServoPacket : public ServoPacket {
 public:
  STServoPacket() = default;
  STServoPacket(const buffer_t& buffer);

  virtual buffer_t sync_buffer();
  virtual buffer_t data_buffer();
  virtual size_t len_index();

  virtual int reset();

  virtual bool is_well_formed();

  uint8_t compute_checksum();
  uint8_t compute_length();

  uint16_t get_sync();
  uint8_t get_device_id();
  uint8_t get_length();
  uint8_t get_code();
  buffer_const_t get_data();
  uint8_t get_checksum();

  int set_sync(uint16_t val);
  int set_device_id(uint8_t val);
  int set_length(uint8_t val);
  int set_code(uint8_t val);
  int set_data(const buffer_const_t val);
  int set_checksum(uint8_t val);
  int update_length();
  int update_checksum();

  int init(uint16_t sync, uint8_t dev_id, uint8_t length, uint8_t code, const buffer_const_t data, uint8_t checksum);
  int init(uint8_t dev_id, uint8_t code, const buffer_const_t data);

 private:
  inline uint8_t get_byte(uint8_t idx, uint8_t default_val = 0x00);
  inline uint16_t get_word(uint8_t idx, uint16_t default_val = 0x0000);

  inline int set_byte(size_t idx, uint8_t val);
  inline int set_word(size_t idx, uint16_t val);
};

static const uint8_t STS_MAX_ID = 0xFC;
static const uint8_t STS_BROADCAST_ID = 0xFE;
static const uint16_t STS_SYNC_PATTERN = 0xFFFF;

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
  STS_1000000 = 0,  // 1000  Kbps
  STS_500000 = 1,   // 500   Kbps
  STS_250000 = 2,   // 250   Kbps
  STS_128000 = 3,   // 128   Kbps
  STS_115200 = 4,   // 115.2 Kbps
  STS_76800 = 5,    // 76.8  Kbps
  STS_57600 = 6,    // 57.6  Kbps
  STS_38400 = 7     // 38.4  Kbps
} sts_baud_rate_e;

// STS Register Map
typedef enum {
  STS_VER_MAJ = 0x00,            //| 0x00 (0)  | 1 Byte | STS_VER_MAJ           | EEPROM  |
  STS_VER_SUB = 0x01,            //| 0x01 (1)  | 1 Byte | STS_VER_SUB           | EEPROM  |
  STS_MODEL_MAJ = 0x03,          //| 0x03 (3)  | 1 Byte | STS_MODEL_MAJ         | EEPROM  |
  STS_MODEL_SUB = 0x04,          //| 0x04 (4)  | 1 Byte | STS_MODEL_SUB         | EEPROM  |
  STS_DEVID = 0x05,              //| 0x05 (5)  | 1 Byte | STS_DEVID             | EEPROM  |
  STS_BAUD_RATE = 0x06,          //| 0x06 (6)  | 1 Byte | STS_BAUD_RATE         | EEPROM  |
  STS_RSP_DELAY = 0x07,          //| 0x07 (7)  | 1 Byte | STS_RSP_DELAY         | EEPROM  |
  STS_RSP_MODE = 0x08,           //| 0x08 (8)  | 1 Byte | STS_RSP_MODE          | EEPROM  |
  STS_MIN_ANGLE = 0x09,          //| 0x09 (9)  | 2 Byte | STS_MIN_ANGLE         | EEPROM  |
  STS_MAX_ANGLE = 0x0B,          //| 0x0B (11) | 2 Byte | STS_MAX_ANGLE         | EEPROM  |
  STS_MAX_TEMP = 0x0D,           //| 0x0D (13) | 1 Byte | STS_MAX_TEMP          | EEPROM  |
  STS_MAX_VOLT = 0x0E,           //| 0x0E (14) | 1 Byte | STS_MAX_VOLT          | EEPROM  |
  STS_MIN_VOLT = 0x0F,           //| 0x0F (15) | 1 Byte | STS_MIN_VOLT          | EEPROM  |
  STS_MAX_TORQUE = 0x10,         //| 0x10 (16) | 2 Byte | STS_MAX_TORQUE        | EEPROM  |
  PHASE = 0x12,                  //| 0x12 (18) | 1 Byte | PHASE                 | EEPROM  |
  STS_UNLOAD_CONDITION = 0x13,   //| 0x13 (19) | 1 Byte | STS_UNLOAD_CONDITION  | EEPROM  |
  STS_ALARM_CONDITION = 0x14,    //| 0x14 (20) | 1 Byte | STS_ALARM_CONDITION   | EEPROM  |
  STS_COEF_P = 0x15,             //| 0x15 (21) | 1 Byte | STS_COEF_P            | EEPROM  |
  STS_COEF_D = 0x16,             //| 0x16 (22) | 1 Byte | STS_COEF_D            | EEPROM  |
  STS_COEF_I = 0x17,             //| 0x17 (23) | 1 Byte | STS_COEF_I            | EEPROM  |
  STS_STARTING_FORCE = 0x18,     //| 0x18 (24) | 2 Byte | STS_MIN_START_FORCE   | EEPROM  |
  STS_CW_DEADBAND = 0x1A,        //| 0x1A (26) | 1 Byte | STS_CW_DEADBAND       | EEPROM  |
  STS_CCW_DEADBAND = 0x1B,       //| 0x1B (27) | 1 Byte | STS_CCW_DEADBAND      | EEPROM  |
  STS_CURRENT_LIMIT = 0x1C,      //| 0x1C (28) | 2 Byte | STS_CURRENT_LIMIT     | EEPROM  |
  STS_ANGLE_RES = 0x1E,          //| 0x1E (30) | 1 Byte | STS_ANGLE_RES         | EEPROM  |
  STS_POS_OFFSET = 0x1F,         //| 0x1F (31) | 2 Byte | STS_POS_OFFSET        | EEPROM  |
  STS_MOVE_MODE = 0x21,          //| 0x21 (33) | 1 Byte | STS_MOVE_MODE         | EEPROM  |
  STS_PROTECTION_TORQUE = 0x22,  //| 0x22 (34) | 1 Byte | STS_PROTECTION_TORQUE | EEPROM  |
  STS_PROTECTION_TIME = 0x23,    //| 0x23 (35) | 1 Byte | STS_PROTECTION_TIME   | EEPROM  |
  STS_OVERLOAD_TORQUE = 0x24,    //| 0x24 (36) | 1 Byte | STS_OVERLOAD_TORQUE   | EEPROM  |
  STS_COEF_P_CLOSED = 0x25,      //| 0x25 (37) | 1 Byte | STS_COEF_P_CLOSED     | EEPROM  |
  STS_OVERCURRENT_TIME = 0x26,   //| 0x26 (38) | 1 Byte | STS_OVERCURRENT_TIME  | EEPROM  |
  STS_COEF_I_CLOSED = 0x27,      //| 0x27 (39) | 1 Byte | STS_COEF_I_CLOSED     | EEPROM  |
  STS_TORQUE_EN = 0x28,          //| 0x28 (40) | 1 Byte | STS_TORQUE_EN         | SRAM    |
  STS_TARGET_ACCEL = 0x29,       //| 0x29 (41) | 1 Byte | STS_TARGET_ACCEL      | SRAM    |
  STS_TARGET_POS = 0x2A,         //| 0x2A (42) | 2 Byte | STS_TARGET_POS        | SRAM    |
  STS_TARGET_TIME = 0x2C,        //| 0x2C (44) | 2 Byte | STS_TARGET_TIME       | SRAM    |
  STS_TARGET_SPEED = 0x2E,       //| 0x2E (46) | 2 Byte | STS_TARGET_SPEED      | SRAM    |
  STS_TORQUE_LIMIT = 0x30,       //| 0x30 (48) | 2 Byte | STS_TORQUE_LIMIT      | SRAM    |
  STS_EEPROM_LOCK = 0x37,        //| 0x37 (55) | 1 Byte | STS_EEPROM_LOCK       | SRAM    |
  STS_POSITION = 0x38,           //| 0x38 (56) | 2 Byte | STS_POSITION          | SRAM    |
  STS_SPEED = 0x3A,              //| 0x3A (58) | 2 Byte | STS_SPEED             | SRAM    |
  STS_LOAD = 0x3C,               //| 0x3C (60) | 2 Byte | STS_LOAD              | SRAM    |
  STS_VOLTAGE = 0x3E,            //| 0x3E (62) | 1 Byte | STS_VOLTAGE           | SRAM    |
  STS_TEMPERATURE = 0x3F,        //| 0x3F (63) | 1 Byte | STS_TEMPERATURE       | SRAM    |
  STS_ASYNC_WRITE_FLAG = 0x40,   //| 0x40 (64) | 1 Byte | STS_ASYNC_WRITE_FLAG  | SRAM    |
  STS_STATUS = 0x41,             //| 0x41 (65) | 1 Byte | STS_STATUS            | SRAM    |
  STS_MOVING_FLAG = 0x42,        //| 0x42 (66) | 1 Byte | STS_MOVING_FLAG       | SRAM    |
  STS_CURRENT = 0x45             //| 0x45 (69) | 2 Byte | STS_CURRENT           | SRAM    |
} sts_reg_addr_e;

#endif  // STSERVOPACKET__H