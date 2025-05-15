#ifndef STSERIES_UTILS__H
#define STSERIES_UTILS__H
#include "serial_servo/base/servo_controller.h"
#include "serial_servo/stseries/stseries_packet.h"

namespace esp_serial_servo {

void dump_stseries_reg(ServoController& ctrl, STSeriesPacket& cmd, STSeriesPacket& rsp, uint8_t id, uint8_t addr,
                      uint8_t bytes, const char* name) {
  int retry;
  srv_result_t result;
  cmd.init(id, STS::CMD_READ, std::array<uint8_t, 2>{addr, bytes});
  for (retry = 0; retry < 5; retry++) {
    result = ctrl.send_command(&cmd);
    if (get_status(result) != SRV_OK) continue;
    result = ctrl.read_response(&rsp);
    if (get_status(result) != SRV_OK) continue;
    break;
  }

  if (retry >= 5) {
    printf("  %-25s:  FAILED\n", name);
  } else if (bytes == 1) {
    uint8_t val = rsp.get_data()[0];
    printf("  %-25s:  %5d    0x%02x\n", name, val, val);
  } else {
    uint8_t val0 = rsp.get_data()[0];
    uint8_t val1 = rsp.get_data()[1];
    uint16_t val = (uint16_t)(val1) << 8 | val0;
    printf("  %-25s:  %5d    0x%04x     [0x%02x  0x%02x]\n", name, val, val, val0, val1);
  }
}

void dump_stseries(ServoController& ctrl, uint8_t id) {
  uint8_t cmd_buff[15];
  STSeriesPacket cmd(cmd_buff);
  uint8_t rsp_buff[15];
  STSeriesPacket rsp(rsp_buff);

  printf("\nManufacturing Data:\n");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_VER_MAJ, 1, "REG_VER_MAJ");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_VER_SUB, 1, "REG_VER_SUB");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_MODEL_MAJ, 1, "REG_MODEL_MAJ");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_MODEL_SUB, 1, "REG_MODEL_SUB");
  printf("\nCommunications:\n");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_DEVICE_ID, 1, "REG_DEVICE_ID");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_BAUD_RATE, 1, "REG_BAUD_RATE");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_RESPONSE_DELAY, 1, "REG_RESPONSE_DELAY");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_RESPONSE_MODE, 1, "REG_RESPONSE_MODE");
  printf("\nTuning Controls:\n");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_CONTROL_TORQUE_NV, 2, "REG_CONTROL_TORQUE_NV");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_PHASE, 1, "REG_PHASE");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_CW_DEADZONE, 1, "REG_CW_DEADZONE");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_CCW_DEADZONE, 1, "REG_CCW_DEADZONE");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_ANGLE_RESOLUTION, 1, "REG_ANGLE_RESOLUTION");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_POS_OFFSET, 2, "REG_POS_OFFSET");
  printf("\nPID Control:\n");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_POS_COEF_P, 1, "REG_POS_COEF_P");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_POS_COEF_D, 1, "REG_POS_COEF_D");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_POS_COEF_I, 1, "REG_POS_COEF_I");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_SPEED_COEF_P, 1, "REG_SPEED_COEF_P");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_SPEED_COEF_I, 1, "REG_SPEED_COEF_I");
  printf("\nALARM Control:\n");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_UNLOAD_ALARM_MASK, 1, "REG_UNLOAD_ALARM_MASK");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_LED_ALARM_MASK, 1, "REG_LED_ALARM_MASK");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_MIN_ANGLE_THRESH, 2, "REG_MIN_ANGLE_THRESH");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_MAX_ANGLE_THRESH, 2, "REG_MAX_ANGLE_THRESH");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_OVERTEMP_THRESH, 1, "REG_OVERTEMP_THRESH");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_OVERVOLT_THRESH, 1, "REG_OVERVOLT_THRESH");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_UNDERVOLT_THRESH, 1, "REG_UNDERVOLT_THRESH");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_OVERCURR_THRESH, 2, "REG_OVERCURR_THRESH");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_OVERCURR_TIME, 1, "REG_OVERCURR_TIME");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_OVERLOAD_THRESH, 1, "REG_OVERLOAD_THRESH");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_OVERLOAD_TIME, 1, "REG_OVERLOAD_TIME");
  printf("\nOperation Control:\n");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_START_TORQUE, 2, "REG_START_TORQUE");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_OPERATION_MODE, 1, "REG_OPERATION_MODE");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_UNLOAD_CTRL_TORQUE, 1, "REG_UNLOAD_CTRL_TORQUE");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_SERVO_EN, 1, "REG_SERVO_EN");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_CTRL_ACCEL, 1, "REG_CTRL_ACCEL");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_CTRL_POS, 2, "REG_CTRL_POS");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_CTRL_PWM, 2, "REG_CTRL_PWM");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_CTRL_SPEED, 2, "REG_CTRL_SPEED");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_CTRL_TORQUE, 2, "REG_CTRL_TORQUE");
  printf("\nEEPROM Control:\n");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_EEPROM_LOCK, 1, "REG_EEPROM_LOCK");
  printf("\nFeedback / Status:\n");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_POSITION, 2, "REG_POSITION");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_SPEED, 2, "REG_SPEED");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_LOAD, 2, "REG_LOAD");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_VOLTAGE, 1, "REG_VOLTAGE");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_TEMPERATURE, 1, "REG_TEMPERATURE");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_ASYNC_WRITE_FLAG, 1, "REG_ASYNC_WRITE_FLAG");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_STATUS, 1, "REG_STATUS");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_MOVE_FLAG, 1, "REG_MOVE_FLAG");
  dump_stseries_reg(ctrl, cmd, rsp, id, STS::REG_CURRENT, 2, "REG_CURRENT");
}

}  // namespace esp_serial_servo

#endif  // STSERIESUTILS__H