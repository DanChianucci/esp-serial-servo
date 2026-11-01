#ifndef SERVOKIT__H
#define SERVOKIT__H


  #include "servokit/serial/serial_port.h"
  #include "servokit/serial/servo_controller.h"
  #include "servokit/serial/servo_packet.h"
  #include "servokit/core/servo_utils.h"
  #include "servokit/core/servo_types.h"
  #include "servokit/core/servo.h"

  #include "servokit/stseries.h"


  #if defined(ESP_PLATFORM) |  defined(SERVOKIT_IMPL_ESP)
    #include "servokit/platform/esp_idf.h"
  #endif

  #if defined(ARDUINO) | defined(SERVOKIT_IMPL_ARDUINO)
    #include "servokit/platform/arduino.h"
  #endif

#endif  // SERVOKIT__H