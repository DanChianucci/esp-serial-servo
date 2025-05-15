#ifndef SERIAL_SERVO__H
#define SERIAL_SERVO__H


  #include "serial_servo/base/servo_bus.h"
  #include "serial_servo/base/servo_controller.h"
  #include "serial_servo/base/servo_packet.h"
  #include "serial_servo/base/servo_utils.h"

  #include "serial_servo/stseries.h"


  #if defined(ESP_PLATFORM) |  defined(SERIAL_SERVO_IMPL_ESP)
    #include "serial_servo/platform/esp_idf.h"
  #endif

  #if defined(ARDUINO) | defined(SERIAL_SERVO_IMPL_ARDUINO)
    #include "serial_servo/platform/arduino.h"
  #endif

#endif  // SERIALSERVO__H