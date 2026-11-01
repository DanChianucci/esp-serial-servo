# Standalone host test build (separate from the ESP-IDF component CMake).
# Requires a C++23 compiler with <expected> (GCC 12+, Clang 16+ w/ libc++/libstdc++ 12+, MSVC 2022).
cmake_minimum_required(VERSION 3.16)
project(servokit_host_tests CXX)

# Dialect flag set explicitly: older CMake versions do not know CXX23.
if(MSVC)
  add_compile_options(/std:c++latest)
else()
  add_compile_options(-std=c++23)
endif()

enable_testing()

add_executable(test_fake_serial_port test_fake_serial_port.cpp)
target_include_directories(test_fake_serial_port PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
target_compile_options(test_fake_serial_port PRIVATE -Wall -Wextra)
add_test(NAME test_fake_serial_port COMMAND test_fake_serial_port)

add_executable(test_transaction
  test_transaction.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/serial/servo_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/serial/servo_packet.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/families/stseries/stseries_packet.cpp)
target_include_directories(test_transaction PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
target_compile_options(test_transaction PRIVATE -Wall -Wextra)
add_test(NAME test_transaction COMMAND test_transaction)

add_executable(test_sts_servo
  test_sts_servo.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/serial/servo_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/serial/servo_packet.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/families/stseries/stseries_packet.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/families/stseries/sts_servo.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/families/stseries/sts_servo_family.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/families/stseries/sts_servo_generic.cpp)
target_include_directories(test_sts_servo PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
target_compile_options(test_sts_servo PRIVATE -Wall -Wextra)
add_test(NAME test_sts_servo COMMAND test_sts_servo)

add_executable(test_sts_family
  test_sts_family.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/serial/servo_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/serial/servo_packet.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/families/stseries/stseries_packet.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/families/stseries/sts_servo.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/families/stseries/sts_servo_family.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/families/stseries/sts_servo_generic.cpp)
target_include_directories(test_sts_family PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
target_compile_options(test_sts_family PRIVATE -Wall -Wextra)
add_test(NAME test_sts_family COMMAND test_sts_family)

add_executable(test_sts_generic
  test_sts_generic.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/serial/servo_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/serial/servo_packet.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/families/stseries/stseries_packet.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/families/stseries/sts_servo.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/families/stseries/sts_servo_family.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/families/stseries/sts_servo_generic.cpp)
target_include_directories(test_sts_generic PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
target_compile_options(test_sts_generic PRIVATE -Wall -Wextra)
add_test(NAME test_sts_generic COMMAND test_sts_generic)

# Examples (one per access tier) built and run with the tests.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../examples ${CMAKE_BINARY_DIR}/exbin)
