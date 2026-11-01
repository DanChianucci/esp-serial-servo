# Host build of the examples: no platform SDK, scripted fake SerialPort.
# Included from tests/host/CMakeLists.txt, or configure this directory alone
# with a C++23 compiler.
cmake_minimum_required(VERSION 3.16)
if(NOT CMAKE_PROJECT_NAME)
  project(servokit_examples CXX)
  if(MSVC)
    add_compile_options(/std:c++latest)
  else()
    add_compile_options(-std=c++23)
  endif()
  enable_testing()
endif()

set(SERVOKIT_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

add_library(servokit_host STATIC
  ${SERVOKIT_ROOT}/lib/serial/servo_controller.cpp
  ${SERVOKIT_ROOT}/lib/serial/servo_packet.cpp
  ${SERVOKIT_ROOT}/lib/families/stseries/stseries_packet.cpp
  ${SERVOKIT_ROOT}/lib/families/stseries/sts_servo.cpp
  ${SERVOKIT_ROOT}/lib/families/stseries/sts_servo_family.cpp
  ${SERVOKIT_ROOT}/lib/families/stseries/sts_servo_generic.cpp)
target_include_directories(servokit_host PUBLIC ${SERVOKIT_ROOT}/include)

foreach(example example_generic example_family example_raw)
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} PRIVATE servokit_host)
  # The fake port is shared with the unit tests.
  target_include_directories(${example} PRIVATE ${CMAKE_CURRENT_LIST_DIR} ${SERVOKIT_ROOT}/tests/host)
  target_compile_options(${example} PRIVATE -Wall -Wextra)
  add_test(NAME ${example} COMMAND ${example})
endforeach()
