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
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/stseries/stseries_packet.cpp)
target_include_directories(test_transaction PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
target_compile_options(test_transaction PRIVATE -Wall -Wextra)
add_test(NAME test_transaction COMMAND test_transaction)
