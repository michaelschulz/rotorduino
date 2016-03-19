set(PROJECT_NAME ardurotor)

set(CMAKE_ARDUINO_PATH ~/work/arduino-cmake-master)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_ARDUINO_PATH}/cmake/ArduinoToolchain.cmake) # Arduino Toolchain
set(CMAKE_EXTERNAL_LIBS_PATH ~/Desktop/arduino-1.0.3/libraries) # arduino external libs

# Link external libs directories (as Arduino-IDE uses it)
link_directories(${CMAKE_EXTERNAL_LIBS_PATH})

cmake_minimum_required(VERSION 2.8)

project(${PROJECT_NAME} C CXX)

# find_package(Arduino 1.0 REQUIRED)

# Define your Arduino board (below is for Arduino uno)
set(${PROJECT_NAME}_BOARD atmega328)

# Define the source code
set(${PROJECT_NAME}_SRCS ${PROJECT_NAME}.cpp)

# Define the port for uploading code to the Arduino
set(${PROJECT_NAME}_PORT /dev/ttyUSB1)

# Command to generate code arduino firmware (.hex file)
generate_arduino_firmware(${PROJECT_NAME})
