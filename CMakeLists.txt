cmake_minimum_required(VERSION 2.8.12)

project(open_cv_testing)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_C_COMPILER /usr/bin/gcc)
set(CMAKE_CXX_COMPILER /usr/bin/g++)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp modbus/modbus.c modbus/modbus-rtu.c modbus/modbus-data.c csvparser/csvparser.c mic488.c)

add_executable(main ${SOURCE_FILES})
target_link_libraries(main ${OpenCV_LIBS})