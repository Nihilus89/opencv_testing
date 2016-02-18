cmake_minimum_required(VERSION 2.8.12)

project(open_cv_testing)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_C_COMPILER /usr/bin/gcc)
set(CMAKE_CXX_COMPILER /usr/bin/g++)

add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS})