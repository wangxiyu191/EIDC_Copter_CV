cmake_minimum_required(VERSION 3.0)
project(opflow)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp)
add_executable(opflow ${SOURCE_FILES})

include_directories(${OpenCV_INCLUDE_DIRS} /usr/local/include/)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} /usr/local/lib/libwiringPi.so)