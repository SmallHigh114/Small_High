cmake_minimum_required(VERSION 3.10)
project(coordinate_transform)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

# 🔥 强制 Eigen 路径
include_directories(/usr/include/eigen3)

add_executable(test test.cpp)

target_include_directories(test PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(test
    ${OpenCV_LIBS}
)