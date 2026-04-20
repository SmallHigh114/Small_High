# coordinate_test/CMakeLists.txt
cmake_minimum_required(VERSION 3.23)

find_package(OpenCV REQUIRED)
find_package(OpenVINO QUIET)

file(GLOB SRC_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
)

set(SOURCES ${SRC_SOURCES})

add_executable(test ${SOURCES})

target_include_directories(test
    PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/transform_coordinate/include
)

# 链接库 - 注意顺序很重要！
target_link_libraries(test
    PRIVATE 
        transform_coordinate
)

message(STATUS "Building test executable with ${SOURCES}")