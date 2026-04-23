# transform_coordinate/CMakeLists.txt
cmake_minimum_required(VERSION 3.23)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(tools_lad_transform_coordinate INTERFACE)

target_include_directories(tools_lad_transform_coordinate
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}  # 对应的 OpenCV 包含目录
)

target_link_libraries(tools_lad_transform_coordinate
    INTERFACE
        Eigen3::Eigen
        ${OpenCV_LIBS}
)

set_target_properties(tools_lad_transform_coordinate PROPERTIES
    CXX_STANDARD 20 # 设置 C++ 标准
    CXX_STANDARD_REQUIRED ON # 强制要求 C++ 标准
    CXX_EXTENSIONS OFF  # 禁止使用编译器特定的扩展
)

file(GLOB_RECURSE TRANSFORM_HPP_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/transform_coordinate/*.hpp")

target_sources(tools_lad_transform_coordinate INTERFACE
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    FILES
        ${TRANSFORM_HPP_FILES}
)

message(STATUS "Building tools_lad_transform_coordinate as interface library (header-only)")
