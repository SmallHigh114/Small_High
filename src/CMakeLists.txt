# 项目根目录 CMakeLists.txt
cmake_minimum_required(VERSION 3.23)

project(rune-vision VERSION 2.0 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# 编译选项
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -DNDEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-O0 -g -DDEBUG)
endif()

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/run)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 设置项目结构变量
set(CORE_DIR ${CMAKE_SOURCE_DIR}/core)
set(MAIN_DIR ${CMAKE_SOURCE_DIR}/main)
set(TOOLS_DIR ${CMAKE_SOURCE_DIR}/tools)

# 打印配置信息
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Source directory: ${CMAKE_SOURCE_DIR}")
message(STATUS "Binary directory: ${CMAKE_BINARY_DIR}")

# 添加子目录
add_subdirectory(${CORE_DIR}/detector)
add_subdirectory(${CORE_DIR}/filter)
add_subdirectory(${CORE_DIR}/pnp_solver)
add_subdirectory(${CORE_DIR}/yolo)
add_subdirectory(${MAIN_DIR})   # 主程序没有子目录
add_subdirectory(${TOOLS_DIR}/hikcamera)
add_subdirectory(${TOOLS_DIR}/ui)
add_subdirectory(${TOOLS_DIR}/manage)
add_subdirectory(${TOOLS_DIR}/transform_coordinate)

# 生成 compile_commands.json 用于代码补全
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)