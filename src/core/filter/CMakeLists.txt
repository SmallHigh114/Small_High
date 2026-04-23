# core/filter/CMakeLists.txt
cmake_minimum_required(VERSION 3.23)

# 查找依赖包

# 收集源文件
file(GLOB SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)

# 创建库
add_library(core_lad_filter ${SOURCES})

# 设置目标属性
target_include_directories(core_lad_filter 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_SOURCE_DIR}/core/detector/include
        ${OpenCV_INCLUDE_DIRS}
)

# 链接库 - 注意顺序
target_link_libraries(core_lad_filter 
    PUBLIC 
    ${OpenCV_LIBS}
    core_lad_detector
)

# 设置 C++ 标准
set_target_properties(core_lad_filter PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# -------------------- 编译定义 --------------------
message(STATUS "Building core_lad_filter library with ${SOURCES}")