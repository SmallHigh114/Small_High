# core/yolo/CMakeLists.txt
cmake_minimum_required(VERSION 3.23)

# 查找依赖包
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)

# 收集源文件
file(GLOB SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)

# 创建库
add_library(core_lad_yolo ${SOURCES})

# 设置目标属性
target_include_directories(core_lad_yolo 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${OpenVINO_INCLUDE_DIRS}
)

# 链接库 - 注意顺序
target_link_libraries(core_lad_yolo 
    PUBLIC 
    ${OpenCV_LIBS}
    openvino::runtime
)

# 设置 C++ 标准
set_target_properties(core_lad_yolo PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# -------------------- 编译定义 --------------------
target_compile_definitions(core_lad_yolo PRIVATE USE_OPENVINO)

message(STATUS "Building core_lad_yolo library with ${SOURCES}")