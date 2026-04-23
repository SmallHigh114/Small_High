# tools/hikcamera/CMakeLists.txt
cmake_minimum_required(VERSION 3.23)

# 查找依赖包
find_package(OpenCV REQUIRED)

# 收集源文件
file(GLOB SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)

find_library(MVS_LIBRAY
    NAMES MvCameraControl
    PATHS 
        /opt/MVS/lib/64
    NO_DEFAULT_PATH
)

# 创建库
add_library(tools_lad_hikcamera ${SOURCES})

# 设置目标属性
target_include_directories(tools_lad_hikcamera 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        
)

# 链接库 - 注意顺序
target_link_libraries(tools_lad_hikcamera 
    PUBLIC
    ${OpenCV_LIBS}
    ${MVS_LIBRAY}
)

# 设置 C++ 标准
set_target_properties(tools_lad_hikcamera PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# -------------------- 编译定义 --------------------
message(STATUS "Building tools_lad_hikcamera library with ${SOURCES}")