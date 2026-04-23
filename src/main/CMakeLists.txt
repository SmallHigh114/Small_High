# main/CMakeLists.txt
cmake_minimum_required(VERSION 3.23)

# 查找依赖包
find_package(OpenCV REQUIRED)
find_package(OpenVINO QUIET)

# 然后查找 src 目录下的文件
file(GLOB SRC_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

# 合并所有源文件
set(SOURCES ${SRC_SOURCES})

# 创建可执行文件
add_executable(lad_main ${SOURCES})

target_include_directories(lad_main
    PRIVATE 
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/core/detector/include
        ${CMAKE_SOURCE_DIR}/core/filter/include
        ${CMAKE_SOURCE_DIR}/core/pnp_solver/include
        ${CMAKE_SOURCE_DIR}/core/yolo/include
        ${CMAKE_SOURCE_DIR}/tools/hikcamera/include
        ${CMAKE_SOURCE_DIR}/tools/ui/include
        ${CMAKE_SOURCE_DIR}/tools/manage/include
        ${CMAKE_SOURCE_DIR}/tools/coordinate_transform/include
        ${OpenCV_INCLUDE_DIRS}
        ${OpenVINO_INCLUDE_DIRS}
)

# 链接库 - 注意顺序很重要！
target_link_libraries(lad_main
    PRIVATE 
        ${OpenCV_LIBS}
        openvino::runtime
        core_lad_detector
        core_lad_filter
        core_lad_pnp_solver
        core_lad_yolo
        tools_lad_hikcamera
        tools_lad_ui
        tools_lad_manage
        tools_lad_transform_coordinate
)

# 设置 C++ 标准
set_target_properties(lad_main PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

message(STATUS "Building lad_main executable with ${SOURCES}")

# 拷贝模型文件夹
if(EXISTS ${CMAKE_SOURCE_DIR}/../models)
    add_custom_command(TARGET lad_main POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/../models
            $<TARGET_FILE_DIR:lad_main>/models
        COMMENT "Copying model directory to output directory"
    )
else()
    message(WARNING "model directory not found at ${CMAKE_SOURCE_DIR}/models, skipping copy")
endif()

# 拷贝视频文件夹
if(EXISTS ${CMAKE_SOURCE_DIR}/../docs)
    add_custom_command(TARGET lad_main POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/../docs
            $<TARGET_FILE_DIR:lad_main>/docs
        COMMENT "Copying videos directory to output directory"
    )
else()
    message(WARNING "videos directory not found at ${CMAKE_SOURCE_DIR}/docs, skipping copy")
endif()

# 拷贝配置文件，创建符号链接到配置文件夹
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../configs)
    get_filename_component(CONFIG_SRC_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/../../configs"
        ABSOLUTE)

    set(CONFIG_LINK_PATH "$<TARGET_FILE_DIR:lad_main>/configs")

    # 只保留一个符号链接创建命令
    add_custom_command(TARGET lad_main POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E remove -f ${CONFIG_LINK_PATH}
        COMMAND ${CMAKE_COMMAND} -E create_symlink ${CONFIG_SRC_DIR} ${CONFIG_LINK_PATH}
        COMMENT "Creating symlink for config directory in output directory"
    )
else()
    message(WARNING "config directory not found at ${CMAKE_CURRENT_SOURCE_DIR}/../../configs, skipping symlink")
endif()