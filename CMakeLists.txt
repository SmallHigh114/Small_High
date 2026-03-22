cmake_minimum_required(VERSION 3.16)

project(LaserModule)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)

set(MVS_PATH /opt/MVS)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${MVS_PATH}/include

    ${CMAKE_SOURCE_DIR}/detector/include
    ${CMAKE_SOURCE_DIR}/HikCamera/include
    ${CMAKE_SOURCE_DIR}/UI/include
    ${CMAKE_SOURCE_DIR}/YOLO/include
    ${CMAKE_SOURCE_DIR}/pnp_solver/include
)

link_directories(${MVS_PATH}/lib/64)

file(GLOB_RECURSE SRC_FILES
    detector/src/*.cpp
    HikCamera/src/*.cpp
    UI/src/*.cpp
    YOLO/src/*.cpp
    pnp_solver/src/*.cpp
)

add_executable(LaserModule
    main.cpp
    ${SRC_FILES}
)

target_link_libraries(LaserModule
    ${OpenCV_LIBS}
    openvino::runtime
    MvCameraControl
    pthread
)