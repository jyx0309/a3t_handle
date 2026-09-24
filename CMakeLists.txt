cmake_minimum_required(VERSION 3.16)
project(a3t_handle LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(ARM_CONTROL_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../arm_control_sdk" CACHE PATH
    "Path to the vendor arm_control_sdk directory")
find_path(ARM_CONTROL_SDK_INCLUDE_DIR arm_control_sdk/carm_cobot.h
    PATHS "${ARM_CONTROL_SDK_DIR}/include" REQUIRED)
find_library(ARM_CONTROL_SDK_LIBRARY arm_control_sdk
    PATHS "${ARM_CONTROL_SDK_DIR}/lib" REQUIRED NO_DEFAULT_PATH)
find_library(MLOG_LIBRARY mlog PATHS "${ARM_CONTROL_SDK_DIR}/lib" REQUIRED NO_DEFAULT_PATH)
find_file(JSONCPP_LIBRARY libjsoncpp.so.1 PATHS "${ARM_CONTROL_SDK_DIR}/lib" REQUIRED NO_DEFAULT_PATH)

add_executable(a3t_handle
    src/main.cpp
    src/arm/arm_worker.cpp
    src/control/handle_mode_manager.cpp
    src/control/mit_handle_controller.cpp
    src/logging/session_logger.cpp
    src/math/handle_controller.cpp
    src/safety/safety_monitor.cpp
    src/ui/main_window.cpp
)

target_include_directories(a3t_handle PRIVATE src "${ARM_CONTROL_SDK_INCLUDE_DIR}")
target_link_options(a3t_handle PRIVATE -Wl,--no-as-needed)
target_link_libraries(a3t_handle PRIVATE Qt5::Widgets "${ARM_CONTROL_SDK_LIBRARY}" "${MLOG_LIBRARY}" "${JSONCPP_LIBRARY}")
set_target_properties(a3t_handle PROPERTIES BUILD_RPATH "${ARM_CONTROL_SDK_DIR}/lib")
