cmake_minimum_required(VERSION 3.16)
project(a3t_handle LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
include(CTest)

find_package(Qt5 REQUIRED COMPONENTS Widgets Xml Network)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

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
    src/math/urdf_gravity.cpp
    src/safety/safety_monitor.cpp
    src/ui/main_window.cpp
    src/ui/local_control.cpp
)

target_include_directories(a3t_handle PRIVATE src "${ARM_CONTROL_SDK_INCLUDE_DIR}")
file(SHA256 "${ARM_CONTROL_SDK_LIBRARY}" SDK_BINARY_SHA256)
target_compile_definitions(a3t_handle PRIVATE SDK_BINARY_SHA256="${SDK_BINARY_SHA256}")
target_link_options(a3t_handle PRIVATE -Wl,--no-as-needed)
target_link_libraries(a3t_handle PRIVATE Qt5::Widgets Qt5::Xml Qt5::Network Eigen3::Eigen Threads::Threads "${ARM_CONTROL_SDK_LIBRARY}" "${MLOG_LIBRARY}" "${JSONCPP_LIBRARY}")
set_target_properties(a3t_handle PROPERTIES BUILD_RPATH "${ARM_CONTROL_SDK_DIR}/lib")

if(BUILD_TESTING)
  add_executable(a3t_logger_tests tests/session_logger_test.cpp src/logging/session_logger.cpp src/control/handle_mode_manager.cpp)
  target_include_directories(a3t_logger_tests PRIVATE src)
  target_link_libraries(a3t_logger_tests PRIVATE Qt5::Core Threads::Threads)
  add_test(NAME a3t_logger_tests COMMAND a3t_logger_tests)
  set_tests_properties(a3t_logger_tests PROPERTIES TIMEOUT 15)
  add_executable(a3t_handle_unit_tests
      tests/handle_controller_test.cpp
      src/control/mit_handle_controller.cpp
      src/math/handle_controller.cpp
      src/math/urdf_gravity.cpp
      src/safety/safety_monitor.cpp
  )
  target_include_directories(a3t_handle_unit_tests PRIVATE src)
  target_compile_definitions(a3t_handle_unit_tests PRIVATE TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/handle.json")
  target_link_libraries(a3t_handle_unit_tests PRIVATE Qt5::Core Qt5::Xml Eigen3::Eigen)
  add_test(NAME a3t_handle_unit_tests COMMAND a3t_handle_unit_tests)
  add_test(NAME a3t_ui_smoke COMMAND a3t_handle --smoke-test)
  set_tests_properties(a3t_ui_smoke PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 10)
  find_package(Python3 COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    add_test(NAME a3t_local_control_tests COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_local_control.py" "$<TARGET_FILE:a3t_handle>")
    set_tests_properties(a3t_local_control_tests PROPERTIES TIMEOUT 15)
    add_test(NAME a3t_diagnostics_tests COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_diagnostics.py")
  endif()
endif()
