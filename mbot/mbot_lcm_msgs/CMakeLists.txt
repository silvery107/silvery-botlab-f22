cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(mbot_lcm_msgs)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(lcm REQUIRED)
find_package(Python3)
find_package(Java)

if(JAVA_FOUND)
  include(UseJava)
endif()

include(${LCM_USE_FILE})
include(GenerateExportHeader)

lcm_wrap_types(
  C_EXPORT mbot_lcm_msgs
  C_SOURCES c_sources
  C_HEADERS c_headers
  CPP_HEADERS cpp_headers
  JAVA_SOURCES java_sources
  PYTHON_SOURCES python_install_sources
  lcmtypes/exploration_status_t.lcm
  lcmtypes/mbot_motor_command_t.lcm
  lcmtypes/odometry_t.lcm
  lcmtypes/pose_xyt_t.lcm
  lcmtypes/lidar_t.lcm
  lcmtypes/mbot_state_t.lcm
  lcmtypes/oled_message_t.lcm
  lcmtypes/reset_odometry_t.lcm
  lcmtypes/mbot_wheel_ctrl_t.lcm
  lcmtypes/omni_motor_command_t.lcm
  lcmtypes/omni_encoder_t.lcm
  lcmtypes/robot_path_t.lcm
  lcmtypes/mbot_encoder_t.lcm
  lcmtypes/message_received_t.lcm
  lcmtypes/particles_t.lcm
  lcmtypes/timestamp_t.lcm
  lcmtypes/mbot_imu_t.lcm
  lcmtypes/occupancy_grid_t.lcm
  lcmtypes/particle_t.lcm
  lcmtypes/wifi_data_t.lcm
  lcmtypes/mbot_system_reset_t.lcm
  lcmtypes/planner_request_t.lcm
  lcmtypes/pid_values_t.lcm
)

lcm_add_library(mbot_lcm_msgs C ${c_sources} ${c_headers})
generate_export_header(mbot_lcm_msgs)
target_include_directories(mbot_lcm_msgs INTERFACE
  ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(mbot_lcm_msgs INTERFACE
  ${CMAKE_CURRENT_BINARY_DIR})

lcm_add_library(mbot_lcm_msgs-cpp CPP ${cpp_headers})
target_include_directories(mbot_lcm_msgs-cpp INTERFACE
  ${CMAKE_CURRENT_BINARY_DIR})

if(JAVA_FOUND)
  add_jar(mbot_lcm_msgs-jar
    OUTPUT_NAME mbot_lcm_msgs
    INCLUDE_JARS lcm-java
    SOURCES ${java_sources}
  )
endif()

if(Python3_FOUND)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "if True:
      from distutils import sysconfig as sc
      print(sc.get_python_lib())"
    OUTPUT_VARIABLE PY_INSTALL_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  lcm_install_python(DESTINATION ${PY_INSTALL_DIR} ${python_install_sources})
endif()
