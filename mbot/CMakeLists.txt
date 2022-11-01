cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(mbot_codebase)

option(BUILD_ON_BOT "Build code for the MBot." OFF)

if(BUILD_ON_BOT)
  message("Building code for the MBot.")
  set(MACHINE_TYPE "OMNI")
  add_subdirectory(drivers)
else()
  message("Building code for laptop.")
  set(MACHINE_TYPE "LAPTOP")
endif()

add_subdirectory(mbot_lcm_msgs)
add_subdirectory(common_utils)
add_subdirectory(mbot_autonomy)
add_subdirectory(mbot_pico_shim)
add_subdirectory(visualizers)

