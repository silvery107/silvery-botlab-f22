cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(common_utils)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(GTK2 2.6 REQUIRED gtk)
find_library(mbot_lcm_msgs REQUIRED)

file(GLOB common_SRC
  "src/*.c"
)
add_library(common_utils STATIC
  ${common_SRC}
  src/geometric/pose_trace.cpp
)
target_link_libraries(common_utils
  # PRIVATE
    ${GTK2_LIBRARIES}
    mbot_lcm_msgs
  # PUBLIC
    # where top-level project will look for the library's public headers
    # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_include_directories(common_utils PUBLIC
  ${GTK2_INCLUDE_DIRS}
  include
)
