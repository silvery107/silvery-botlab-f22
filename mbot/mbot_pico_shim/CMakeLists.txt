cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(mbot_pico_shim C)

#set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

find_package(Threads REQUIRED)
find_package(lcm REQUIRED)

find_library(common_utils REQUIRED)
find_library(mbot_lcm_msgs REQUIRED)

include(${LCM_USE_FILE})

# Shim binary.
add_executable(pico_shim src/mbot_shim_main.c
  src/comms_common.c
  src/listener.c
  src/messages_mb.c
  src/protocol.c
  src/topic_data.c
)
target_link_libraries(pico_shim
  ${CMAKE_THREAD_LIBS_INIT}
  lcm
  common_utils
  mbot_lcm_msgs
)
target_include_directories(pico_shim PRIVATE
  include
)
