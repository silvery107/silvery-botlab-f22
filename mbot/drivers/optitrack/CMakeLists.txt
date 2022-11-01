# cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
# project(optitrack_driver)

# set(CMAKE_BUILD_TYPE RelWithDebInfo)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# find_package(Threads REQUIRED)
# find_package(lcm REQUIRED)

# find_library(common_utils REQUIRED)

# add_executable(optitrack_driver src/optitrack.cpp src/PacketClient.cpp)

# target_link_libraries(optitrack_driver
#     ${CMAKE_THREAD_LIBS_INIT}
#     lcm
#     common_utils
#     mbot_lcm_msgs-cpp
# )

# target_include_directories(optitrack_driver PRIVATE include)