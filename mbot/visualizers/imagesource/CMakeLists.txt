cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(imagesource)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# find_package(GTK2 2.6 REQUIRED gtk)
find_library(common_utils REQUIRED)

# VX library
set(imagesource_SRC
  "src/image_convert.c"
  "src/image_source.c"
  "src/image_source_dc1394.c"
  "src/image_source_filedir.c"
  "src/image_source_islog.c"
  "src/image_source_null.c"
  "src/image_source_pgusb.c"
  "src/image_source_tcp.c"
  "src/image_source_v4l2.c"
  "src/image_u32.c"
  "src/image_u8.c"
  "src/image_u8x3.c"
  "src/image_util.c"
  "src/pnm.c"
)
add_library(imagesource STATIC
  ${imagesource_SRC}
)
target_link_libraries(imagesource
  common_utils
  # PRIVATE
    # ${GTK2_LIBRARIES}
  # PUBLIC
    # where top-level project will look for the library's public headers
    # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_include_directories(imagesource PUBLIC
  # ${GTK2_INCLUDE_DIRS}
  include
)
