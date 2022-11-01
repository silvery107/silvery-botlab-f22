cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(vx)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(OpenGL_GL_PREFERENCE GLVND)
# set(OpenGL_GL_PREFERENCE LEGACY)
find_package(GTK2 2.6 REQUIRED gtk)
find_package(X11)
find_package(OpenGL)
find_package(ZLIB)

find_library(LIBDC libdc1394-22-dev REQUIRED)
find_library(common_utils REQUIRED)

# VX library
file(GLOB vx_SRC
  "src/*.c"
  # "src/vx/gtk/*.c"
  # "src/vx/math/*.c"
)
file(GLOB vx_HEADERS
  "src/*.h"
  # "src/vx/gtk/*.h"
  # "src/vx/math/*.h"
)
set(vx_SRC
  "${vx_SRC}"
  "src/gtk/gtkuimagepane.c"
  "src/gtk/priority_queue.c"
  "src/gtk/vx_gtk_buffer_manager.c"
  "src/gtk/vx_gtk_display.c"
  "src/gtk/vx_gtk_display_source.c"
  "src/math/fasttrig.c"
  "src/math/matd.c"
  # "src/math/math_util.c"
  "src/math/svd22.c"
)
set(vx_HEADERS
  "${vx_HEADERS}"
  "src/gtk/gtkuimagepane.h"
  "src/gtk/priority_queue.h"
  "src/gtk/vx_gtk_buffer_manager.h"
  "src/gtk/vx_gtk_display.h"
  # "src/gtk/vx_gtk_display_source.h"
  "src/math/fasttrig.h"
  "src/math/matd.h"
  # "src/math/math_util.h"
  "src/math/svd22.h"
)
add_library(vx STATIC
  ${vx_SRC}
  ${vx_HEADERS}
)
target_link_libraries(vx
  # PRIVATE
  common_utils
    imagesource
    ${GTK2_LIBRARIES}
    ${X11_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${ZLIB_LIBRARIES}
  # PUBLIC
    # where top-level project will look for the library's public headers
    # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_include_directories(vx PUBLIC
  include
  ${GTK2_INCLUDE_DIRS}
  ${X11_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR}
  ${ZLIB_INCLUDE_DIRS}
)
