cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME mapserver)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
#include(cmake/lcmtypes.cmake)
#lcmtypes_build()

#include_directories(${LCMTYPES_INCLUDE_DIRS})

find_package(PkgConfig REQUIRED)

pkg_check_modules(GLIB2 REQUIRED glib-2.0)
pkg_check_modules(GTK2 REQUIRED gtk+-2.0)
pkg_check_modules(LCM REQUIRED lcm)
pkg_check_modules(OPENCV REQUIRED opencv)
if(NOT OPENCV_FOUND)
    message("OpenCV was not found.")
endif()

pkg_check_modules(BOT_CORE REQUIRED bot2-core)

add_subdirectory(src)
