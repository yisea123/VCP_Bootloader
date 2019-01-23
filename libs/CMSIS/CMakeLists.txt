# CMakeList.txt : CMake project for CMSIS include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

set(TARGET		CMSIS)
set(HDR_DIR		Include)

project(${TARGET} C)

# headers are always in include/
file(GLOB_RECURSE HDRS ${HDR_DIR}/*.h)
include_directories(${HDR_DIR})

# create library
add_library(${TARGET} INTERFACE)
target_include_directories(${TARGET} INTERFACE ${HDR_DIR})