# CMakeList.txt : CMake project for stm32_usb_cdc, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

set(TARGET		stm32_usb_cdc)
set(HDR_DIR		Inc)
set(SRCS_DIR	Src)

project(${TARGET} C)

# headers are always in include/
file(GLOB_RECURSE HDRS ${HDR_DIR}/*.h)
include_directories(${HDR_DIR})
# sources are always in src/
file(GLOB_RECURSE TEMPLATES ${SRCS_DIR}/*template.c)
file(GLOB_RECURSE SRCS ${SRCS_DIR}/*.c)
list(REMOVE_ITEM SRCS ${TEMPLATES})

# create library
add_library(${TARGET} STATIC ${HDRS} ${SRCS})
target_include_directories(${TARGET} PUBLIC ${HDR_DIR})
target_link_libraries(${PROJECT_NAME} CMSIS)
target_link_libraries(${PROJECT_NAME} stm32_usb_core)