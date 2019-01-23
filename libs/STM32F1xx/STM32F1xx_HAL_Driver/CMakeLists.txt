# CMakeList.txt : CMake project for STM32F1xx_HAL_Driver include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

set(TARGET		STM32F1xx_HAL_Driver)
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
target_link_libraries(${PROJECT_NAME} Device)