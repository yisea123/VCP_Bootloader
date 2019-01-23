# CMakeList.txt : CMake project for STM32F1xx include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

set(TARGET		Device)
set(HDR_DIR		Include)

project(${TARGET} C)

# headers are always in include/
file(GLOB_RECURSE HDRS ${HDR_DIR}/*.h)
include_directories(${HDR_DIR})

# create library
add_library(${TARGET} INTERFACE)
add_subdirectory (CMSIS_STM32F1xx)
add_subdirectory (STM32F1xx_HAL_Driver)
target_link_libraries(${PROJECT_NAME} INTERFACE CMSIS_Device)
target_link_libraries(${PROJECT_NAME} INTERFACE STM32F1xx_HAL_Driver)
target_include_directories(${TARGET} INTERFACE ${HDR_DIR})