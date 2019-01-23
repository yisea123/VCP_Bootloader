cmake_minimum_required ( VERSION 3.9 )
project ( VCP_Bootloader C CXX ASM )

set ( DEVICE STM32F103xB )
add_definitions ( -DUSE_HAL_DRIVER )
add_definitions ( -D${DEVICE} )
set ( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T\"${PROJECT_SOURCE_DIR}/STM32F103CBTx_FLASH.ld\"")

include_directories(Inc)

file(GLOB_RECURSE USER_CSOURCES Src/*.c)
file(GLOB_RECURSE USER_CXXSOURCES Src/*.cpp)

set(SOURCE_FILES ${USER_CSOURCES} ${USER_CXXSOURCES} ${HAL_SOURCES})
add_executable(${PROJECT_NAME}.elf ${SOURCE_FILES})

set(LIBRARIES	libs)
add_subdirectory (${LIBRARIES}/stm32_usb_core)
add_subdirectory (${LIBRARIES}/stm32_usb_cdc)
add_subdirectory (${LIBRARIES}/CMSIS)
if(DEVICE MATCHES "^STM32F1")
        add_subdirectory (${LIBRARIES}/STM32F1xx)
endif()
target_link_libraries(${PROJECT_NAME}.elf stm32_usb_core)
target_link_libraries(${PROJECT_NAME}.elf stm32_usb_cdc)
target_link_libraries(${PROJECT_NAME}.elf CMSIS)
target_link_libraries(${PROJECT_NAME}.elf Device)

set(HEX_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} ARGS -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} ARGS -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")
