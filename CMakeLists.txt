# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/opt/pico-sdk")

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(powerboard C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

#set(PICO_CXX_ENABLE_RTTI 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(powerboard powerboard.cpp )

pico_set_program_name(powerboard "powerboard")
pico_set_program_version(powerboard "0.1")

pico_enable_stdio_uart(powerboard 0)
pico_enable_stdio_usb(powerboard 1)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI build)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/TinyGPSPlus)

# Add the standard library to the build
target_link_libraries(powerboard
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(powerboard PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(powerboard
        hardware_i2c
        hardware_pio
        hardware_timer
        hardware_watchdog
        hardware_spi
        pico_time
        FatFs_SPI
        TinyGPSPlus
        )

pico_add_extra_outputs(powerboard)

