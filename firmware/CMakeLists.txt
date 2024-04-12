cmake_minimum_required(VERSION 3.12)

#set(PICO_SDK_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk")
#set(PICO_TOOLCHAIN_PATH "${PICO_SDK_PATH}/../gcc-arm-none-eabi")
#set(CMAKE_MAKE_PROGRAM "${PICO_SDK_PATH}/../ninja/ninja")

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(psdaptwor C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# swy: write a RAM-only .uf2 file that does not persist, can be run with `picotool load -x psdaptwor.uf2`
set(PICO_NO_FLASH 1)

set(PICO_BOARD psdaptwor)
set(PICO_BOARD_HEADER_DIRS "${PROJECT_SOURCE_DIR}/boards")

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

# Add blink example
add_executable(padaptwor main.c)

# pull in common dependencies
target_link_libraries(padaptwor pico_stdlib hardware_adc hardware_i2c)

pico_enable_stdio_usb(padaptwor ENABLED)

# create map/bin/hex file etc.
pico_add_extra_outputs(padaptwor)


add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()
