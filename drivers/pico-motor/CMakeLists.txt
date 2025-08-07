cmake_minimum_required(VERSION 3.13)

include ($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
# if you have pico_sdk_import.cmake in the project and you did not 
# set PICO_SDK_PATH, uncomment the following and comment above
# include(pico_sdk_import.cmake)

project(Pico-Motor C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
pico_sdk_init()

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

set(CMAKE_C_FLAGS "-mcpu=cortex-m0plus -mthumb")
set(CMAKE_EXE_LINKER_FLAGS "-mcpu=cortex-m0plus -mthumb")

add_executable(pico-motor
    src/main.c
)
pico_enable_stdio_usb(pico-motor 1)
pico_enable_stdio_uart(pico-motor 0)

pico_add_extra_outputs(pico-motor)

target_link_libraries(pico-motor pico_stdlib hardware_pwm hardware_uart hardware_gpio hardware_irq)

