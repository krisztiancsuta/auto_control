cmake_minimum_required(VERSION 3.17)

# Default to Pico (RP2040). Override with -DPICO_BOARD=<board> if needed.
if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico_w CACHE STRING "Pico board type")
endif()

if(NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "PICO_SDK_PATH is not set. Run scripts/setup_pico_sdk.sh first.")
endif()

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pio_quadrature_encoder C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

option(PWM_DEBUG_BUILD "Build serial PWM debug firmware (GPIO 9, 1000-2000 us)" OFF)

set(MICRO_ROS_DIR ${CMAKE_CURRENT_LIST_DIR}/third_party/micro_ros_raspberrypi_pico_sdk)

if(PWM_DEBUG_BUILD)
    add_executable(pio_quadrature_encoder
        src/main_debug.c
        src/pwm_esc.c
        src/serial_cli.c
    )
else()
    link_directories(${MICRO_ROS_DIR}/libmicroros)

    add_executable(pio_quadrature_encoder
        src/main.c
        src/pwm_esc.c
        src/serial_cli.c
        src/state_space_control.c
        src/pico_uart_transport.c
    )
endif()

if(NOT PWM_DEBUG_BUILD)
    pico_generate_pio_header(pio_quadrature_encoder
        ${CMAKE_CURRENT_LIST_DIR}/src/quadrature_encoder.pio
    )
endif()

target_include_directories(pio_quadrature_encoder PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

if(NOT PWM_DEBUG_BUILD)
    target_include_directories(pio_quadrature_encoder PRIVATE
        ${MICRO_ROS_DIR}
        ${MICRO_ROS_DIR}/libmicroros/include
    )
endif()

target_link_libraries(pio_quadrature_encoder
    pico_stdlib
    pico_multicore
    hardware_gpio
    hardware_pwm
    hardware_clocks
)

if(NOT PWM_DEBUG_BUILD)
    target_link_libraries(pio_quadrature_encoder
        hardware_pio
        hardware_uart
        microros
    )
endif()

target_compile_options(pio_quadrature_encoder PRIVATE
    -ffunction-sections
    -fdata-sections
)

pico_enable_stdio_usb(pio_quadrature_encoder 1)
pico_enable_stdio_uart(pio_quadrature_encoder 0)

add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

pico_add_extra_outputs(pio_quadrature_encoder)
