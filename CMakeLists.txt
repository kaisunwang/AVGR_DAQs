cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(double_DAQ C CXX ASM)

pico_sdk_init()

# ── FatFS/SD library (central only) ──────────────────────────────────────────
add_subdirectory(
    central/no-OS-FatFS-SD-SPI-RPi-Pico-master/FatFs_SPI
    build_fatfs_spi
)

# ── Peripheral target ─────────────────────────────────────────────────────────
add_executable(peripheral
    peripheral/peripheral.c
    peripheral/spi_slave_config.c
    peripheral/neopixel.c
    peripheral/capture.c
)

pico_generate_pio_header(peripheral ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

target_include_directories(peripheral PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/peripheral)

target_link_libraries(peripheral PRIVATE
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_clocks
    hardware_spi
    hardware_i2c
)

pico_enable_stdio_usb(peripheral 1)
pico_enable_stdio_uart(peripheral 0)
pico_add_extra_outputs(peripheral)

# ── Central target ────────────────────────────────────────────────────────────
add_executable(central
    central/central.c
    central/flash_log.c
    central/neopixel.c
    central/sd_config.c
    central/rtc.c
)

pico_generate_pio_header(central ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

target_include_directories(central PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/central)

target_link_libraries(central PRIVATE
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_flash
    hardware_sync
    hardware_clocks
    hardware_spi
    hardware_i2c
    FatFs_SPI
)

pico_enable_stdio_usb(central 1)
pico_enable_stdio_uart(central 0)
pico_add_extra_outputs(central)