# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_C_FLAGS_DEBUG " -g -Os")
# string(APPEND CMAKE_C_FLAGS_DEBUG " -Os -g")
string(REGEX REPLACE "-Og" "-Os" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}" ) 

set(CMAKE_VERBOSE_MAKEFILE 1)


# enable FetchContent
include(FetchContent)

# declare Catch2
FetchContent_Declare(littlefs
    GIT_REPOSITORY https://github.com/littlefs-project/littlefs.git
    GIT_TAG        v2.11.2
    )

# make available
FetchContent_MakeAvailable(littlefs)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

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
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/src/boards)

set(PICO_BOARD smol_servo_rpico CACHE STRING "Board type")
# set(PICO_BOARD pico2 CACHE STRING "Board type")

add_compile_definitions(smol_servo_rpico PUBLIC
    PICO_USE_STACK_GUARDS=1
    PICO_STACK_SIZE=0x1000
# do not support lockout of core1 because realtime
# core1 will never access flash (including XIP)
    PICO_FLASH_SAFE_EXECUTE_PICO_SUPPORT_MULTICORE_LOCKOUT=0
    PICO_FLASH_ASSUME_CORE1_SAFE=1
# TODO: PICO_EMBED_XIP_SETUP=1 is supposed to configure flash better, but seems to crash in the debugger?
    PICO_EMBED_XIP_SETUP=0
    )

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(smol_servo_rpico C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

if (TARGET tinyusb_device)
    add_executable(smol_servo_rpico
        src/main.c
        src/fusb302.c
        src/tmc6200.c
        src/as5047d.c
        src/smol_servo_adc.c
        src/smol_servo_drv.c
        src/smol_servo_mag.c
        src/smol_servo_loop.c
        src/smol_servo_pwm.c
        src/smol_servo_usb_pd.c
        src/smol_console.c
        src/smol_fs.c
        src/pico_debug.c
        ${littlefs_SOURCE_DIR}/lfs.c
        ${littlefs_SOURCE_DIR}/lfs_util.c
        )

    target_include_directories(smol_servo_rpico PUBLIC
        ${littlefs_SOURCE_DIR}
        )

    # pull in common dependencies
    target_link_libraries(
        smol_servo_rpico 
        pico_stdlib 
        pico_multicore 
        hardware_i2c
        hardware_spi
        hardware_adc
        hardware_dma
        hardware_pwm
        hardware_exception
        )

    # target_compile_definitions(${PROJECT_NAME} PUBLIC PICO_DEFAULT_UART_TX_PIN=12)
    # target_compile_definitions(${PROJECT_NAME} PUBLIC PICO_DEFAULT_UART_RX_PIN=13)

pico_set_linker_script(smol_servo_rpico ${CMAKE_SOURCE_DIR}/src/mmap_smol_servo_rp2350.ld)

    # enable usb / uart output
    pico_enable_stdio_usb(smol_servo_rpico 1)
    pico_enable_stdio_uart(smol_servo_rpico 1)
    # do not enable semihosting
    pico_enable_stdio_semihosting(smol_servo_rpico 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(smol_servo_rpico)

    # for debugging flash access latency issues, use copy_to_ram to eliminate all flash access
    # pico_set_binary_type(smol_servo_rpico copy_to_ram)

    # add url via pico_set_program_url
elseif(PICO_ON_DEVICE)
    message("Skipping hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()

# assumes a pico-sdk install like done by the VS-Code plugin
set(OPENOCD_PATH "${PICO_SDK_PATH}/../../openocd/0.12.0+dev/")
set(PROGRAMMER "${OPENOCD_PATH}/openocd")
set(PROGRAMMER_FLAGS 
    "-s" "${OPENOCD_PATH}/scripts" 
    -f interface/cmsis-dap.cfg 
    -f target/rp2350.cfg 
    -c "adapter speed 1000")

set(GDB "${PICO_TOOLCHAIN_PATH}/bin/arm-none-eabi-gdb")

add_custom_target(swd_flash
    COMMAND ${PROGRAMMER} ${PROGRAMMER_FLAGS} 
    -c "program $<TARGET_FILE:smol_servo_rpico> verify"
    -c "init"
    -c "reset halt"
    -c "rp2350.cm1 arp_reset assert 0"
    -c "rp2350.cm0 arp_reset assert 0"
    -c "exit"
    DEPENDS smol_servo_rpico
    VERBATIM)

add_custom_target(swd_reset
    COMMAND ${PROGRAMMER} ${PROGRAMMER_FLAGS} 
    -c "init" 
    -c "reset halt"
    -c "rp2350.cm1 arp_reset assert 0"
    -c "rp2350.cm0 arp_reset assert 0"
    -c "exit"
    DEPENDS smol_servo_rpico
    VERBATIM)

add_custom_target(gdbhost
    ${PROGRAMMER} ${PROGRAMMER_FLAGS} 
    -c "init" 
    -c "reset halt"
    # -c "rp2350.dap.core1 cortex_m reset_config sysresetreq"
    DEPENDS  "$<TARGET_FILE:smol_servo_rpico>"
    COMMENT "Running OpenOCD as a GDB host.")

add_custom_target(debug DEPENDS ${PROJECT_NAME}.elf)
add_custom_command(TARGET debug
    #COMMAND bash "-c" "pgrep -x \"openocd\" || (echo \"Please start openocd\" && exit -1)"
    COMMAND echo "Starting GDB client and loading ${PROJECT_NAME}.elf dashboard to $(GDB_TTY)"
    COMMAND ${GDB} "$<TARGET_FILE:smol_servo_rpico>"
    -ex "target remote localhost:3333"
    # -ex "monitor swdp_scan"
    # -ex "dashboard -output $(GDB_TTY)"
    # -ex "load $<TARGET_FILE:smol_servo_rpico>"
    # -ex "monitor arm semihosting enable"
    )
