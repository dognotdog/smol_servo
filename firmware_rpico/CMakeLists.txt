# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
set(PICO_BOARD smol_servo_rpico CACHE STRING "Board type")
# set(PICO_BOARD pico2 CACHE STRING "Board type")

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
        )

    # pull in common dependencies
    target_link_libraries(
        smol_servo_rpico 
        pico_stdlib 
        hardware_i2c
        hardware_spi
        )

    target_compile_definitions(${PROJECT_NAME} PUBLIC PICO_DEFAULT_UART_TX_PIN=12)
    target_compile_definitions(${PROJECT_NAME} PUBLIC PICO_DEFAULT_UART_RX_PIN=13)

    # enable usb output, disable uart output
    pico_enable_stdio_usb(smol_servo_rpico 1)
    pico_enable_stdio_uart(smol_servo_rpico 1)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(smol_servo_rpico)

    # add url via pico_set_program_url
elseif(PICO_ON_DEVICE)
    message("Skipping hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()

# assumes a pico-sdk install like done by the VS-Code plugin
set(OPENOCD_PATH "${PICO_SDK_PATH}/../../openocd/0.12.0+dev/")
set(PROGRAMMER "${OPENOCD_PATH}/openocd")
set(PROGRAMMER_FLAGS "-s" "${OPENOCD_PATH}/scripts" -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 1000")

add_custom_target(swd_flash
    COMMAND ${PROGRAMMER} ${PROGRAMMER_FLAGS} -c "program $<TARGET_FILE:smol_servo_rpico> verify reset exit"
    DEPENDS smol_servo_rpico
    VERBATIM)

add_custom_target(swd_reset
    COMMAND ${PROGRAMMER} ${PROGRAMMER_FLAGS} -c "init" -c "reset run" -c "exit"
    DEPENDS smol_servo_rpico
    VERBATIM)

add_custom_target(gdbhost
    ${PROGRAMMER} ${PROGRAMMER_FLAGS} -c "init" -c "reset halt"
    DEPENDS  "$<TARGET_FILE:smol_servo_rpico>"
    COMMENT "Running OpenOCD as a GDB host.")

add_custom_target(debug DEPENDS ${PROJECT_NAME}.elf)
add_custom_command(TARGET debug
    #COMMAND bash "-c" "pgrep -x \"openocd\" || (echo \"Please start openocd\" && exit -1)"
    COMMAND echo "Starting GDB client and loading ${PROJECT_NAME}.elf dashboard to $(GDB_TTY)"
    COMMAND ${CMAKE_C_GDB} "$<TARGET_FILE:smol_servo_rpico>"
    -ex "target extended :3333"
    -ex "dashboard -output $(GDB_TTY)"
    -ex "load $<TARGET_FILE:smol_servo_rpico>"
    -ex "monitor arm semihosting enable"
    )
