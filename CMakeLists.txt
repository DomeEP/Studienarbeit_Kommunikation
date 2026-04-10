cmake_minimum_required(VERSION 3.22)

project(Studienarbeit_Kommunikation C ASM)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_EXECUTABLE_SUFFIX ".elf")

# ============================================================================
# BOARD_ROLE Selection: "master" or "slave"
# USE_MODBUS: ON = Modbus-Firmware, OFF = Raw RS485 Test
#
# Beispiele:
#   cmake -B build_master -DBOARD_ROLE=master -DUSE_MODBUS=OFF  (Raw-Test)
#   cmake -B build_master -DBOARD_ROLE=master -DUSE_MODBUS=ON   (Modbus)
# ============================================================================
if(NOT BOARD_ROLE)
    set(BOARD_ROLE "master" CACHE STRING "Board role: master or slave")
endif()

option(USE_MODBUS "Enable Modbus RTU protocol (OFF = raw RS485 test)" ON)

message(STATUS ">>> BOARD_ROLE = ${BOARD_ROLE}")
message(STATUS ">>> USE_MODBUS = ${USE_MODBUS}")

if(BOARD_ROLE STREQUAL "master")
    set(MAIN_SOURCE Core/Src/main_master.c)
elseif(BOARD_ROLE STREQUAL "slave")
    set(MAIN_SOURCE Core/Src/main_slave.c)
else()
    message(FATAL_ERROR "Invalid BOARD_ROLE '${BOARD_ROLE}'. Use 'master' or 'slave'.")
endif()

# Source files (immer benötigt)
set(SOURCES
    ${MAIN_SOURCE}
    Core/Src/gpio.c
    Core/Src/i2c.c
    Core/Src/usart.c
    Core/Src/stm32g4xx_it.c
    Core/Src/stm32g4xx_hal_msp.c
    Core/Src/sysmem.c
    Core/Src/syscalls.c
    Core/Src/system_stm32g4xx.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c
    startup_stm32g474xx.s
)

# Modbus-spezifische Dateien nur wenn USE_MODBUS=ON
if(USE_MODBUS)
    list(APPEND SOURCES
        Core/Src/tim.c
        Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
        Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
        Drivers/Modbus/modbus_rtu.c
    )
    if(BOARD_ROLE STREQUAL "slave")
        list(APPEND SOURCES Drivers/SlaveNode/slave_node.c)
    endif()
    if(BOARD_ROLE STREQUAL "master")
        list(APPEND SOURCES Drivers/Arbiter/system_arbiter.c)
    endif()
endif()

# Include directories
set(INCLUDES
    Core/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Drivers/CMSIS/Include
    Drivers/Modbus
    Drivers/SlaveNode
    Drivers/Arbiter
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDES})

# Compile definitions
set(COMPILE_DEFS USE_HAL_DRIVER STM32G474xx $<$<CONFIG:Debug>:DEBUG>)
if(USE_MODBUS)
    list(APPEND COMPILE_DEFS USE_MODBUS)
endif()
target_compile_definitions(${PROJECT_NAME} PRIVATE ${COMPILE_DEFS})

# Post-build commands
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}${CMAKE_EXECUTABLE_SUFFIX} ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}${CMAKE_EXECUTABLE_SUFFIX} ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}${CMAKE_EXECUTABLE_SUFFIX}
)
