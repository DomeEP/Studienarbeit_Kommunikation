set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER "E:/Programme/STI/STM32CubeCLT/STM32CubeCLT_1.20.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe")
set(CMAKE_CXX_COMPILER "E:/Programme/STI/STM32CubeCLT/STM32CubeCLT_1.20.0/GNU-tools-for-STM32/bin/arm-none-eabi-g++.exe")
set(CMAKE_ASM_COMPILER "E:/Programme/STI/STM32CubeCLT/STM32CubeCLT_1.20.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe")
set(CMAKE_OBJCOPY "E:/Programme/STI/STM32CubeCLT/STM32CubeCLT_1.20.0/GNU-tools-for-STM32/bin/arm-none-eabi-objcopy.exe")
set(CMAKE_SIZE "E:/Programme/STI/STM32CubeCLT/STM32CubeCLT_1.20.0/GNU-tools-for-STM32/bin/arm-none-eabi-size.exe")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(COMMON_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS "${COMMON_FLAGS}" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS}" CACHE INTERNAL "")
set(CMAKE_ASM_FLAGS "${COMMON_FLAGS}" CACHE INTERNAL "")
