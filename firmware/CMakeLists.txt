cmake_minimum_required(VERSION 3.22)
project(baize_base LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(baize_base)
target_sources(baize_base PRIVATE
    ../src/protocol/baize_protocol.c
    Core/Src/base_feedback.c
    Core/Src/base_motion.c
    Core/Src/maintenance.c
    Core/Src/mpu6050.c
    control/drive.c
    control/encoders.c)
target_include_directories(baize_base PRIVATE ../src/protocol control)
add_subdirectory(cmake/stm32cubemx)
target_link_libraries(baize_base stm32cubemx)
add_custom_command(TARGET baize_base POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:baize_base> ${CMAKE_CURRENT_BINARY_DIR}/chassis.bin)
