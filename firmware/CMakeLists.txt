cmake_minimum_required(VERSION 3.22)
project(baize_base LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(baize_base)
target_sources(baize_base PRIVATE
    ../protocol/baize_protocol.c
    Core/Src/base_feedback.c
    Core/Src/base_motion.c
    Core/Src/mpu6050.c
    control/baize_drive.c
    control/baize_encoders.c)
target_include_directories(baize_base PRIVATE ../protocol control)
add_subdirectory(cmake/stm32cubemx)
target_link_libraries(baize_base stm32cubemx)
