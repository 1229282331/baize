cmake_minimum_required(VERSION 3.22)
project(baize_host LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
add_executable(baize-monitor monitor.c serial_port.c baize_session.c ../protocol/baize_protocol.c)
target_include_directories(baize-monitor PRIVATE ../protocol)
