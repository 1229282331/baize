cmake_minimum_required(VERSION 3.22)
project(baize_host LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
add_library(baize_host STATIC baize_session.c serial_port.c ../protocol/baize_protocol.c)
set_target_properties(baize_host PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(baize_host PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../protocol)

add_executable(baize-monitor monitor.c jog.c)
target_link_libraries(baize-monitor PRIVATE baize_host)

add_library(baize_panel_io SHARED panel/link.c jog.c)
target_link_libraries(baize_panel_io PRIVATE baize_host)
set_target_properties(baize_panel_io PROPERTIES PREFIX "" WINDOWS_EXPORT_ALL_SYMBOLS ON)
