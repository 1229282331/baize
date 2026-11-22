cmake_minimum_required(VERSION 3.22)
project(baize_host_legacy LANGUAGES C)

# The v1.0.0 add_subdirectory(.../host) entry exposes the original raw headers.
if(NOT TARGET baize_host)
    add_subdirectory(../src src)
endif()
target_include_directories(baize_host INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/../src
                                                ${CMAKE_CURRENT_SOURCE_DIR}/../src/protocol)
if(NOT TARGET baize-monitor)
    add_subdirectory(../tools tools)
    set_target_properties(baize-monitor baize_panel_io PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
endif()
