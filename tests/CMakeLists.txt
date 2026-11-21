cmake_minimum_required(VERSION 3.22)
project(baize_desktop_tests LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
enable_testing()

add_subdirectory(../host host)
add_library(baize_logic STATIC
    ../firmware/control/baize_drive.c
    ../firmware/control/baize_encoders.c)
target_include_directories(baize_logic PUBLIC ../firmware/control)
target_link_libraries(baize_logic PUBLIC baize_host)
if(NOT MSVC)
    target_link_libraries(baize_logic PUBLIC m)
endif()

foreach(area IN ITEMS protocol drive encoders session)
    add_executable(test_${area} test_${area}.c)
    target_link_libraries(test_${area} PRIVATE baize_logic)
    add_test(NAME ${area} COMMAND test_${area})
endforeach()

find_package(Python3 QUIET COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(NAME ground_geometry COMMAND ${Python3_EXECUTABLE} -B
        ${CMAKE_CURRENT_SOURCE_DIR}/test_ground_geometry.py)
    if(UNIX)
        add_test(NAME monitor_mixed_stream COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test_monitor.py $<TARGET_FILE:baize-monitor>)
        add_test(NAME timed_jog COMMAND ${Python3_EXECUTABLE} -B
            ${CMAKE_CURRENT_SOURCE_DIR}/test_jog.py $<TARGET_FILE:baize-monitor>)
    endif()
endif()
