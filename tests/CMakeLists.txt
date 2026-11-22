cmake_minimum_required(VERSION 3.22)
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    project(baize_desktop_tests LANGUAGES C)
    enable_testing()
    add_subdirectory(../host host)
    add_subdirectory(../examples/c example-c)
    add_subdirectory(../examples/chassis example-cpp)
endif()
add_library(baize_logic STATIC
    ../firmware/control/drive.c
    ../firmware/control/encoders.c)
target_include_directories(baize_logic PUBLIC ../firmware/control ../src/protocol ../src)
target_link_libraries(baize_logic PUBLIC baize_host)
if(NOT MSVC)
    target_link_libraries(baize_logic PUBLIC m)
endif()

foreach(area IN ITEMS protocol drive encoders session)
    add_executable(test_${area} test_${area}.c)
    target_link_libraries(test_${area} PRIVATE baize_logic)
    add_test(NAME ${area} COMMAND test_${area})
endforeach()

add_executable(test_chassis test_chassis.c)
target_include_directories(test_chassis PRIVATE ../src ../src/protocol)
target_link_libraries(test_chassis PRIVATE baize_host)
add_test(NAME chassis COMMAND test_chassis)

find_package(Python3 QUIET COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(NAME ground_geometry COMMAND ${Python3_EXECUTABLE} -B
        ${CMAKE_CURRENT_SOURCE_DIR}/test_ground_geometry.py)
    add_test(NAME imu_calibration COMMAND ${Python3_EXECUTABLE} -B
        ${CMAKE_CURRENT_SOURCE_DIR}/test_imu_calibration.py)
    if(Python3_VERSION VERSION_GREATER_EQUAL "3.10")
        add_test(NAME panel_results COMMAND ${Python3_EXECUTABLE} -B
            ${CMAKE_CURRENT_SOURCE_DIR}/test_panel_results.py $<TARGET_FILE:baize_panel_io>)
        if(UNIX)
            add_test(NAME panel_serial COMMAND ${Python3_EXECUTABLE} -B
                ${CMAKE_CURRENT_SOURCE_DIR}/test_panel_serial.py $<TARGET_FILE:baize_panel_io>)
        endif()
    endif()
    if(UNIX)
        add_test(NAME chassis_examples COMMAND ${Python3_EXECUTABLE} -B
            ${CMAKE_CURRENT_SOURCE_DIR}/test_chassis_serial.py $<TARGET_FILE:chassis-c-example>
            $<TARGET_FILE:chassis-example>)
        add_test(NAME monitor_mixed_stream COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test_monitor.py $<TARGET_FILE:baize-monitor>)
        add_test(NAME timed_jog COMMAND ${Python3_EXECUTABLE} -B
            ${CMAKE_CURRENT_SOURCE_DIR}/test_jog.py $<TARGET_FILE:baize-monitor>)
        add_test(NAME ground_trial COMMAND ${Python3_EXECUTABLE} -B
            ${CMAKE_CURRENT_SOURCE_DIR}/test_ground_trial.py $<TARGET_FILE:baize-monitor>)
    endif()
endif()
