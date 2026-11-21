cmake_minimum_required(VERSION 3.22)
project(baize_desktop_tests LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
enable_testing()

add_library(baize_logic STATIC
    ../protocol/baize_protocol.c
    ../firmware/control/baize_drive.c
    ../firmware/control/baize_encoders.c
    ../host/baize_session.c)
target_include_directories(baize_logic PUBLIC ../protocol ../firmware/control ../host)

foreach(area IN ITEMS protocol drive encoders session)
    add_executable(test_${area} test_${area}.c)
    target_link_libraries(test_${area} PRIVATE baize_logic)
    add_test(NAME ${area} COMMAND test_${area})
endforeach()
