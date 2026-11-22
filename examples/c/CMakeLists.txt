cmake_minimum_required(VERSION 3.22)
project(chassis_c_example LANGUAGES C)

if(NOT TARGET baize_host)
    set(BAIZE_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." CACHE PATH "Baize source checkout")
    add_subdirectory("${BAIZE_SOURCE_DIR}" baize EXCLUDE_FROM_ALL)
endif()
add_executable(chassis-c-example main.c)
target_link_libraries(chassis-c-example PRIVATE baize_host)
