cmake_minimum_required(VERSION 3.22)
project(chassis_example LANGUAGES C CXX)

set(BAIZE_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." CACHE PATH "Baize source checkout")
add_subdirectory("${BAIZE_SOURCE_DIR}/host" baize-host EXCLUDE_FROM_ALL)
add_executable(chassis-example main.cpp)
target_compile_features(chassis-example PRIVATE cxx_std_11)
target_link_libraries(chassis-example PRIVATE baize_host)
