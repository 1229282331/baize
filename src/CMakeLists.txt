add_library(baize_host STATIC chassis.c baize_session.c serial_port.c protocol/baize_protocol.c)
target_compile_features(baize_host PUBLIC c_std_11)
set_target_properties(baize_host PROPERTIES POSITION_INDEPENDENT_CODE ON C_EXTENSIONS OFF)
target_include_directories(baize_host PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include
                                      PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/protocol)
if(UNIX)
    target_link_libraries(baize_host PRIVATE m)
endif()
