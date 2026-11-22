add_executable(baize-monitor monitor.c jog.c)
add_library(baize_panel_io SHARED panel/link.c jog.c)
foreach(tool IN ITEMS baize-monitor baize_panel_io)
    target_link_libraries(${tool} PRIVATE baize_host)
    target_include_directories(${tool} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../src ${CMAKE_CURRENT_SOURCE_DIR}/../src/protocol)
endforeach()
set_target_properties(baize_panel_io PROPERTIES PREFIX "" WINDOWS_EXPORT_ALL_SYMBOLS ON)
