target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/cps.c
    ${CMAKE_CURRENT_SOURCE_DIR}/cscs.c
)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})