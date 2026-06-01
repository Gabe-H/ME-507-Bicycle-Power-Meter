target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/eekf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/eekf_mat.c
)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/includes)