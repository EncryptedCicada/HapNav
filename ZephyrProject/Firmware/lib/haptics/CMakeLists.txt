target_sources(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src/pca9546a.c
    ${CMAKE_CURRENT_LIST_DIR}/src/da7280.c
    ${CMAKE_CURRENT_LIST_DIR}/src/haptics.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
