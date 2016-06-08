include_directories( ${PROJECT_SOURCE_DIR}/lib )

# build image node
add_executable(companion companion.cpp temperature_thread.cpp )
target_link_libraries( companion libcompanion )
