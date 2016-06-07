include_directories( ${PROJECT_SOURCE_DIR}/lib )

# build image node
add_executable(temperature temperature.cpp )
target_link_libraries( temperature libcompanion )
