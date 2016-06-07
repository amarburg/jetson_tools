
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  # ${Boost_INCLUDE_DIR}
)

# build shared library.
add_library(libcompanion  SHARED
			Adafruit_BMP280.cpp )
# target_link_libraries(active_object
#   ${Boost_LIBRARIES}
# )
