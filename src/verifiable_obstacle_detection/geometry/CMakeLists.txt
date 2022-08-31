# Add libraries
add_library(
	vod_geometry SHARED
	distance.cpp
)

# Set header targets
file(GLOB VOD_GEOMETRY_HEADERS "*.h" "*.hpp")

# Set library target properties
set_target_properties(
	vod_geometry PROPERTIES
	PUBLIC_HEADER "${VOD_GEOMETRY_HEADERS}"
)

# Set library link targets
target_link_libraries(
	vod_geometry
	${Boost_LIBRARIES}
)

# Set installation targets
install(
	TARGETS vod_geometry
	LIBRARY DESTINATION lib
	PUBLIC_HEADER DESTINATION include/verifiable_obstacle_detection/geometry
)
