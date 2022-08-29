# Add libraries
add_library(
	vod_api SHARED
	api.cpp
)

# Set header targets
file(GLOB VOD_API_HEADERS "*.h" "*.hpp")

# Set library target properties
set_target_properties(
	vod_api PROPERTIES
	PUBLIC_HEADER "${VOD_API_HEADERS}"
)

# Set library link targets
target_link_libraries(
	vod_api
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
)

# Set installation targets
install(
	TARGETS vod_api
	LIBRARY DESTINATION lib
	PUBLIC_HEADER DESTINATION include/verifiable_obstacle_detection/api
)
