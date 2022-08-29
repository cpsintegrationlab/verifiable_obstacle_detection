# Add executables
add_executable(verifiable_obstacle_detection verifiable_obstacle_detection.cpp)

# Set library link targets
if(ARCH STREQUAL "arm64")
	target_link_libraries(verifiable_obstacle_detection "-Wl,--no-whole-archive" vod_api)
else()
	target_link_libraries(verifiable_obstacle_detection vod_api)
endif()

# Set installation targets
install(
	TARGETS verifiable_obstacle_detection
	RUNTIME DESTINATION bin
)
