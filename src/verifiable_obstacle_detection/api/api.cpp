/*
 * api.cpp
 *
 *  Created on: Aug 29, 2022
 *      Author: simonyu
 */

#include <iostream>

#include "api/api.h"

namespace verifiable_obstacle_detection
{
VerifiableObstacleDetection::VerifiableObstacleDetection()
{
	ego_extent_.x() = 5.02;
	ego_extent_.y() = 2.13;
	ego_extent_.z() = 1.50;
}

void
VerifiableObstacleDetection::disableConsoleLogging()
{
	std::cout.rdbuf(NULL);
	stdout = freopen("/dev/null", "w", stdout);
}

bool
VerifiableObstacleDetection::initializeForApollo()
{
	std::vector<Point2D> ego_points;

	ego_points.push_back(Point2D(ego_extent_.x() / 2, ego_extent_.y() / 2));
	ego_points.push_back(Point2D(ego_extent_.x() / 2, -1 * ego_extent_.y() / 2));
	ego_points.push_back(Point2D(-1 * ego_extent_.x() / 2, -1 * ego_extent_.y() / 2));
	ego_points.push_back(Point2D(-1 * ego_extent_.x() / 2, ego_extent_.y() / 2));

	boost::geometry::assign_points(ego_, ego_points);
	boost::geometry::correct(ego_);

	std::cout << "ego_ " << boost::geometry::dsv(ego_) << " has an area of "
			<< boost::geometry::area(ego_) << "." << std::endl;

	return true;
}

const std::string
VerifiableObstacleDetection::processOneFrameForApollo(const std::string& frame_name,
		const std::vector<Polygon>& detections_mission,
		const std::vector<Polygon>& detections_safety)
{
	// Iterate through all safety layer detections
	for (const auto &detection_safety : detections_safety)
	{
		const auto distance_end_points = distance_.getDistanceEndPoints(ego_, detection_safety);
		const auto distance = boost::geometry::distance(distance_end_points.first, distance_end_points.second);

		std::cout << "Closest point on detection_safety: " << boost::geometry::dsv(distance_end_points.second) << "." << std::endl;
		std::cout << "Distance from ego to detection_safety: " << distance << "." << std::endl;
	}

	return frame_name;
}
} // namespace verifiable_obstacle_detection

