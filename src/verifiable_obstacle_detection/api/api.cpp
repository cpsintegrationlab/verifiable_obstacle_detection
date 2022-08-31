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

std::vector<std::pair<Point2D, Point2D>>
VerifiableObstacleDetection::getDetectionsSafetyDistanceEndPoints() const
{
	return detections_safety_distance_end_points_;
}

Polygon
VerifiableObstacleDetection::getEgo() const
{
	return ego_;
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

	return true;
}

void
VerifiableObstacleDetection::processOneFrameForApollo(
		const std::vector<Polygon>& detections_mission,
		const std::vector<Polygon>& detections_safety)
{
	detections_safety_distance_end_points_.clear();

	// Iterate through all safety layer detections
	for (const auto &detection_safety : detections_safety)
	{
		const auto distance_end_points = distance_.getDistanceEndPoints(ego_, detection_safety);

		detections_safety_distance_end_points_.push_back(distance_end_points);
	}
}
} // namespace verifiable_obstacle_detection

