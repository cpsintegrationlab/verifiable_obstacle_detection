/*
 * verifiable_obstacle_detection.cpp
 *
 *  Created on: Aug 30, 2022
 *      Author: simonyu
 */

#include <iostream>

#include "api/api.h"

using verifiable_obstacle_detection::Point2D;
using verifiable_obstacle_detection::Polygon;
using verifiable_obstacle_detection::VerifiableObstacleDetection;

int
main()
{
	std::vector<Polygon> detections_mission;
	std::vector<Polygon> detections_safety;

	Polygon detection_safety;
	Eigen::Vector3d detection_safety_center;
	Eigen::Vector3d detection_safety_extent;
	std::vector<Point2D> detection_safety_points;

	detection_safety_center.x() = 28.5936356;
	detection_safety_center.y() = -2.06204128;
	detection_safety_center.z() = 0.551451683;

	detection_safety_extent.x() = 2.44496346;
	detection_safety_extent.y() = 2.26267076;
	detection_safety_extent.z() = 1.07261384;

	detection_safety_points.push_back(
			Point2D(detection_safety_center.x() + detection_safety_extent.x() / 2,
					detection_safety_center.y() + detection_safety_extent.y() / 2));
	detection_safety_points.push_back(
			Point2D(detection_safety_center.x() + detection_safety_extent.x() / 2,
					detection_safety_center.y() - detection_safety_extent.y() / 2));
	detection_safety_points.push_back(
			Point2D(detection_safety_center.x() - detection_safety_extent.x() / 2,
					detection_safety_center.y() - detection_safety_extent.y() / 2));
	detection_safety_points.push_back(
			Point2D(detection_safety_center.x() - detection_safety_extent.x() / 2,
					detection_safety_center.y() + detection_safety_extent.y() / 2));

	boost::geometry::assign_points(detection_safety, detection_safety_points);
	boost::geometry::correct(detection_safety);

	detections_safety.push_back(detection_safety);

	VerifiableObstacleDetection verifiable_obstacle_detection;

	verifiable_obstacle_detection.initializeForApollo();

	const auto ego = verifiable_obstacle_detection.getEgo();

	std::cout << "ego " << boost::geometry::dsv(ego) << " has an area of "
			<< boost::geometry::area(ego) << "." << std::endl;

	std::cout << "detection_safety " << boost::geometry::dsv(detection_safety) << " has an area of "
			<< boost::geometry::area(detection_safety) << "." << std::endl;

	verifiable_obstacle_detection.processOneFrameForApollo("", detections_mission,
			detections_safety);

	const auto detections_safety_distance_end_points =
			verifiable_obstacle_detection.getDetectionsSafetyDistanceEndPoints();

	const auto& distance_end_points_ego = detections_safety_distance_end_points[0].first;
	const auto& distance_end_points_detection_safety = detections_safety_distance_end_points[0].second;

	std::cout << "distance_end_points_ego: " << boost::geometry::dsv(distance_end_points_ego) << "." << std::endl;
	std::cout << "distance_end_points_detection_safety: " << boost::geometry::dsv(distance_end_points_detection_safety) << "." << std::endl;

	return 0;
}
