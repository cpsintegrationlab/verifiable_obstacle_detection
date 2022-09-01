/*
 * verifiable_obstacle_detection.cpp
 *
 *  Created on: Aug 30, 2022
 *      Author: simonyu
 */

#include <Eigen/Dense>
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

	Polygon detection_mission;
	Polygon detection_safety;

	Eigen::Vector3d detection_mission_center;
	Eigen::Vector3d detection_mission_extent;
	std::vector<Point2D> detection_mission_points;

	Eigen::Vector3d detection_safety_center;
	Eigen::Vector3d detection_safety_extent;
	std::vector<Point2D> detection_safety_points;

	detection_mission_center.x() = 28.5936356 - 5;
	detection_mission_center.y() = -2.06204128 - 10;
	detection_mission_center.z() = 0.551451683;

	detection_mission_extent.x() = 2.44496346 + 1;
	detection_mission_extent.y() = 2.26267076 + 1;
	detection_mission_extent.z() = 1.07261384 + 1;

	detection_safety_center.x() = 28.5936356;
	detection_safety_center.y() = -2.06204128 - 10;
	detection_safety_center.z() = 0.551451683;

	detection_safety_extent.x() = 2.44496346;
	detection_safety_extent.y() = 2.26267076;
	detection_safety_extent.z() = 1.07261384;

	detection_mission_points.push_back(
			Point2D(detection_mission_center.x() + detection_mission_extent.x() / 2,
					detection_mission_center.y() + detection_mission_extent.y() / 2));
	detection_mission_points.push_back(
			Point2D(detection_mission_center.x() + detection_mission_extent.x() / 2,
					detection_mission_center.y() - detection_mission_extent.y() / 2));
	detection_mission_points.push_back(
			Point2D(detection_mission_center.x() - detection_mission_extent.x() / 2,
					detection_mission_center.y() - detection_mission_extent.y() / 2));
	detection_mission_points.push_back(
			Point2D(detection_mission_center.x() - detection_mission_extent.x() / 2,
					detection_mission_center.y() + detection_mission_extent.y() / 2));

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

	boost::geometry::assign_points(detection_mission, detection_mission_points);
	boost::geometry::correct(detection_mission);

	boost::geometry::assign_points(detection_safety, detection_safety_points);
	boost::geometry::correct(detection_safety);

	detections_mission.push_back(detection_mission);
	detections_safety.push_back(detection_safety);

	VerifiableObstacleDetection verifiable_obstacle_detection;

	verifiable_obstacle_detection.initializeForApollo();

	const auto ego = verifiable_obstacle_detection.getEgo();

	std::cout << "ego " << boost::geometry::dsv(ego) << " has an area of "
			<< boost::geometry::area(ego) << "." << std::endl;

	std::cout << "detection_mission " << boost::geometry::dsv(detection_mission)
			<< " has an area of " << boost::geometry::area(detection_mission) << "." << std::endl;

	std::cout << "detection_safety " << boost::geometry::dsv(detection_safety) << " has an area of "
			<< boost::geometry::area(detection_safety) << "." << std::endl;

	verifiable_obstacle_detection.processOneFrameForApollo(detections_mission, detections_safety);

	const auto detections_safety_distance_end_points =
			verifiable_obstacle_detection.getDetectionsSafetyDistanceEndPoints();

	return 0;
}
