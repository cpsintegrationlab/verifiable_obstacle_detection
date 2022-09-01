/*
 * api.cpp
 *
 *  Created on: Aug 29, 2022
 *      Author: simonyu
 */

#include <cmath>
#include <fstream>
#include <iostream>

#include "api/api.h"
#include "geometry/line.h"

namespace verifiable_obstacle_detection
{
VerifiableObstacleDetection::VerifiableObstacleDetection() :
		plot_(true)
{
	ego_extent_.x(5.02);
	ego_extent_.y(2.13);
}

std::vector<std::pair<Point2D, Point2D>>
VerifiableObstacleDetection::getSegmentsProjectedMission() const
{
	return segments_projected_mission_;
}

std::vector<std::pair<Point2D, Point2D>>
VerifiableObstacleDetection::getSegmentsProjectedSafety() const
{
	return segments_projected_safety_;
}

std::vector<std::pair<Point2D, Point2D>>
VerifiableObstacleDetection::getSegmentsOverlap() const
{
	return segments_overlap_;
}

std::vector<double>
VerifiableObstacleDetection::getDetectionsSafetyCoverages() const
{
	return detections_safety_coverages_;
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
	plot_ = false;
}

bool
VerifiableObstacleDetection::initializeForApollo(const std::string& log_path)
{
	log_path_ = log_path;

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
VerifiableObstacleDetection::processOneFrameForApollo(const std::string& frame_name,
		const std::vector<Polygon>& detections_mission,
		const std::vector<Polygon>& detections_safety)
{
	clear();

	// Iterate through all safety layer detections
	for (const auto &detection_safety : detections_safety)
	{
		std::vector<Point2D> points_projected_mission;
		std::vector<Point2D> points_projected_safety;

		const auto distance_end_points = distance_.getDistanceEndPoints(ego_, detection_safety);

		detections_safety_distance_end_points_.push_back(distance_end_points);

		const auto line_ego_center_detection_safety = Line(Point2D(0, 0),
				distance_end_points.second);
		const auto line_detection_safety_perpendicular =
				line_ego_center_detection_safety.getPerpendicular(distance_end_points.second);

		// Iterate through all vertices on current safety detection polygon
		for (const auto &point : detection_safety.outer())
		{
			const auto line_ego_center_point = Line(Point2D(0, 0), point);
			const auto point_projected = line_ego_center_point.getIntersection(
					line_detection_safety_perpendicular);

			points_projected_safety.push_back(point_projected);
		}

		// Iterate through all mission layer detections
		for (const auto &detection_mission : detections_mission)
		{
			// Iterate through all vertices on current mission detection polygon
			for (const auto &point : detection_mission.outer())
			{
				const auto line_ego_center_point = Line(Point2D(0, 0), point);
				const auto point_projected = line_ego_center_point.getIntersection(
						line_detection_safety_perpendicular);

				points_projected_mission.push_back(point_projected);
			}
		}

		const auto segment_projected_mission = findEndPoints(points_projected_mission);
		const auto segment_projected_safety = findEndPoints(points_projected_safety);

		segments_projected_mission_.push_back(segment_projected_mission);
		segments_projected_safety_.push_back(segment_projected_safety);

		const auto segment_overlap = findOverlap(segment_projected_mission,
				segment_projected_safety);

		segments_overlap_.push_back(segment_overlap);

		const auto segment_projected_safety_length = findLength(segment_projected_safety);
		const auto segment_overlap_length = findLength(segment_overlap);

		if (segment_projected_safety_length == 0)
		{
			std::cerr << "[ERROR]: Invalid safety detection projected segment length." << std::endl;
			detections_safety_coverages_.push_back(0);
		}
		else
		{
			detections_safety_coverages_.push_back(
					segment_overlap_length / segment_projected_safety_length);
		}
	}

	plot(frame_name, detections_mission, detections_safety);
}

void
VerifiableObstacleDetection::clear()
{
	detections_safety_distance_end_points_.clear();
	segments_projected_mission_.clear();
	segments_projected_safety_.clear();
	detections_safety_coverages_.clear();
}

std::pair<Point2D, Point2D>
VerifiableObstacleDetection::findEndPoints(const std::vector<Point2D>& points)
{
	unsigned i_min = 0;
	unsigned i_max = 0;

	for (unsigned i = 0; i < points.size(); i++)
	{
		if (points[i].x() < points[i_min].x())
		{
			i_min = i;
		}

		if (points[i].x() > points[i_max].x())
		{
			i_max = i;
		}
	}

	if (i_min != i_max)
	{
		return std::make_pair(points[i_min], points[i_max]);
	}

	for (unsigned i = 0; i < points.size(); i++)
	{
		if (points[i].y() < points[i_min].y())
		{
			i_min = i;
		}
		if (points[i].y() > points[i_max].y())
		{
			i_max = i;
		}
	}

	if (points.empty())
	{
		return std::make_pair(Point2D(0, 0), Point2D(0, 0));
	}
	else
	{
		return std::make_pair(points[i_min], points[i_max]);
	}
}

std::pair<Point2D, Point2D>
VerifiableObstacleDetection::findOverlap(const std::pair<Point2D, Point2D>& segment_1,
		const std::pair<Point2D, Point2D>& segment_2)
{
	Point2D segment_1_point_1;
	Point2D segment_1_point_2;
	Point2D segment_2_point_1;
	Point2D segment_2_point_2;

	if (segment_1.first.x() >= segment_1.second.x())
	{
		segment_1_point_1 = segment_1.second;
		segment_1_point_2 = segment_1.first;
	}
	else
	{
		segment_1_point_1 = segment_1.first;
		segment_1_point_2 = segment_1.second;
	}

	if (segment_2.first.x() >= segment_2.second.x())
	{
		segment_2_point_1 = segment_2.second;
		segment_2_point_2 = segment_2.first;
	}
	else
	{
		segment_2_point_1 = segment_2.first;
		segment_2_point_2 = segment_2.second;
	}

// If segments do not overlap
	if (segment_1_point_2.x() <= segment_2_point_1.x()
			|| segment_1_point_1.x() >= segment_2_point_2.x())
	{
		return std::make_pair(Point2D(0, 0), Point2D(0, 0));
	}
// If segment 1 contains segment 2
	else if (segment_2_point_1.x() >= segment_1_point_1.x()
			&& segment_2_point_2.x() <= segment_1_point_2.x())
	{
		return std::make_pair(segment_2_point_1, segment_2_point_2);
	}
// If segment 2 contains segment 1
	else if (segment_1_point_1.x() >= segment_2_point_1.x()
			&& segment_1_point_2.x() <= segment_2_point_2.x())
	{
		return std::make_pair(segment_1_point_1, segment_1_point_2);
	}
// If segment 1 overlaps with segment 2
	else if (segment_2_point_1.x() >= segment_1_point_1.x()
			&& segment_1_point_2.x() <= segment_2_point_2.x()
			&& segment_2_point_1.x() <= segment_1_point_2.x())
	{
		return std::make_pair(segment_2_point_1, segment_1_point_2);
	}
// If segment 2 overlaps with segment 1
	else if (segment_1_point_1.x() >= segment_2_point_1.x()
			&& segment_2_point_2.x() <= segment_1_point_2.x()
			&& segment_1_point_1.x() <= segment_2_point_2.x())
	{
		return std::make_pair(segment_1_point_1, segment_2_point_2);
	}
	else
	{
		std::cerr << "[ERROR]: Unexpected error in finding overlap." << std::endl;
		return std::make_pair(Point2D(0, 0), Point2D(0, 0));
	}
}

double
VerifiableObstacleDetection::findLength(const std::pair<Point2D, Point2D>& segment)
{
	const auto segment_x_error = segment.first.x() - segment.second.x();
	const auto segment_y_error = segment.first.y() - segment.second.y();
	const auto segment_x_error_squared = segment_x_error * segment_x_error;
	const auto segment_y_error_squared = segment_y_error * segment_y_error;

	return sqrt(segment_x_error_squared + segment_y_error_squared);
}

void
VerifiableObstacleDetection::plot(const std::string& frame_name,
		const std::vector<Polygon>& detections_mission,
		const std::vector<Polygon>& detections_safety)
{
	if (!plot_)
	{
		return;
	}

	std::ofstream log_file(log_path_ + "/" + frame_name + ".svg",
			std::fstream::out | std::fstream::trunc);

	if (!log_file.is_open() || !log_file.good())
	{
		std::cerr << "[ERROR]: Failed to open log file." << std::endl;
		return;
	}

	std::unique_ptr<boost::geometry::svg_mapper<Point2D>> mapper(
			new boost::geometry::svg_mapper<Point2D>(log_file, 200, 200,
					"style='fill-opacity:1;fill:rgb(255,255,255)'"));

	mapper->add(ego_);
	mapper->map(ego_, "fill-opacity:0;fill:rgb(0,0,0);stroke:rgb(0,0,255);stroke-width:5");

	// Iterate through all mission layer detections
	for (const auto &detection_mission : detections_mission)
	{
		mapper->add(detection_mission);
		mapper->map(detection_mission,
				"fill-opacity:0;fill:rgb(0,0,0);stroke:rgb(255,0,0);stroke-width:5");
	}

	// Iterate through all safety layer detections
	for (const auto &detection_safety : detections_safety)
	{
		mapper->add(detection_safety);
		mapper->map(detection_safety,
				"fill-opacity:0;fill:rgb(0,0,0);stroke:rgb(0,255,0);stroke-width:5");
	}

	// Iterate through all safety layer detections distance end points
	for (const auto &detection_safety_distance_end_points_ : detections_safety_distance_end_points_)
	{
		const Segment segment(detection_safety_distance_end_points_.first,
				detection_safety_distance_end_points_.second);
		mapper->add(segment);
		mapper->map(segment, "fill-opacity:0;fill:rgb(0,0,0);stroke:rgb(0,0,255);stroke-width:5");
	}

	// Iterate through all mission layer projected segments
	for (const auto &segment_projected_mission_ : segments_projected_mission_)
	{
		const Segment segment(segment_projected_mission_.first, segment_projected_mission_.second);
		mapper->add(segment);
		mapper->map(segment, "fill-opacity:0;fill:rgb(0,0,0);stroke:rgb(255,0,0);stroke-width:5");
	}

	// Iterate through all safety layer projected segments
	for (const auto &segment_projected_safety_ : segments_projected_safety_)
	{
		const Segment segment(segment_projected_safety_.first, segment_projected_safety_.second);
		mapper->add(segment);
		mapper->map(segment, "fill-opacity:0;fill:rgb(0,0,0);stroke:rgb(0,255,0);stroke-width:5");
	}

	// Iterate through all overlap segments
	for (const auto &segment_overlap_ : segments_overlap_)
	{
		const Segment segment(segment_overlap_.first, segment_overlap_.second);
		mapper->add(segment);
		mapper->map(segment, "fill-opacity:0;fill:rgb(0,0,0);stroke:rgb(255,255,0);stroke-width:5");
	}

	mapper.reset();
	log_file.close();
}
} // namespace verifiable_obstacle_detection

