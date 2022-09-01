/*
 * api.h
 *
 *  Created on: Aug 29, 2022
 *      Author: simonyu
 */

#ifndef API_API_H_
#define API_API_H_

#include <memory>
#include <tuple>
#include <vector>

#include "geometry/distance.h"
#include "geometry/geometry.h"

namespace verifiable_obstacle_detection
{
class VerifiableObstacleDetection
{
public:

	VerifiableObstacleDetection();

	std::vector<std::pair<Point2D, Point2D>>
	getDetectionsSafetyDistanceEndPoints() const;

	std::vector<std::pair<Point2D, Point2D>>
	getSegmentsProjectedMission() const;

	std::vector<std::pair<Point2D, Point2D>>
	getSegmentsProjectedSafety() const;

	std::vector<std::pair<Point2D, Point2D>>
	getSegmentsOverlap() const;

	std::vector<double>
	getDetectionsSafetyCoverages() const;

	Polygon
	getEgo() const;

	void
	disableConsoleLogging();

	bool
	initializeForApollo();

	void
	processOneFrameForApollo(const std::vector<Polygon>& detections_mission,
			const std::vector<Polygon>& detections_safety);

private:

	void
	clear();

	std::pair<Point2D, Point2D>
	findEndPoints(const std::vector<Point2D>& points);

	std::pair<Point2D, Point2D>
	findOverlap(const std::pair<Point2D, Point2D>& segment_1,
			const std::pair<Point2D, Point2D>& segment_2);

	double
	findLength(const std::pair<Point2D, Point2D>& segment);

	void
	plot(const std::vector<Polygon>& detections_mission,
			const std::vector<Polygon>& detections_safety);

	Distance distance_;
	Polygon ego_;
	Point2D ego_extent_;

	std::vector<std::pair<Point2D, Point2D>> detections_safety_distance_end_points_; // <ego, detections_safety>
	std::vector<std::pair<Point2D, Point2D>> segments_projected_mission_;
	std::vector<std::pair<Point2D, Point2D>> segments_projected_safety_;
	std::vector<std::pair<Point2D, Point2D>> segments_overlap_;
	std::vector<double> detections_safety_coverages_;

	bool plot_;
};
} // namespace verifiable_obstacle_detection

#endif /* API_API_H_ */
