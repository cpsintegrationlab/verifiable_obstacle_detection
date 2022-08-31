/*
 * distance.h
 *
 *  Created on: Aug 30, 2022
 *      Author: simonyu
 */

#ifndef GEOMETRY_DISTANCE_H_
#define GEOMETRY_DISTANCE_H_

#include "geometry/geometry.h"

namespace verifiable_obstacle_detection
{
class Distance
{
public:

	Distance();

	std::pair<Point2D, Point2D>
	getDistanceEndPoints(const Polygon& polygon_1, const Polygon& polygon_2);

private:

	Point2D
	getDistanceEndPoint(const Point2D& start_point, const Point2D& segment_point_1,
			const Point2D& segment_point_2);
};
} // namespace verifiable_obstacle_detection

#endif /* GEOMETRY_DISTANCE_H_ */
