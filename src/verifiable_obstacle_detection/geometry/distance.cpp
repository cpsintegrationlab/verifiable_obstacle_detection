/*
 * distance.cpp
 *
 *  Created on: Aug 30, 2022
 *      Author: simonyu
 */

#include <tuple>

#include "geometry/distance.h"

namespace verifiable_obstacle_detection
{
Distance::Distance()
{
}

std::pair<Point2D, Point2D>
Distance::getDistanceEndPoints(const Polygon& polygon_1, const Polygon& polygon_2)
{
	auto distance_polygon_1 = std::numeric_limits<double>::max();
	auto distance_polygon_2 = std::numeric_limits<double>::max();

	Point2D start_point_polygon_1;
	Point2D end_point_polygon_1;
	Point2D start_point_polygon_2;
	Point2D end_point_polygon_2;

	// Iterate through all vertices on polygon 1
	for (const auto &start_point : polygon_1.outer())
	{
		// Iterate through all vertices on polygon 2
		for (int current = 0; current < polygon_2.outer().size(); current++)
		{
			int next = (current + 1) % polygon_2.outer().size();
			const Point2D& segment_point_1 = polygon_2.outer()[current];
			const Point2D& segment_point_2 = polygon_2.outer()[next];

			Point2D end_point = getDistanceEndPoint(start_point, segment_point_1, segment_point_2);
			auto distance = boost::geometry::distance(start_point, end_point);

			if (distance < distance_polygon_1)
			{
				distance_polygon_1 = distance;
				start_point_polygon_1 = start_point;
				end_point_polygon_1 = end_point;
			}
		}
	}

	// Iterate through all vertices on polygon 2
	for (const auto &start_point : polygon_2.outer())
	{
		// Iterate through all vertices on polygon 1
		for (int current = 0; current < polygon_1.outer().size(); current++)
		{
			int next = (current + 1) % polygon_1.outer().size();
			const Point2D& segment_point_1 = polygon_1.outer()[current];
			const Point2D& segment_point_2 = polygon_1.outer()[next];

			Point2D end_point = getDistanceEndPoint(start_point, segment_point_1, segment_point_2);
			auto distance = boost::geometry::distance(start_point, end_point);

			if (distance < distance_polygon_2)
			{
				distance_polygon_2 = distance;
				start_point_polygon_2 = start_point;
				end_point_polygon_2 = end_point;
			}
		}
	}

	if (distance_polygon_1 < distance_polygon_2)
	{
		return std::make_pair(start_point_polygon_1, end_point_polygon_1);
	}
	else
	{
		return std::make_pair(end_point_polygon_2, start_point_polygon_2);
	}
}

Point2D
Distance::getDistanceEndPoint(const Point2D& start_point, const Point2D& segment_point_1,
		const Point2D& segment_point_2)
{
	Point2D u = segment_point_1;
	Point2D v = segment_point_2;
	Point2D w = start_point;

	boost::geometry::subtract_point(v, segment_point_1);
	boost::geometry::subtract_point(w, segment_point_1);

	auto const c1 = boost::geometry::dot_product(w, v);
	auto const c2 = boost::geometry::dot_product(v, v);

	if (c1 <= 0)
	{
		return segment_point_1;
	}

	if (c2 <= c1)
	{
		return segment_point_2;
	}

	boost::geometry::multiply_value(v, c1 / c2);
	boost::geometry::add_point(u, v);

	return u;
}
} // namespace verifiable_obstacle_detection
