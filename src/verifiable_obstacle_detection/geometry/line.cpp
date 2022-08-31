/*
 * line.cpp
 *
 *  Created on: Aug 31, 2022
 *      Author: simonyu
 */

#include <iostream>

#include "geometry/line.h"

namespace verifiable_obstacle_detection
{
Line::Line() :
		Line(0, 0)
{
}

Line::Line(const double& slope) :
		Line(slope, 0)
{
}

Line::Line(const double& slope, const double& intercept)
{
	slope_ = slope;
	intercept_ = intercept;
}

Line::Line(const Point2D& point_1, const Point2D& point_2) :
		Line()
{
	if (point_1.x() == point_2.x())
	{
		std::cerr << "[ERROR]: Invalid points." << std::endl;
		return;
	}

	slope_ = (point_1.y() - point_2.y()) / (point_1.x() - point_2.x());
	intercept_ = point_1.y() - slope_ * point_1.x();
}

double
Line::getSlope() const
{
	return slope_;
}

double
Line::getIntercept() const
{
	return intercept_;
}

Line
Line::getPerpendicular(const Point2D& point) const
{
	auto slope_perpendicular = -1 / slope_;

	return Line(slope_perpendicular, point.y() - slope_perpendicular * point.x());
}

Point2D
Line::getIntersection(const Line& line) const
{
	if (line.getSlope() == slope_)
	{
		std::cerr << "[ERROR]: Invalid line." << std::endl;
		return Point2D();
	}

	Point2D intersection;

	intersection.x((line.getIntercept() - intercept_) / (slope_ - line.getSlope()));
	intersection.y(slope_ * intersection.x() + intercept_);

	return intersection;
}
} // namespace verifiable_obstacle_detection
