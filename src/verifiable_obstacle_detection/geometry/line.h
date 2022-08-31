/*
 * line.h
 *
 *  Created on: Aug 31, 2022
 *      Author: simonyu
 */

#ifndef GEOMETRY_LINE_H_
#define GEOMETRY_LINE_H_

#include "geometry/geometry.h"

namespace verifiable_obstacle_detection
{
class Line
{
public:

	Line();

	Line(const double& slope);

	Line(const double& slope, const double& intercept);

	Line(const Point2D& point_1, const Point2D& point_2);

	double
	getSlope() const;

	double
	getIntercept() const;

	Line
	getPerpendicular(const Point2D& point) const;

	Point2D
	getIntersection(const Line& line) const;

private:

	double slope_;
	double intercept_;
};
} // namespace verifiable_obstacle_detection

#endif /* GEOMETRY_LINE_H_ */
