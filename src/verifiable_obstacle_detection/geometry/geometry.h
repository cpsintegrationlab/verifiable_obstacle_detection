/*
 * geometry.h
 *
 *  Created on: Aug 30, 2022
 *      Author: simonyu
 */

#ifndef GEOMETRY_GEOMETRY_H_
#define GEOMETRY_GEOMETRY_H_

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/strategies/cartesian/distance_pythagoras.hpp>

namespace verifiable_obstacle_detection
{
using Point2D = boost::geometry::model::d2::point_xy<double>;
using Polygon = boost::geometry::model::polygon<Point2D>;
using Segment = boost::geometry::model::segment<Point2D>;
} // namespace verifiable_obstacle_detection

#endif /* GEOMETRY_GEOMETRY_H_ */
