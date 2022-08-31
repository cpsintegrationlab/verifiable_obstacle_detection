/*
 * api.h
 *
 *  Created on: Aug 29, 2022
 *      Author: simonyu
 */

#ifndef API_API_H_
#define API_API_H_

#include <Eigen/Dense>
#include <memory>

#include "geometry/distance.h"
#include "geometry/geometry.h"

namespace verifiable_obstacle_detection
{
class VerifiableObstacleDetection
{
public:

	VerifiableObstacleDetection();

	void
	disableConsoleLogging();

	bool
	initializeForApollo();

	const std::string
	processOneFrameForApollo(const std::string& frame_name,
			const std::vector<Polygon>& detections_mission,
			const std::vector<Polygon>& detections_safety);

private:

	Distance distance_;
	Polygon ego_;
	Eigen::Vector3d ego_extent_;
};
} // namespace verifiable_obstacle_detection

#endif /* API_API_H_ */
