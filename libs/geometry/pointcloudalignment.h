#ifndef POINTCLOUDALIGNMENT_H
#define POINTCLOUDALIGNMENT_H

#include "geometry/core.h"

#include <vector>


namespace  StereoVisionApp {


AffineTransform estimateShapePreservingMap(Eigen::VectorXf const& obs,
										   Eigen::Matrix3Xf const& pts,
										   std::vector<int> const& idxs,
										   std::vector<Axis> const& coordinate,
										   IterativeTermination * status,
										   int n_steps = 250,
										   float incrLimit = 1e-8,
										   float damping = 5e-1,
										   float dampingScale = 1e-1,
										   float initial_s = 0.,
										   Eigen::Vector3f const& initial_r = Eigen::Vector3f(0,0,0),
										   Eigen::Vector3f const& initial_t = Eigen::Vector3f(0,0,0));

}; //namespace StereoVisionApp

#endif // POINTCLOUDALIGNMENT_H
