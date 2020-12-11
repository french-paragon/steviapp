#ifndef POINTCLOUDALIGNMENT_H
#define POINTCLOUDALIGNMENT_H

#include "geometry/rotations.h"

#include <vector>


namespace  StereoVisionApp {


AffineTransform estimateQuasiShapePreservingMap(Eigen::VectorXf const& obs,
												Eigen::Matrix3Xf const& pts,
												std::vector<int> const& idxs,
												std::vector<Axis> const& coordinate,
												float regularizationWeight = 10);

ShapePreservingTransform affine2ShapePreservingMap(AffineTransform const & initial);

AffineTransform estimateShapePreservingMap(Eigen::VectorXf const& obs,
										   Eigen::Matrix3Xf const& pts,
										   std::vector<int> const& idxs,
										   std::vector<Axis> const& coordinate,
										   IterativeTermination * status,
										   int n_steps = 50,
										   float incrLimit = 1e-8,
										   float damping = 5e-1,
										   float dampingScale = 1e-1,
										   float initializerRegularizationWeight = 10);

}; //namespace StereoVisionApp

#endif // POINTCLOUDALIGNMENT_H
