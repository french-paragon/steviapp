#ifndef ROTATIONS_H
#define ROTATIONS_H

#include "geometry/core.h"

namespace StereoVisionApp {

Eigen::Matrix3f rodriguezFormula(Eigen::Vector3f const& r);

Eigen::Matrix3f diffRodriguezLieAlgebra(Eigen::Vector3f const& r);
Eigen::Matrix3f diffRodriguez(Eigen::Vector3f const& r, Axis direction);

} //namespace StereoVisionApp

#endif // ROTATIONS_H
