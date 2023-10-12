#include "camerase3leverarmcostfunctor.h"

namespace StereoVisionApp {

CameraSE3LevelArmCostFunctor::CameraSE3LevelArmCostFunctor(Eigen::Matrix3d const& R, Eigen::Vector3d const& t):
    _R(R),
    _t(t)
{

}

} // namespace StereoVisionApp
