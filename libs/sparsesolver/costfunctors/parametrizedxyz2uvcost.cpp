#include "parametrizedxyz2uvcost.h"

namespace StereoVisionApp {

ParametrizedXYZ2UVCost::ParametrizedXYZ2UVCost(const Eigen::Vector2d &uv, const Eigen::Matrix2d &info) :
    _uv(uv),
    _info(info)
{

}

} // namespace StereoVisionApp
