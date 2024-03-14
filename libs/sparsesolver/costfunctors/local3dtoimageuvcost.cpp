#include "local3dtoimageuvcost.h"

namespace StereoVisionApp {

Local3DtoImageUVCost::Local3DtoImageUVCost(Eigen::Vector3d const& localPos, Eigen::Vector2d const& uv, Eigen::Matrix2d const& info) :
    _localPos(localPos),
    _uv(uv),
    _info(info)
{

}

} // namespace StereoVisionApp
