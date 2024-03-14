#include "local3dcoalignementcost.h"

namespace StereoVisionApp {

Local3DCoalignementCost::Local3DCoalignementCost(Eigen::Vector3d const& localPos1, Eigen::Vector3d const& localPos2, Eigen::Matrix3d const& info) :
    _localPos1(localPos1),
    _localPos2(localPos2),
    _info(info)
{

}

} // namespace StereoVisionApp
