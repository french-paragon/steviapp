#include "localpointalignementcost.h"

namespace StereoVisionApp {

LocalPointAlignementCost::LocalPointAlignementCost(Eigen::Vector3d const& localPos, Eigen::Matrix3d const& info) :
    _localPos(localPos),
    _info(info)
{

}

} // namespace StereoVisionApp
