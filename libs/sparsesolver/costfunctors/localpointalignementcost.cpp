#include "localpointalignementcost.h"

namespace StereoVisionApp {

LocalPoint2TargetAlignementCost::LocalPoint2TargetAlignementCost(
        Eigen::Vector3d const& localPos,
        Eigen::Vector3d const& mappingPos,
        Eigen::Matrix3d const& info) :
    _localPos(localPos),
    _mappingPos(mappingPos),
    _info(info)
{

}

LocalPointAlignementCost::LocalPointAlignementCost(
        Eigen::Vector3d const& localPos,
        Eigen::Matrix3d const& info) :
    _localPos(localPos),
    _info(info)
{

}

LocalRelativePointAlignementCost::LocalRelativePointAlignementCost(
        Eigen::Vector3d const& localPos,
        Eigen::Matrix3d const& info) :
    _localPos(localPos),
    _info(info)
{

}

LocalInterpRelativePointAlignementCost::LocalInterpRelativePointAlignementCost(
        Eigen::Vector3d const& localPos,
        double w1,
        double w2,
        Eigen::Matrix3d const& info) :
    _localPos(localPos),
    _w1(w1),
    _w2(w2),
    _info(info)
{

}

} // namespace StereoVisionApp
