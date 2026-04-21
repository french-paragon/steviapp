#include "imustepcost.h"

namespace StereoVisionApp {

GyroStepCostBase::GyroStepCostBase(Eigen::Matrix3d const& attitudeDelta,
                           double delta_t) :
    _attitudeDelta(attitudeDelta),
    _delta_t(delta_t)
{
    _FrameRotation = Eigen::Matrix3d::Identity();
}

GyroStepCostBase::GyroStepCostBase(Eigen::Matrix3d const& attitudeDelta,
                 Eigen::Vector3d const& FrameRotationRate,
                 double delta_t) :
    _attitudeDelta(attitudeDelta),
    _delta_t(delta_t)
{
    _FrameRotation = StereoVision::Geometry::rodriguezFormula<double>(FrameRotationRate*delta_t);
}



AccelerometerStepCostBase::AccelerometerStepCostBase(Eigen::Vector3d const& speedDelta,
                          double delta_t1,
                          double delta_t2) :
    _speedDelta(speedDelta),
    _delta_t1(delta_t1),
    _delta_t2(delta_t2)
{

}

} // namespace StereoVisionApp
