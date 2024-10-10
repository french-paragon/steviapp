#include "imustepcost.h"

namespace StereoVisionApp {

ImuStepCost::ImuStepCost(const Eigen::Vector3d &orientationDelta,
                         const Eigen::Vector3d &posSpeedDelta,
                         const Eigen::Vector3d &speedDelta,
                         double delta_t) :
    _orientationDelta(StereoVision::Geometry::rodriguezFormula(orientationDelta)),
    _posSpeedDelta(posSpeedDelta),
    _speedDelta(speedDelta),
    _delta_t(delta_t)
{

}

GyroStepCost::GyroStepCost(Eigen::Matrix3d const& attitudeDelta,
                           double delta_t) :
    _attitudeDelta(attitudeDelta),
    _delta_t(delta_t)
{

}



AccelerometerStepCostBase::AccelerometerStepCostBase(Eigen::Vector3d const& speedDelta,
                          double delta_t1,
                          double delta_t2) :
    _speedDelta(speedDelta),
    _delta_t1(delta_t1),
    _delta_t2(delta_t2),
    _delta_tv((delta_t2 + delta_t1)/2)
{

}

} // namespace StereoVisionApp
