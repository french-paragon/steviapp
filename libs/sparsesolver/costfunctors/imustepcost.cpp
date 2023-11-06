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

} // namespace StereoVisionApp
