#ifndef STEREOVISIONAPP_LOCALPOINTALIGNEMENTCOST_H
#define STEREOVISIONAPP_LOCALPOINTALIGNEMENTCOST_H

#include <Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>

#ifndef NDEBUG
#include <iostream>
#include <ceres/jet.h>
#endif

namespace StereoVisionApp {

/*!
 * \brief The LocalPointAlignementCost class measure the alignement between a point in global coordinate and its coordinate in local system
 */
class LocalPointAlignementCost
{
public:
    LocalPointAlignementCost(Eigen::Vector3d const& localPos, Eigen::Matrix3d const& info);

    template <typename T>
    bool operator()(const T* const p, const T* const r, const T* const t, T* residual) const {

        using MatType = Eigen::Matrix<T,3,3>;
        using VecType = Eigen::Matrix<T,3,1>;

        using PoseType = StereoVision::Geometry::AffineTransform<T>;

        VecType vr;
        vr << r[0], r[1], r[2];

        PoseType LocaltoWorld;

        LocaltoWorld.R = StereoVision::Geometry::rodriguezFormula(vr);

        LocaltoWorld.t << t[0], t[1], t[2];

        VecType vp;
        vp << p[0], p[1], p[2];

        VecType closure = _info.cast<T>()*(vp - LocaltoWorld*_localPos.cast<T>());

        residual[0] = closure[0];
        residual[1] = closure[1];
        residual[2] = closure[2];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1]) or !ceres::IsFinite(residual[2])) {
            std::cout << "Error in LocalPointAlignementCost cost computation" << std::endl;
        }
#endif

        return true;
    }

protected:

    Eigen::Vector3d _localPos;
    Eigen::Matrix3d _info;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LOCALPOINTALIGNEMENTCOST_H
