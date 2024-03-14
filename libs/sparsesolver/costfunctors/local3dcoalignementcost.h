#ifndef STEREOVISIONAPP_LOCAL3DCOALIGNEMENTCOST_H
#define STEREOVISIONAPP_LOCAL3DCOALIGNEMENTCOST_H

#include <Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>

namespace StereoVisionApp {

/*!
 * \brief The Local3DCoalignementCost class measure the alignement between the local positions of a point in two systems
 */
class Local3DCoalignementCost
{
public:
    Local3DCoalignementCost(Eigen::Vector3d const& localPos1, Eigen::Vector3d const& localPos2, Eigen::Matrix3d const& info);

    template <typename T>
    bool operator()(const T* const r1, const T* const t1, const T* const r2, const T* const t2, T* residual) const {

        using MatType = Eigen::Matrix<T,3,3>;
        using VecType = Eigen::Matrix<T,3,1>;

        using PoseType = StereoVision::Geometry::AffineTransform<T>;

        VecType vr1;
        vr1 << r1[0], r1[1], r1[2];

        PoseType Local1toWorld;

        Local1toWorld.R = StereoVision::Geometry::rodriguezFormula<T>(vr1);

        Local1toWorld.t << t1[0], t1[1], t1[2];

        VecType vr2;
        vr2 << r2[0], r2[1], r2[2];

        PoseType World2Local2;

        World2Local2.R = StereoVision::Geometry::rodriguezFormula<T>(-vr2);

        VecType vt2;
        vt2 << t2[0], t2[1], t2[2];

        World2Local2.t << -World2Local2.R*vt2;

        VecType closure = _info.cast<T>()*(_localPos2.cast<T>() - World2Local2*Local1toWorld*_localPos1.cast<T>());

        residual[0] = closure[0];
        residual[1] = closure[1];
        residual[2] = closure[2];

        return true;
    }

protected:

    Eigen::Vector3d _localPos1;
    Eigen::Vector3d _localPos2;
    Eigen::Matrix3d _info;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LOCAL3DCOALIGNEMENTCOST_H
