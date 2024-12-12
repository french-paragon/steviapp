#ifndef LOCALPOINTPROJECTIONCOST_H
#define LOCALPOINTPROJECTIONCOST_H

#include <Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>

#ifndef NDEBUG
#include <iostream>
#include <ceres/jet.h>
#endif

namespace StereoVisionApp {

/*!
 * \brief The LocalPoint2TargetAlignementCost class measure the alignement between a fixed point in global coordinate and its coordinate in local system
 */
template <int projSpaceDim>
class LocalPoint2TargetProjectionCost {

    static_assert (projSpaceDim >= 1, "Projection space should have dimension >= 1");

public:

    using ProjVector = Eigen::Matrix<double,projSpaceDim,1>;
    using ProjMat = Eigen::Matrix<double,projSpaceDim,3>;
    using InfosMat = Eigen::Matrix<double,projSpaceDim,projSpaceDim>;

    LocalPoint2TargetProjectionCost(Eigen::Vector3d const& localPos,
                                        ProjVector const& projPos,
                                        ProjMat const& projMat,
                                        InfosMat const& info) :
        _localPos(localPos),
        _projPos(projPos),
        _projMat(projMat),
        _info(info)
    {

    }

    template <typename T>
    bool operator()(const T* const r,
                    const T* const t,
                    T* residual) const {

        using MatType = Eigen::Matrix<T,3,3>;
        using VecType = Eigen::Matrix<T,3,1>;
        using ProjVecType = Eigen::Matrix<T,projSpaceDim,1>;

        using PoseType = StereoVision::Geometry::RigidBodyTransform<T>;

        PoseType local2mapping;

        local2mapping.r << r[0], r[1], r[2];
        local2mapping.t << t[0], t[1], t[2];

        ProjVecType closure = _info.template cast<T>()*(_projPos.template cast<T>() - _projMat.template cast<T>()*(local2mapping*_localPos.cast<T>()));

        for (int i = 0; i < projSpaceDim; i++) {
            residual[i] = closure[i];
        }

        #ifndef NDEBUG
        for (int i = 0; i < projSpaceDim; i++) {
            if (!ceres::IsFinite(residual[i])) {
                std::cout << "Error in LocalPoint2TargetProjectionCost cost computation" << std::endl;
            }
        }
        #endif

        return true;
    }

protected:

    Eigen::Vector3d _localPos;
    ProjVector _projPos;
    ProjMat _projMat;
    InfosMat _info;
};
}

#endif // LOCALPOINTPROJECTIONCOST_H
