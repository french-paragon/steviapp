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
 * \brief The LocalPoint2TargetAlignementCost class measure the alignement between a fixed point in global coordinate and its coordinate in local system
 */
class LocalPoint2TargetAlignementCost
{
public:
    LocalPoint2TargetAlignementCost(Eigen::Vector3d const& localPos,
                                    Eigen::Vector3d const& mappingPos,
                                    Eigen::Matrix3d const& info);

    template <typename T>
    bool operator()(const T* const r,
                    const T* const t,
                    T* residual) const {

        using MatType = Eigen::Matrix<T,3,3>;
        using VecType = Eigen::Matrix<T,3,1>;

        using PoseType = StereoVision::Geometry::RigidBodyTransform<T>;

        PoseType local2mapping;
        local2mapping.r << r[0], r[1], r[2];
        local2mapping.t << t[0], t[1], t[2];

        VecType closure = _info.cast<T>()*(_mappingPos.cast<T>() - local2mapping*_localPos.cast<T>());

        residual[0] = closure[0];
        residual[1] = closure[1];
        residual[2] = closure[2];

#ifndef NDEBUG
        if (!ceres::isfinite(residual[0]) or !ceres::isfinite(residual[1]) or !ceres::isfinite(residual[2])) {
            std::cout << "Error in LocalPointAlignementCost cost computation" << std::endl;
        }
#endif

        return true;
    }

protected:

    Eigen::Vector3d _localPos;
    Eigen::Vector3d _mappingPos;
    Eigen::Matrix3d _info;
};

/*!
 * \brief The LocalPointAlignementCost class measure the alignement between a point in global coordinate and its coordinate in local system
 */
class LocalPointAlignementCost
{
public:
    LocalPointAlignementCost(Eigen::Vector3d const& localPos,
                             Eigen::Matrix3d const& info);

    template <typename T>
    bool operator()(const T* const p,
                    const T* const r,
                    const T* const t,
                    T* residual) const {

        using MatType = Eigen::Matrix<T,3,3>;
        using VecType = Eigen::Matrix<T,3,1>;

        using PoseType = StereoVision::Geometry::RigidBodyTransform<T>;

        PoseType local2mapping;
        local2mapping.r << r[0], r[1], r[2];
        local2mapping.t << t[0], t[1], t[2];

        VecType vp;
        vp << p[0], p[1], p[2];

        VecType closure = _info.cast<T>()*(vp - local2mapping*_localPos.cast<T>());

        residual[0] = closure[0];
        residual[1] = closure[1];
        residual[2] = closure[2];

#ifndef NDEBUG
        if (!ceres::isfinite(residual[0]) or !ceres::isfinite(residual[1]) or !ceres::isfinite(residual[2])) {
            std::cout << "Error in LocalPointAlignementCost cost computation" << std::endl;
        }
#endif

        return true;
    }

protected:

    Eigen::Vector3d _localPos;
    Eigen::Matrix3d _info;
};

/*!
 * \brief The LocalRelativePointAlignementCost class measure the position of a point in a local system, which is itself expressed relative to another local system.
 *
 * This is usefull to express containts with local systems relative to their attached trajectories, as an example.
 */
class LocalRelativePointAlignementCost
{
public:
    LocalRelativePointAlignementCost(Eigen::Vector3d const& localPos,
                                         Eigen::Matrix3d const& info);

    template <typename T>
    bool operator()(const T* const p,
                    const T* const r,
                    const T* const t,
                    const T* const localr,
                    const T* const localt,
                    T* residual) const {

        using MatType = Eigen::Matrix<T,3,3>;
        using VecType = Eigen::Matrix<T,3,1>;

        using PoseType = StereoVision::Geometry::RigidBodyTransform<T>;

        PoseType intermediate2mapping;
        intermediate2mapping.r << r[0], r[1], r[2];
        intermediate2mapping.t << t[0], t[1], t[2];

        PoseType local2intermediate;
        local2intermediate.r << localr[0], localr[1], localr[2];
        local2intermediate.t << localt[0], localt[1], localt[2];

        VecType vp;
        vp << p[0], p[1], p[2];

        VecType closure = _info.cast<T>()*(vp - intermediate2mapping*local2intermediate*_localPos.cast<T>());

        residual[0] = closure[0];
        residual[1] = closure[1];
        residual[2] = closure[2];

#ifndef NDEBUG
        if (!ceres::isfinite(residual[0]) or !ceres::isfinite(residual[1]) or !ceres::isfinite(residual[2])) {
            std::cout << "Error in LocalPointAlignementCost cost computation" << std::endl;
        }
#endif

        return true;
    }

protected:

    Eigen::Vector3d _localPos;
    Eigen::Matrix3d _info;
};

/*!
 * \brief The LocalInterpRelativePointAlignementCost class measure the position of a point in a local system, which is itself expressed relative to another local system interpolated between two poses.
 *
 * This is usefull to express containts with local systems relative to their attached trajectories, as an example.
 */
class LocalInterpRelativePointAlignementCost
{
public:
    LocalInterpRelativePointAlignementCost(Eigen::Vector3d const& localPos,
                                           double w1,
                                           double w2,
                                         Eigen::Matrix3d const& info);

    template <typename T>
    bool operator()(const T* const p,
                    const T* const r1,
                    const T* const t1,
                    const T* const r2,
                    const T* const t2,
                    const T* const localr,
                    const T* const localt,
                    T* residual) const {

        using MatType = Eigen::Matrix<T,3,3>;
        using VecType = Eigen::Matrix<T,3,1>;

        using PoseType = StereoVision::Geometry::RigidBodyTransform<T>;

        PoseType intermediate12mapping;
        intermediate12mapping.r << r1[0], r1[1], r1[2];
        intermediate12mapping.t << t1[0], t1[1], t1[2];

        PoseType intermediate22mapping;
        intermediate22mapping.r << r2[0], r2[1], r2[2];
        intermediate22mapping.t << t2[0], t2[1], t2[2];

        PoseType intermediate2mapping =
                StereoVision::Geometry::interpolateRigidBodyTransformOnManifold(T(_w1), intermediate12mapping, T(_w2), intermediate22mapping);

        PoseType local2intermediate;
        local2intermediate.r << localr[0], localr[1], localr[2];
        local2intermediate.t << localt[0], localt[1], localt[2];

        VecType vp;
        vp << p[0], p[1], p[2];

        VecType closure = _info.cast<T>()*(vp - intermediate2mapping*local2intermediate*_localPos.cast<T>());

        residual[0] = closure[0];
        residual[1] = closure[1];
        residual[2] = closure[2];

#ifndef NDEBUG
        if (!ceres::isfinite(residual[0]) or !ceres::isfinite(residual[1]) or !ceres::isfinite(residual[2])) {
            std::cout << "Error in LocalPointAlignementCost cost computation" << std::endl;
        }
#endif

        return true;
    }

protected:

    double _w1;
    double _w2;

    Eigen::Vector3d _localPos;
    Eigen::Matrix3d _info;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LOCALPOINTALIGNEMENTCOST_H
