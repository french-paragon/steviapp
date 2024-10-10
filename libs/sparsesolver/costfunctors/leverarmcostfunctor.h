#ifndef STEREOVISIONAPP_XYZPRIORCOSTFUNCTOR_H
#define STEREOVISIONAPP_XYZPRIORCOSTFUNCTOR_H

#include <eigen3/Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>

#ifndef NDEBUG
#include <iostream>
#include <ceres/jet.h>
#endif

namespace StereoVisionApp {

/*!
 * \brief The LeverArmCostFunctor class represent a lever arm between two rigid bodies (e.g. cameras) .
 */
class LeverArmCostFunctor
{
public:
    /*!
     * \brief LeverArmCostFunctor build a functor with a prior pose (rigid body2 to rigid body1)
     * \param R the rotation part of the transform
     * \param t the translation part of the transform
     */
    LeverArmCostFunctor(Eigen::Matrix3d const& R, Eigen::Vector3d const& t);

    template <typename T>
    bool operator()(const T* const r1, const T* const t1,
                    const T* const r2, const T* const t2,
                    T* residual) const {

        using MatType = Eigen::Matrix<T,3,3>;
        using VecType = Eigen::Matrix<T,3,1>;

        using PoseType = StereoVision::Geometry::AffineTransform<T>;

        VecType vr1;
        vr1 << r1[0], r1[1], r1[2];

        MatType MR1 = StereoVision::Geometry::rodriguezFormula(vr1);

        VecType vt1;
        vt1 << t1[0], t1[1], t1[2];

        VecType vr2;
        vr2 << r2[0], r2[1], r2[2];

        MatType MR2 = StereoVision::Geometry::rodriguezFormula(vr2);

        VecType vt2;
        vt1 << t2[0], t2[1], t2[2];

        PoseType Pose1toWorld(MR1, vt1);
        PoseType WorldtoPose2(MR2.transpose(), -MR2.transpose()*vt2);

        PoseType pose1topose2 = WorldtoPose2*Pose1toWorld;

        PoseType pose2topose1Prior(_R.cast<T>(), _t.cast<T>());

        PoseType closure = pose1topose2*pose2topose1Prior;

        VecType Rclosure = StereoVision::Geometry::inverseRodriguezFormula(closure.R);
        VecType& tclosure = closure.t;

        residual[0] = Rclosure[0];
        residual[1] = Rclosure[1];
        residual[2] = Rclosure[2];

        residual[3] = tclosure[0];
        residual[4] = tclosure[1];
        residual[5] = tclosure[2];

#ifndef NDEBUG
        for (int i = 0; i < 6; i++) {
            if (!ceres::IsFinite(residual[i])) {
                std::cout << "Error in LeverArmCostFunctor cost computation" << std::endl;
            }
        }
#endif

        return true;
    }

protected:

    Eigen::Matrix3d _R;
    Eigen::Vector3d _t;
};


/*!
 * \brief The LeverArmCostFunctor class represent a parametrized lever arm between two rigid bodies (e.g. cameras) .
 */
class ParametrizedLeverArmCostFunctor
{
public:
    /*!
     * \brief ParametrizedLeverArmCostFunctor build a functor with a parametrized pose (rigid body2 to rigid body1)
     */
    ParametrizedLeverArmCostFunctor();

    template <typename T>
    bool operator()(const T* const rp, const T* const tp,
                    const T* const r1, const T* const t1,
                    const T* const r2, const T* const t2,
                    T* residual) const {

        using MatType = Eigen::Matrix<T,3,3>;
        using VecType = Eigen::Matrix<T,3,1>;

        using PoseType = StereoVision::Geometry::AffineTransform<T>;

        VecType vrp;
        vrp << rp[0], rp[1], rp[2];

        MatType MRp = StereoVision::Geometry::rodriguezFormula(vrp);

        VecType vtp;
        vtp << tp[0], tp[1], tp[2];

        VecType vr1;
        vr1 << r1[0], r1[1], r1[2];

        MatType MR1 = StereoVision::Geometry::rodriguezFormula(vr1);

        VecType vt1;
        vt1 << t1[0], t1[1], t1[2];

        VecType vr2;
        vr2 << r2[0], r2[1], r2[2];

        MatType MR2 = StereoVision::Geometry::rodriguezFormula(vr2);

        VecType vt2;
        vt1 << t2[0], t2[1], t2[2];

        PoseType Pose1toWorld(MR1, vt1);
        PoseType WorldtoPose2(MR2.transpose(), -MR2.transpose()*vt2);

        PoseType pose1topose2 = WorldtoPose2*Pose1toWorld;

        PoseType pose2topose1Prior(MRp, vtp);

        PoseType closure = pose1topose2*pose2topose1Prior;

        VecType Rclosure = StereoVision::Geometry::inverseRodriguezFormula(closure.R);
        VecType& tclosure = closure.t;

        residual[0] = Rclosure[0];
        residual[1] = Rclosure[1];
        residual[2] = Rclosure[2];

        residual[3] = tclosure[0];
        residual[4] = tclosure[1];
        residual[5] = tclosure[2];

#ifndef NDEBUG
        for (int i = 0; i < 6; i++) {
            if (!ceres::IsFinite(residual[i])) {
                std::cout << "Error in ParametrizedLeverArmCostFunctor cost computation" << std::endl;
            }
        }
#endif

        return true;
    }
};

class InterpolatedLeverArmCostFunctor
{
public:
    /*!
     * \brief InterpolatedLeverArmCostFunctor build a functor with a prior lever arm (w1*rigid body1 + w2*rigid body2 to rigid body ref)
     * \param R the rotation part of the transform
     * \param t the translation part of the transform
     */
    InterpolatedLeverArmCostFunctor(Eigen::Matrix3d const& R, Eigen::Vector3d const& t, double w1, double w2);

    template <typename T>
    bool operator()(const T* const r1, const T* const t1,
                    const T* const r2, const T* const t2,
                    const T* const r, const T* const t,
                    T* residual) const {

        using MatType = Eigen::Matrix<T,3,3>;
        using VecType = Eigen::Matrix<T,3,1>;

        using PoseType = StereoVision::Geometry::AffineTransform<T>;

        VecType vr1;
        vr1 << r1[0], r1[1], r1[2];

        VecType vt1;
        vt1 << t1[0], t1[1], t1[2];

        VecType vr2;
        vr2 << r2[0], r2[1], r2[2];

        VecType vt2;
        vt1 << t2[0], t2[1], t2[2];

        VecType vti = T(_w1)*vt1 + T(_w2)*vt2;

        MatType pose_R1 = StereoVision::Geometry::rodriguezFormula<T>(vr1);
        MatType pose_R2 = StereoVision::Geometry::rodriguezFormula<T>(vr2);

        MatType pose_RDelta = pose_R1.transpose()*pose_R2;

        VecType pose_RDeltaLog = StereoVision::Geometry::inverseRodriguezFormula<T>(pose_RDelta);
        pose_RDeltaLog *= T(_w2);

        MatType pose_RDeltaInterp = StereoVision::Geometry::rodriguezFormula<T>(pose_RDeltaLog);

        MatType MRi = pose_R1*pose_RDeltaInterp;

        VecType vr;
        vr2 << r2[0], r2[1], r2[2];

        MatType MR = StereoVision::Geometry::rodriguezFormula(vr);

        VecType vt;
        vt1 << t2[0], t2[1], t2[2];

        PoseType PoseReftoWorld(MR, vt);
        PoseType WorldtoPoseInterp(MRi.transpose(), -MRi.transpose()*vti);

        PoseType pose2pose = WorldtoPoseInterp*PoseReftoWorld;

        PoseType prior(_R.cast<T>(), _t.cast<T>());

        PoseType closure = pose2pose*prior;

        VecType Rclosure = StereoVision::Geometry::inverseRodriguezFormula(closure.R);
        VecType& tclosure = closure.t;

        residual[0] = Rclosure[0];
        residual[1] = Rclosure[1];
        residual[2] = Rclosure[2];

        residual[3] = tclosure[0];
        residual[4] = tclosure[1];
        residual[5] = tclosure[2];

#ifndef NDEBUG
        for (int i = 0; i < 6; i++) {
            if (!ceres::IsFinite(residual[i])) {
                std::cout << "Error in InterpolatedLeverArmCostFunctor cost computation" << std::endl;
            }
        }
#endif

        return true;
    }

protected:

    Eigen::Matrix3d _R;
    Eigen::Vector3d _t;
    double _w1, _w2;
};

class ParametrizedInterpolatedLeverArmCostFunctor
{
public:
    /*!
     * \brief ParametrizedInterpolatedLeverArmCostFunctor build a functor with a parametrized lever arm (w1*rigid body1 + w2*rigid body2 to rigid body ref)
     * \param R the rotation part of the transform
     * \param t the translation part of the transform
     */
    ParametrizedInterpolatedLeverArmCostFunctor(double w1, double w2);

    template <typename T>
    bool operator()(const T* const rp, const T* const tp,
                    const T* const r1, const T* const t1,
                    const T* const r2, const T* const t2,
                    const T* const r, const T* const t,
                    T* residual) const {

        using MatType = Eigen::Matrix<T,3,3>;
        using VecType = Eigen::Matrix<T,3,1>;

        using PoseType = StereoVision::Geometry::AffineTransform<T>;

        VecType vrp;
        vrp << rp[0], rp[1], rp[2];

        MatType MRp = StereoVision::Geometry::rodriguezFormula(vrp);

        VecType vtp;
        vtp << tp[0], tp[1], tp[2];

        VecType vr1;
        vr1 << r1[0], r1[1], r1[2];

        VecType vt1;
        vt1 << t1[0], t1[1], t1[2];

        VecType vr2;
        vr2 << r2[0], r2[1], r2[2];

        VecType vt2;
        vt1 << t2[0], t2[1], t2[2];

        VecType vti = T(_w1)*vt1 + T(_w2)*vt2;

        MatType pose_R1 = StereoVision::Geometry::rodriguezFormula<T>(vr1);
        MatType pose_R2 = StereoVision::Geometry::rodriguezFormula<T>(vr2);

        MatType pose_RDelta = pose_R1.transpose()*pose_R2;

        VecType pose_RDeltaLog = StereoVision::Geometry::inverseRodriguezFormula<T>(pose_RDelta);
        pose_RDeltaLog *= T(_w2);

        MatType pose_RDeltaInterp = StereoVision::Geometry::rodriguezFormula<T>(pose_RDeltaLog);

        MatType MRi = pose_R1*pose_RDeltaInterp;

        VecType vr;
        vr2 << r2[0], r2[1], r2[2];

        MatType MR = StereoVision::Geometry::rodriguezFormula(vr);

        VecType vt;
        vt1 << t2[0], t2[1], t2[2];

        PoseType PoseReftoWorld(MR, vt);
        PoseType WorldtoPoseInterp(MRi.transpose(), -MRi.transpose()*vti);

        PoseType pose2pose = WorldtoPoseInterp*PoseReftoWorld;

        PoseType prior(MRp, vtp);

        PoseType closure = pose2pose*prior;

        VecType Rclosure = StereoVision::Geometry::inverseRodriguezFormula(closure.R);
        VecType& tclosure = closure.t;

        residual[0] = Rclosure[0];
        residual[1] = Rclosure[1];
        residual[2] = Rclosure[2];

        residual[3] = tclosure[0];
        residual[4] = tclosure[1];
        residual[5] = tclosure[2];

#ifndef NDEBUG
        for (int i = 0; i < 6; i++) {
            if (!ceres::IsFinite(residual[i])) {
                std::cout << "Error in ParametrizedInterpolatedLeverArmCostFunctor cost computation" << std::endl;
            }
        }
#endif

        return true;
    }

protected:

    double _w1, _w2;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_XYZPRIORCOSTFUNCTOR_H
