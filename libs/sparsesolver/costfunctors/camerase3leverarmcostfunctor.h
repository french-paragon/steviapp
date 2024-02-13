#ifndef STEREOVISIONAPP_XYZPRIORCOSTFUNCTOR_H
#define STEREOVISIONAPP_XYZPRIORCOSTFUNCTOR_H

#include <eigen3/Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>

namespace StereoVisionApp {

/*!
 * \brief The CameraSE3LevelArmCostFunctor class represent a lever arm between two cameras.
 */
class CameraSE3LevelArmCostFunctor
{
public:
    /*!
     * \brief CameraSE3LevelArmCostFunctor build a functor with a prior pose (cam2 to cam1)
     * \param R the rotation part of the transform
     * \param t the translation part of the transform
     */
    CameraSE3LevelArmCostFunctor(Eigen::Matrix3d const& R, Eigen::Vector3d const& t);

    template <typename T>
    bool operator()(const T* const r1, const T* const t1, const T* const r2, const T* const t2, T* residual) const {

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

        PoseType Cam1toWorld(MR1, vt1);
        PoseType WorldtoCam2(MR2.transpose(), -MR2.transpose()*vt2);

        PoseType cam1tocam2 = WorldtoCam2*Cam1toWorld;

        PoseType cam2tocam1Prior(_R.cast<T>(), _t.cast<T>());

        PoseType closure = cam1tocam2*cam2tocam1Prior;

        VecType Rclosure = StereoVision::Geometry::inverseRodriguezFormula(closure.R);
        VecType& tclosure = closure.t;

        residual[0] = Rclosure[0];
        residual[1] = Rclosure[1];
        residual[2] = Rclosure[2];

        residual[3] = tclosure[0];
        residual[4] = tclosure[1];
        residual[5] = tclosure[2];

        return true;
    }

protected:

    Eigen::Matrix3d _R;
    Eigen::Vector3d _t;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_XYZPRIORCOSTFUNCTOR_H
