#ifndef STEREOVISIONAPP_LOCAL3DTOIMAGEUVCOST_H
#define STEREOVISIONAPP_LOCAL3DTOIMAGEUVCOST_H

#include <eigen3/Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>
#include <StereoVision/geometry/alignement.h>

#ifndef NDEBUG
#include <iostream>
#include <ceres/jet.h>
#endif


namespace StereoVisionApp {

/*!
 * \brief The Local3DtoImageUVCost class measure the alignement between a 3D point in a local frame and its uv projection in a pinhole camera.
 */
class Local3DtoImageUVCost
{
public:
    Local3DtoImageUVCost(Eigen::Vector3d const& localPos, Eigen::Vector2d const& uv, Eigen::Matrix2d const& info);

    template <typename T>
    bool operator()(const T* const rls,
                    const T* const tls,
                    const T* const r,
                    const T* const t,
                    const T* const f,
                    const T* const pp,
                    const T* const ks,
                    const T* const ts,
                    const T* const Ds,
                    T* residual) const {

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        using PoseType = StereoVision::Geometry::AffineTransform<T>;

        //projection

        V3T ls_r;
        ls_r << rls[0], rls[1], rls[2];

        PoseType LocaltoWorld;

        LocaltoWorld.R = StereoVision::Geometry::rodriguezFormula(ls_r);

        LocaltoWorld.t << tls[0], tls[1], tls[2];

        V3T lm_pos = LocaltoWorld*_localPos.cast<T>();

        V3T pose_r;
        pose_r << r[0], r[1], r[2];

        M3T pose_R = StereoVision::Geometry::rodriguezFormula(pose_r);

        V3T pose_t;
        pose_t << t[0], t[1], t[2];

        V3T Pbar = pose_R.transpose()*(lm_pos - pose_t);
        if (Pbar[2] < 0.0) {
            return false;
        }

        V2T proj = StereoVision::Geometry::projectPoints(Pbar);

        V3T k_dist;
        k_dist << ks[0], ks[1], ks[2];

        V2T t_dist;
        t_dist << ts[0], ts[1];

        V2T dRadial = StereoVision::Geometry::radialDistortion<T>(proj, k_dist);
        V2T dTangential = StereoVision::Geometry::tangentialDistortion<T>(proj, t_dist);

        proj += dRadial + dTangential;

        V2T s_dist;
        s_dist << Ds[0], Ds[1];

        T cam_f = *f;

        V2T cam_pp;
        cam_pp << pp[0], pp[1];

        V2T final = StereoVision::Geometry::skewDistortion<T>(proj, s_dist, cam_f, cam_pp);

        V2T error;
        error << final[0] - _uv[0], final[1] - _uv[1];

        residual[0] = _info(0,0)*error[0] + _info(0,1)*error[1];
        residual[1] = _info(1,0)*error[0] + _info(1,1)*error[1];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1])) {
            std::cout << "Error in Local3DtoImageUVCost cost computation" << std::endl;
        }
#endif

        return true;

    }

protected:

    Eigen::Vector3d _localPos;
    Eigen::Vector2d _uv;
    Eigen::Matrix2d _info;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LOCAL3DTOIMAGEUVCOST_H
