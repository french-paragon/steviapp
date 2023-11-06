#ifndef STEREOVISIONAPP_CAMPOSEWITHPARAMETRIZEDLEVERARMCOST_H
#define STEREOVISIONAPP_CAMPOSEWITHPARAMETRIZEDLEVERARMCOST_H


#include <eigen3/Eigen/Core>

#include <LibStevi/geometry/core.h>
#include <LibStevi/geometry/rotations.h>


namespace StereoVisionApp {

/*!
 * \brief The CamPosWithParametrizedLeverArmCost class represent a prior on the camera position (e.g. given by GPS), along with a parametrized lever arm.
 */
class CamPosWithParametrizedLeverArmCost
{
public:

    /*!
     * \brief CamPosWithParametrizedLeverArmCost constructor
     * \param t the position of the sensor in the world frame (or translation part of  body2world transform)
     */
    CamPosWithParametrizedLeverArmCost(Eigen::Vector3d const& t);

    template <typename T>
    bool operator()(const T* const t_la, const T* const r_cam, const T* const t_cam, T* residual) const { //assuming t_la is the translation from B2C

        using MatType = Eigen::Matrix<T,3,3>;
        using VecType = Eigen::Matrix<T,3,1>;

        using PoseType = StereoVision::Geometry::AffineTransform<T>;

        //lever arm

        VecType vt_la;
        vt_la << t_la[0], t_la[1], t_la[2];

        //pose
        VecType vr_cam;
        vr_cam << t_cam[0], t_cam[1], t_cam[2];

        MatType MR_cam = StereoVision::Geometry::rodriguezFormula(vr_cam);

        VecType vt_cam;
        vt_cam << t_cam[0], t_cam[1], t_cam[2];

        PoseType CamtoWorld(MR_cam, vt_cam);
        PoseType WorldtoCam(MR_cam.transpose(), -MR_cam.transpose()*vt_cam);

        VecType posInWorld = WorldtoCam*vt_la;

        residual[0] = posInWorld[0] - _t[0];
        residual[1] = posInWorld[0] - _t[1];
        residual[2] = posInWorld[0] - _t[2];

        return true;
    }

protected:

    Eigen::Vector3d _t;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CAMPOSEWITHPARAMETRIZEDLEVERARMCOST_H
