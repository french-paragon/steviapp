#ifndef STEREOVISIONAPP_PINHOLECAMERAPROJECTORMODULE_H
#define STEREOVISIONAPP_PINHOLECAMERAPROJECTORMODULE_H

#include "../modularsbasolver.h"

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>
#include <StereoVision/geometry/alignement.h>

namespace StereoVisionApp {

class Project;
class PushBroomPinholeCamera;

/*!
 * \brief The PinholeUVProjector class is a functor representing a basic lens distortion model based on the brown model
 */
class PinholeUVProjector
{
public:
    PinholeUVProjector(double sensorHeight, double sensorWidth) :
        _sensorHeight(sensorHeight),
        _sensorWidth(sensorWidth)
    {

    }

    template<typename T>
    Eigen::Matrix<T,3,1> dirFromUV(T const* uv,
                                     T const* const* params) {

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        const T* f = params[0];
        const T* pp = params[1];
        const T* ks = params[2];
        const T* ts = params[3];
        const T* Bs = params[4];

        T cam_f = *f;

        V2T cam_pp;
        cam_pp << pp[0], pp[1];

        V2T B_dist;
        B_dist << ts[0], ts[1];

        V2T uvPos;
        uvPos << uv[0], uv[1];

        V2T normalized = StereoVision::Geometry::inverseSkewDistortion<T>(uvPos, B_dist, cam_f, cam_pp);

        V3T k_dist;
        k_dist << ks[0], ks[1], ks[2];

        V2T t_dist;
        t_dist << ts[0], ts[1];

        V2T dRadial = StereoVision::Geometry::radialDistortion<T>(normalized, k_dist);
        V2T dTangential = StereoVision::Geometry::tangentialDistortion<T>(normalized, t_dist);

        V2T corrected = normalized + dRadial + dTangential;

        V3T ret;
        ret << corrected[0], corrected[1], T(1);

        ret *= *f;

        return ret;
    }

protected:

    double _sensorHeight;
    double _sensorWidth;
};

class PinholePushbroomUVProjector
{
public:
    PinholePushbroomUVProjector(double sensorWidth) :
        _sensorWidth(sensorWidth)
    {

    }

    template<typename T>
    Eigen::Matrix<T,3,1> dirFromUV(T const* uv,
                                   T const* const* params) {

        const T* f = params[0];
        const T* pp = params[1];
        const T* as = params[2];
        const T* bs = params[3];

        T s = (uv[0] - T(_sensorWidth)/T(2)) /T(_sensorWidth); //set 0 at the center (usefull to decorrelate the coefficients 0 and 1.
        T s2 = s*s;
        T s3 = s2*s;
        T s4 = s3*s;
        T s5 = s4*s;

        T du = as[0] + as[1]*s + as[2]*s2 + as[3]*s3 + as[4]*s4 + as[5]*s5; //compute corrections backwards for numerical stability
        T dv = bs[0] + bs[1]*s + bs[2]*s2 + bs[3]*s3 + bs[4]*s4 + bs[5]*s5;


        Eigen::Matrix<T,3,1> ret;
        ret << uv[0] - pp[0] + du, uv[1] + dv, f[0];

        return ret;
    }

protected:

    double _sensorWidth;
};

class PinholeCamProjModule : public ModularSBASolver::ProjectorModule
{

public:

    PinholeCamProjModule(Camera* associatedCamera);
    ~PinholeCamProjModule();

    virtual QString moduleName() const override;

    virtual bool addProjectionCostFunction(double* pointData,
                                           double* poseOrientation,
                                           double* posePosition,
                                           Eigen::Vector2d const& ptProjPos,
                                           Eigen::Matrix2d const& ptProjStiffness,
                                           StereoVision::Geometry::RigidBodyTransform<double> const& offset = StereoVision::Geometry::RigidBodyTransform<double>(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()),
                                           double* leverArmOrientation = nullptr,
                                           double* leverArmPosition = nullptr,
                                           QString const& logLabel = "") override;

    virtual ProjectionInfos getProjectionInfos() override;

    virtual bool init() override;
    virtual bool writeResults() override;
    virtual bool writeUncertainty() override;
    virtual void cleanup() override;

protected:

    Camera* _associatedCamera;

    double _fLen; //f
    std::array<double, 2> _principalPoint; //pp
    std::array<double, 3> _radialDistortion; //k1, k2, k3
    std::array<double, 2> _tangentialDistortion; //t1, t2
    std::array<double, 2> _skewDistortion; //B1, B2

};

class PinholePushBroomCamProjectorModule : public ModularSBASolver::ProjectorModule
{

public:

    PinholePushBroomCamProjectorModule(PushBroomPinholeCamera* associatedCamera);
    ~PinholePushBroomCamProjectorModule();

    virtual QString moduleName() const override;

    virtual bool addProjectionCostFunction(double* pointData,
                                           double* poseOrientation,
                                           double* posePosition,
                                           Eigen::Vector2d const& ptProjPos,
                                           Eigen::Matrix2d const& ptProjStiffness,
                                           StereoVision::Geometry::RigidBodyTransform<double> const& offset = StereoVision::Geometry::RigidBodyTransform<double>(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()),
                                           double* leverArmOrientation = nullptr,
                                           double* leverArmPosition = nullptr,
                                           QString const& logLabel = "") override;

    virtual ProjectionInfos getProjectionInfos() override;

    virtual bool init() override;
    virtual bool writeResults() override;
    virtual std::vector<std::pair<const double*, const double*>> requestUncertainty() override;
    virtual bool writeUncertainty() override;
    virtual void cleanup() override;

protected:

    PushBroomPinholeCamera* _associatedCamera;

    double _fLen; //f
    double _principalPoint; //pp
    std::array<double, 6> _horizontalDistortion; //as
    std::array<double, 6> _verticalDistortion; //bs

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_PINHOLECAMERAPROJECTORMODULE_H
