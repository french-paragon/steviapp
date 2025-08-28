#ifndef STEREOVISIONAPP_PINHOLECAMERAPROJECTORMODULE_H
#define STEREOVISIONAPP_PINHOLECAMERAPROJECTORMODULE_H

#include "../modularsbasolver.h"

namespace StereoVisionApp {

class Project;
class PushBroomPinholeCamera;

class PinholePushbroomUVProjector
{
public:
    PinholePushbroomUVProjector(double sensorWidth) :
        _sensorWidth(sensorWidth)
    {

    }

    template<typename T>
    Eigen::Matrix<T,3,1> dirFromUV(T* uv,
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
        ret << uv[1] + dv, uv[0] - pp[0] + du, f[0];

        return ret;
    }

protected:

    double _sensorWidth;
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

    virtual bool addCrossProjectionCostFunction(double* pose1Orientation,
                                                double* pose1Position,
                                                Eigen::Vector2d const& ptProj1Pos,
                                                Eigen::Matrix2d const& ptProj1Stiffness,
                                                double* pose2Orientation,
                                                double* pose2Position,
                                                Eigen::Vector2d const& ptProj2Pos,
                                                Eigen::Matrix2d const& ptProj2Stiffness,
                                                QString const& logLabel = "") override;

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
