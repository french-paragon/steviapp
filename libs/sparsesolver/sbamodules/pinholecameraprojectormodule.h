#ifndef STEREOVISIONAPP_PINHOLECAMERAPROJECTORMODULE_H
#define STEREOVISIONAPP_PINHOLECAMERAPROJECTORMODULE_H

#include "../modularsbasolver.h"

namespace StereoVisionApp {

class Project;
class PushBroomPinholeCamera;

class PinholePushBroomCamProjectorModule : public ModularSBASolver::ProjectorModule
{

public:

    PinholePushBroomCamProjectorModule(PushBroomPinholeCamera* associatedCamera);
    ~PinholePushBroomCamProjectorModule();

    virtual bool addProjectionCostFunction(double* pointData,
                                           double* poseOrientation,
                                           double* posePosition,
                                           Eigen::Vector2d const& ptProjPos,
                                           Eigen::Matrix2d const& ptProjStiffness,
                                           ceres::Problem & problem,
                                           StereoVision::Geometry::RigidBodyTransform<double> const& offset = StereoVision::Geometry::RigidBodyTransform<double>(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()),
                                           double* leverArmOrientation = nullptr,
                                           double* leverArmPosition = nullptr) override;

    virtual bool addCrossProjectionCostFunction(double* pose1Orientation,
                                                double* pose1Position,
                                                Eigen::Vector2d const& ptProj1Pos,
                                                Eigen::Matrix2d const& ptProj1Stiffness,
                                                double* pose2Orientation,
                                                double* pose2Position,
                                                Eigen::Vector2d const& ptProj2Pos,
                                                Eigen::Matrix2d const& ptProj2Stiffness,
                                                ceres::Problem & problem) override;

    virtual bool init(ModularSBASolver* solver, ceres::Problem & problem) override;
    virtual bool writeResults(ModularSBASolver* solver) override;
    virtual bool writeUncertainty(ModularSBASolver* solver) override;
    virtual void cleanup(ModularSBASolver* solver) override;

protected:

    PushBroomPinholeCamera* _associatedCamera;

    double _fLen; //f
    double _principalPoint; //pp
    std::array<double, 6> _horizontalDistortion; //as
    std::array<double, 6> _verticalDistortion; //bs

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_PINHOLECAMERAPROJECTORMODULE_H
