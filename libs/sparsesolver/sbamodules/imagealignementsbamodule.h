#ifndef STEREOVISIONAPP_IMAGEALIGNEMENTSBAMODULE_H
#define STEREOVISIONAPP_IMAGEALIGNEMENTSBAMODULE_H

#include "../modularsbasolver.h"

namespace StereoVisionApp {

class Project;
class GenericSBAGraphReductor;

class ImageAlignementSBAModule : public ModularSBASolver::SBAModule
{

public:

    static const char* ModuleName;
    inline static void registerDefaultModuleFactory(SBASolverModulesInterface* interface) {
        interface->registerSBAModule(ImageAlignementSBAModule::ModuleName, [] (ModularSBASolver* solver) -> ModularSBASolver::SBAModule* {
            return new ImageAlignementSBAModule();
        });
    }

    ImageAlignementSBAModule();

    virtual QString moduleName() const override;

    virtual bool addGraphReductorVariables(Project *currentProject, GenericSBAGraphReductor* graphReductor) override;
    virtual bool addGraphReductorObservations(Project *currentProject, GenericSBAGraphReductor* graphReductor) override;

    virtual bool setupParameters(ModularSBASolver* solver) override;
    virtual bool init(ModularSBASolver* solver, ceres::Problem & problem) override;
    virtual bool writeResults(ModularSBASolver* solver) override;
    virtual bool writeUncertainty(ModularSBASolver* solver) override;
    virtual void cleanup(ModularSBASolver* solver) override;

protected:

    //match camera internal id to a projector module for the images
    QMap<qint64, ModularSBASolver::ProjectorModule*> _cameraProjectors;

};

class PinholdeCamProjModule : public ModularSBASolver::ProjectorModule
{

public:

    PinholdeCamProjModule(Camera* associatedCamera);
    ~PinholdeCamProjModule();

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

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMAGEALIGNEMENTSBAMODULE_H
