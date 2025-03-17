#ifndef STEREOVISIONAPP_TRAJECTORYBASESBAMODULE_H
#define STEREOVISIONAPP_TRAJECTORYBASESBAMODULE_H

#include "../modularsbasolver.h"

namespace StereoVisionApp {

class TrajectoryBaseSBAModule : public ModularSBASolver::SBAModule
{

public:

    static const char* ModuleName;
    inline static void registerDefaultModuleFactory(SBASolverModulesInterface* interface) {
        interface->registerSBAModule(TrajectoryBaseSBAModule::ModuleName, [] (ModularSBASolver* solver) -> ModularSBASolver::SBAModule* {

            double gpsAccuracy = 0.02;
            double angularAccuracy = 0.1;
            double gyroAccuracy = 0.1;
            double accAccuracy = 0.5;
            double tiePointAccuracy = 0.5;

            double integrationtime = 0.5; //half a second

            StereoVisionApp::TrajectoryBaseSBAModule* trajectoryModule =
                    new StereoVisionApp::TrajectoryBaseSBAModule(integrationtime);

            trajectoryModule->setDefaultGpsAccuracy(gpsAccuracy);
            trajectoryModule->setDefaultOrientAccuracy(angularAccuracy);
            trajectoryModule->setDefaultGyroAccuracy(gyroAccuracy);
            trajectoryModule->setDefaultAccAccuracy(accAccuracy);

            return trajectoryModule;
        });
    }

    TrajectoryBaseSBAModule(double defaultIntegrationTime);

    virtual QString moduleName() const override;

    virtual bool addGraphReductorVariables(Project *currentProject, GenericSBAGraphReductor* graphReductor) override;
    virtual bool addGraphReductorObservations(Project *currentProject, GenericSBAGraphReductor* graphReductor) override;

    virtual bool setupParameters(ModularSBASolver* solver) override;
    virtual bool init(ModularSBASolver* solver, ceres::Problem & problem) override;
    virtual bool writeResults(ModularSBASolver* solver) override;
    virtual bool writeUncertainty(ModularSBASolver* solver) override;
    virtual void cleanup(ModularSBASolver* solver) override;

    inline double defaultGpsAccuracy() const
    {
        return _defaultGpsAccuracy;
    }

    inline void setDefaultGpsAccuracy(double newGpsAccuracy)
    {
        _defaultGpsAccuracy = newGpsAccuracy;
    }

    inline double defaultOrientAccuracy() const
    {
        return _defaultOrientAccuracy;
    }

    inline void setDefaultOrientAccuracy(double newOrientAccuracy)
    {
        _defaultOrientAccuracy = newOrientAccuracy;
    }

    inline double defaultAccAccuracy() const
    {
        return _defaultAccAccuracy;
    }

    inline void setDefaultAccAccuracy(double newAccAccuracy)
    {
        _defaultAccAccuracy = newAccAccuracy;
    }

    inline double defaultGyroAccuracy() const
    {
        return _defaultGyroAccuracy;
    }

    inline void setDefaultGyroAccuracy(double newGyroAccuracy)
    {
        _defaultGyroAccuracy = newGyroAccuracy;
    }

    inline bool hasOrientationPriorEnabled() const {
        return _useOrientationPriors;
    }

    inline void enableOrientationPrior(bool enabled) {
        _useOrientationPriors = enabled;
    }

protected:

    bool _useOrientationPriors;

    double _defaultIntegrationTime;

    std::vector<std::array<double,3>> _accelerometersBiases;
    std::vector<std::array<double,3>> _accelerometersScales;

    std::vector<std::array<double,3>> _gyrosBiases;
    std::vector<std::array<double,3>> _gyrosScales;

    QMap<int, int> _accelerometerParametersIndex;
    QMap<int, int> _gyroParametersIndex;

    double _defaultGpsAccuracy;
    double _defaultOrientAccuracy;

    double _defaultAccAccuracy;
    double _defaultGyroAccuracy;

    std::array<double,3> _gravity;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYBASESBAMODULE_H
