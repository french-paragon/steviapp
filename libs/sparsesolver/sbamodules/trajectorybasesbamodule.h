#ifndef STEREOVISIONAPP_TRAJECTORYBASESBAMODULE_H
#define STEREOVISIONAPP_TRAJECTORYBASESBAMODULE_H

#include "../modularsbasolver.h"

namespace StereoVisionApp {

class TrajectoryBaseSBAModule : public ModularSBASolver::SBAModule
{

public:

    static const char* ModuleName;

    TrajectoryBaseSBAModule(double defaultIntegrationTime);

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

    bool _accelerometerBias;
    bool _accelerometerScale;

    std::vector<std::array<double,3>> _accelerometersBiases;
    std::vector<std::array<double,1>> _accelerometersScales;

    QMap<int, int> _accelerometerParametersIndex;

    double _defaultGpsAccuracy;
    double _defaultOrientAccuracy;

    double _defaultAccAccuracy;
    double _defaultGyroAccuracy;

    std::array<double,3> _gravity;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYBASESBAMODULE_H
