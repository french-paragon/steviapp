#ifndef STEREOVISIONAPP_TRAJECTORYBASESBAMODULE_H
#define STEREOVISIONAPP_TRAJECTORYBASESBAMODULE_H

#include "../modularsbasolver.h"

namespace StereoVisionApp {

class TrajectoryBaseSBAModule : public ModularSBASolver::SBAModule
{

public:

    TrajectoryBaseSBAModule(double integrationTime);

    virtual bool init(ModularSBASolver* solver, ceres::Problem & problem) override;
    virtual bool writeResults(ModularSBASolver* solver) override;
    virtual bool writeUncertainty(ModularSBASolver* solver) override;
    virtual void cleanup(ModularSBASolver* solver) override;

    inline double gpsAccuracy() const
    {
        return _gpsAccuracy;
    }

    inline void setGpsAccuracy(double newGpsAccuracy)
    {
        _gpsAccuracy = newGpsAccuracy;
    }

    inline double orientAccuracy() const
    {
        return _orientAccuracy;
    }

    inline void setOrientAccuracy(double newOrientAccuracy)
    {
        _orientAccuracy = newOrientAccuracy;
    }

    inline double accAccuracy() const
    {
        return _accAccuracy;
    }

    inline void setAccAccuracy(double newAccAccuracy)
    {
        _accAccuracy = newAccAccuracy;
    }

    inline double gyroAccuracy() const
    {
        return _gyroAccuracy;
    }

    inline void setGyroAccuracy(double newGyroAccuracy)
    {
        _gyroAccuracy = newGyroAccuracy;
    }

protected:

    double _integrationTime;

    bool _accelerometerBias;
    bool _accelerometerScale;

    std::vector<std::array<double,3>> _accelerometersBiases;
    std::vector<std::array<double,1>> _accelerometersScales;

    QMap<int, int> _accelerometerParametersIndex;

    double _gpsAccuracy;
    double _orientAccuracy;

    double _accAccuracy;
    double _gyroAccuracy;

    std::array<double,3> _gravity;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYBASESBAMODULE_H
