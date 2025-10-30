#include "generatedtrajectory.h"

namespace StereoVisionApp {

GeneratedTrajectory::GeneratedTrajectory(Project* parent) : Trajectory(parent) {

}


void GeneratedTrajectory::configureStandardNonAccelaratingTrajectory(double t0,
                                                                     double tf,
                                                                     double dtIns,
                                                                     double dtPos,
                                                                     Eigen::Vector3d const& x0,
                                                                     Eigen::Vector3d const& xf,
                                                                     Eigen::Vector3d const& r0,
                                                                     GeneratedTrajectory* generatedTrajectory) {

    generatedTrajectory->setInitialAndFinalTimes(t0,tf);

    generatedTrajectory->setAngularSpeedGenerator(TrajGeneratorInfos{[] (double t) { return Eigen::Vector3d::Zero(); }, dtIns});
    generatedTrajectory->setAccelerationGenerator(TrajGeneratorInfos{[] (double t) { return Eigen::Vector3d::Zero(); }, dtIns});

    generatedTrajectory->setPositionGenerator(TrajGeneratorInfos{[x0, xf, t0, tf] (double t) { return ((tf-t)*x0 + (t-t0)*xf)/(tf-t0); }, dtPos});
    generatedTrajectory->setOrientationGenerator(TrajGeneratorInfos{[r0] (double t) { return r0; }, dtPos});

}

StatusOptionalReturn<std::vector<Trajectory::TimeCartesianBlock>> GeneratedTrajectory::loadAngularSpeedRawData() const {

    if (!_angularSpeedGenerator.functor) {
        return Trajectory::loadAngularSpeedRawData();
    }

    double interval = _tf - _t0;
    double steps = interval / _angularSpeedGenerator.step;
    long maxxedSteps = std::ceil(steps) + 1;

    double missing = (maxxedSteps-1)*_angularSpeedGenerator.step - interval;
    double delta = missing/2;

    std::vector<Trajectory::TimeCartesianBlock> ret(maxxedSteps);

    double t = _t0 - delta;

    for (long i = 0; i < maxxedSteps; i++) {
        ret[i] = {t, _angularSpeedGenerator.functor(t)};
        t += _angularSpeedGenerator.step;
    }

    return ret;

}
StatusOptionalReturn<std::vector<Trajectory::TimeCartesianBlock>> GeneratedTrajectory::loadAccelerationRawData() const {

    if (!_accelerationGenerator.functor) {
        return Trajectory::loadAccelerationRawData();
    }

    double interval = _tf - _t0;
    double steps = interval / _accelerationGenerator.step;
    long maxxedSteps = std::ceil(steps) + 1;

    double missing = (maxxedSteps-1)*_accelerationGenerator.step - interval;
    double delta = missing/2;

    std::vector<Trajectory::TimeCartesianBlock> ret(maxxedSteps);

    double t = _t0 - delta;

    for (long i = 0; i < maxxedSteps; i++) {
        ret[i] = {t, _accelerationGenerator.functor(t)};
        t += _accelerationGenerator.step;
    }

    return ret;
}
StatusOptionalReturn<Trajectory::RawGpsData> GeneratedTrajectory::loadRawGPSData() const {
    return StatusOptionalReturn<Trajectory::RawGpsData>::error("Not implemented yet");
}
StatusOptionalReturn<std::vector<Trajectory::TimeCartesianBlock>> GeneratedTrajectory::loadPositionRawData() const {

    if (!_positionGenerator.functor) {
        return Trajectory::loadPositionRawData();
    }

    double interval = _tf - _t0;
    double steps = interval / _positionGenerator.step;
    long maxxedSteps = std::ceil(steps) + 1;

    double missing = (maxxedSteps-1)*_positionGenerator.step - interval;
    double delta = missing/2;

    std::vector<Trajectory::TimeCartesianBlock> ret(maxxedSteps);

    double t = _t0 - delta;

    for (long i = 0; i < maxxedSteps; i++) {
        ret[i] = {t, _positionGenerator.functor(t)};
        t += _positionGenerator.step;
    }

    return ret;
}
StatusOptionalReturn<std::vector<Trajectory::TimeCartesianBlock>> GeneratedTrajectory::loadOrientationRawData() const {

    if (!_orientationGenerator.functor) {
        return Trajectory::loadOrientationRawData();
    }

    double interval = _tf - _t0;
    double steps = interval / _orientationGenerator.step;
    long maxxedSteps = std::ceil(steps) + 1;

    double missing = (maxxedSteps-1)*_orientationGenerator.step - interval;
    double delta = missing/2;

    std::vector<Trajectory::TimeCartesianBlock> ret(maxxedSteps);

    double t = _t0 - delta;

    for (long i = 0; i < maxxedSteps; i++) {
        ret[i] = {t, _orientationGenerator.functor(t)};
        t += _orientationGenerator.step;
    }

    return ret;
}


GeneratedTrajectoryFactory::GeneratedTrajectoryFactory(QObject* parent) :
    DataBlockFactory(parent)
{

}

QString GeneratedTrajectoryFactory::TypeDescrName() const {
    return tr("Trajectory");
}
GeneratedTrajectoryFactory::FactorizableFlags GeneratedTrajectoryFactory::factorizable() const {
    return DataBlockFactory::RootDataBlock;
}
DataBlock* GeneratedTrajectoryFactory::factorizeDataBlock(Project *parent) const {
    return new GeneratedTrajectory(parent);
}

QString GeneratedTrajectoryFactory::itemClassName() const {
    return Trajectory::staticMetaObject.className();
}

} // namespace StereoVisionApp
