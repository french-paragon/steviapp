#ifndef GENERATEDTRAJECTORY_H
#define GENERATEDTRAJECTORY_H

#include <datablocks/trajectory.h>

namespace StereoVisionApp {

/*!
 * \brief The GeneratedTrajectory class is a trajectory datablock where the trajectory data can be generated using functors rather than given from files.
 *
 * This class makes it much easier to build trajectories in unit tests
 */
class GeneratedTrajectory : public Trajectory
{
    Q_OBJECT
public:

    using TrajVectorFunctor = std::function<Eigen::Vector3d(double)>; //vector x, y, z as function of time

    struct TrajGeneratorInfos {
        TrajVectorFunctor functor;
        double step;
    };

    /*!
     * \brief configureStandardNonAccelaratingTrajectory configure a trajectory to represent a straight line with no acceleration
     * \param t0 initial time of the trajectory
     * \param tf final time of the trajectory
     * \param dtIns step interval for the inertial sensors
     * \param dtPos step interval for the position and orientation
     * \param x0 initial position
     * \param xf final position
     * \param r0 constant orientation
     * \param generatedTrajectory the trajectory to configure
     */
    static void configureStandardNonAccelaratingTrajectory(double t0,
                                                           double tf,
                                                           double dtIns,
                                                           double dtPos,
                                                           Eigen::Vector3d const& x0,
                                                           Eigen::Vector3d const& xf,
                                                           Eigen::Vector3d const& r0,
                                                           GeneratedTrajectory* generatedTrajectory);

    GeneratedTrajectory(Project* parent = nullptr);

    inline void setInitialAndFinalTimes(double t0, double tf) {

        _t0 = std::min(t0, tf);
        _tf = std::max(t0, tf);
    }

    inline void setAngularSpeedGenerator(TrajGeneratorInfos const& generator) {
        _angularSpeedGenerator = generator;
    }

    inline void setAccelerationGenerator(TrajGeneratorInfos const& generator) {
        _accelerationGenerator = generator;
    }

    inline void setPositionGenerator(TrajGeneratorInfos const& generator) {
        _positionGenerator = generator;
    }

    inline void setOrientationGenerator(TrajGeneratorInfos const& generator) {
        _orientationGenerator = generator;
    }

    inline TrajGeneratorInfos const& angularSpeedGenerator() const {
        return _angularSpeedGenerator;
    }

    inline TrajGeneratorInfos const& accelerationGenerator() const {
        return _accelerationGenerator;
    }

    inline TrajGeneratorInfos const& positionGenerator() const {
        return _positionGenerator;
    }

    inline TrajGeneratorInfos const& orientationGenerator() const {
        return _orientationGenerator;
    }

protected:

    virtual StatusOptionalReturn<std::vector<TimeCartesianBlock>> loadAngularSpeedRawData() const;
    virtual StatusOptionalReturn<std::vector<TimeCartesianBlock>> loadAccelerationRawData() const;
    virtual StatusOptionalReturn<RawGpsData> loadRawGPSData() const;
    virtual StatusOptionalReturn<std::vector<TimeCartesianBlock>> loadPositionRawData() const;
    virtual StatusOptionalReturn<std::vector<TimeCartesianBlock>> loadOrientationRawData() const;

    double _t0; //start time
    double _tf; //final time

    TrajGeneratorInfos _angularSpeedGenerator;
    TrajGeneratorInfos _accelerationGenerator;
    TrajGeneratorInfos _positionGenerator;
    TrajGeneratorInfos _orientationGenerator;

};

/*!
 * \brief The GeneratedTrajectoryFactory class pretend to be a Trajectory factory but build GeneratedTrajectory instead
 */
class GeneratedTrajectoryFactory : public DataBlockFactory
{
    Q_OBJECT
public:
    explicit GeneratedTrajectoryFactory(QObject* parent = nullptr);

    virtual QString TypeDescrName() const;
    virtual FactorizableFlags factorizable() const;
    virtual DataBlock* factorizeDataBlock(Project *parent = nullptr) const;

    virtual QString itemClassName() const;
};

} // namespace StereoVisionApp

#endif // GENERATEDTRAJECTORY_H
