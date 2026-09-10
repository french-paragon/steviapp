#ifndef ECEFTRAJECTORYINERTIALINSTRUMENTSSIMULATOR_H
#define ECEFTRAJECTORYINERTIALINSTRUMENTSSIMULATOR_H

#include <ceres/jet.h> //used for automatic differentiation

#include <StereoVision/geometry/rotations.h>

namespace StereoVisionApp {
namespace Simulation {

class EcefTrajectoryFunctor {
public :

    virtual ~EcefTrajectoryFunctor();

    /*!
     * \brief trajectory compute the trajectory, nested ceres jets allow to compute derivatives up to the second order automatically
     * \param t the time
     * \return the pose (body2ecef) at time t
     */
    virtual StereoVision::Geometry::RigidBodyTransform<ceres::Jet<ceres::Jet<double,1>,1>> trajectory(ceres::Jet<ceres::Jet<double,1>,1> const& t) const = 0;
};

/*!
 * \brief The EcefTrajectoryInertialInstrumentsSimulator class, from a trajectory function, give noiseless measurements for instruments used in navigation systems.
 *
 * This class use caching, so it is more efficient to collect all measurements for a given time, than all the measure for a single instrument, and then the next instrument, ect.
 */
class EcefTrajectoryInertialInstrumentsSimulator
{
public:

    struct Measurement {
        StereoVision::Geometry::RigidBodyTransform<double> body2ecef;
        Eigen::Vector3d gps; //gps measurement
        Eigen::Vector3d gpsVelocity; //gps velocity measurement
        Eigen::Vector3d gyro; //gyro measurement
        Eigen::Vector3d acc; //accelerometer measurement
    };

    /*!
     * \brief EcefTrajectoryInertialInstrumentsSimulator build a simulator with a given trajectory
     * \param trajectory the functor representing the trajectory. The simulator will take ownership of it
     */
    explicit EcefTrajectoryInertialInstrumentsSimulator(EcefTrajectoryFunctor* trajectory,
                                                        StereoVision::Geometry::RigidBodyTransform<double> const& gps2body =
                                                        StereoVision::Geometry::RigidBodyTransform<double>(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
                                                        StereoVision::Geometry::RigidBodyTransform<double> const& ins2body =
                                                        StereoVision::Geometry::RigidBodyTransform<double>(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
    ~EcefTrajectoryInertialInstrumentsSimulator();

    inline StereoVision::Geometry::RigidBodyTransform<double> body2ecef(double t) {
        if (std::isfinite(t) and t != _cachedMeasurementTime) {
            cacheMeasurement(t);
        }
        return _cachedMeasurement.body2ecef;
    }
    inline Eigen::Vector3d gps(double t) {
        if (std::isfinite(t) and t != _cachedMeasurementTime) {
            cacheMeasurement(t);
        }
        return _cachedMeasurement.gps;
    }
    inline Eigen::Vector3d gpsVelocity(double t) {
        if (std::isfinite(t) and t != _cachedMeasurementTime) {
            cacheMeasurement(t);
        }
        return _cachedMeasurement.gpsVelocity;
    }
    inline Eigen::Vector3d gyro(double t) {
        if (std::isfinite(t) and t != _cachedMeasurementTime) {
            cacheMeasurement(t);
        }
        return _cachedMeasurement.gyro;
    }
    inline Eigen::Vector3d acc(double t) {
        if (std::isfinite(t) and t != _cachedMeasurementTime) {
            cacheMeasurement(t);
        }
        return _cachedMeasurement.acc;
    }

protected:

    void cacheMeasurement(double t);

    double _cachedMeasurementTime;
    Measurement _cachedMeasurement;
    EcefTrajectoryFunctor* _functor;
    StereoVision::Geometry::RigidBodyTransform<double> const& _gps2body;
    StereoVision::Geometry::RigidBodyTransform<double> const& _ins2body;
};

} // namespace Simulation
} // namespace StereoVisionApp

#endif // ECEFTRAJECTORYINERTIALINSTRUMENTSSIMULATOR_H
