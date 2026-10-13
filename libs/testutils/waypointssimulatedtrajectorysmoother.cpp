#include "./waypointssimulatedtrajectorysmoother.h"

#include "geo/wgs84.h"

namespace StereoVisionApp {
namespace Simulation {


WayPointsEcefTrajectoryFunctor WayPointsEcefTrajectoryFunctor::fromWGS84WayPoints(std::vector<std::array<double,3>> const& wayPointsWgs84,
                                                                                  double speed,
                                                                                  double smoothTime) {
    std::vector<std::array<double,3>> wayPointsEcef(wayPointsWgs84.size());

    for (size_t i = 0; i < wayPointsWgs84.size(); i++) {
        wayPointsEcef[i] = Geo::WGS84Ellipsoid::LatLonHeight2ECEF(wayPointsWgs84[i]);
    }

    return WayPointsEcefTrajectoryFunctor(wayPointsEcef, speed, smoothTime);
}

WayPointsEcefTrajectoryFunctor::WayPointsEcefTrajectoryFunctor(std::vector<std::array<double,3>> const& wayPointsEcef, double speed, double smoothTime) :
    _smoothTime(smoothTime)
{

    std::vector<double> times = WayPointsSimulatedTrajectorySmoother::estimateTimesFromWaypoints(wayPointsEcef, speed);

    std::array<double,3> mean_pos{0,0,0};

    for (size_t i = 0; i < wayPointsEcef.size(); i++) {

        for (int j = 0; j < 3; j++) {
            mean_pos[j] += wayPointsEcef[i][j];
        }
    }
    for (int j = 0; j < 3; j++) {
        mean_pos[j] /= wayPointsEcef.size();
    }

    std::array<double,3> gravity = Geo::WGS84Ellipsoid::gravityEcefModel(mean_pos);

    WayPointsSimulatedTrajectorySmoother::TimesWaypointsSet<double> orientations =
        WayPointsSimulatedTrajectorySmoother::estimateAircraftOrientation(wayPointsEcef,times, gravity, smoothTime);

    _position_waypoints.times.resize(times.size());
    _position_waypoints.samples.resize(wayPointsEcef.size());

    for (size_t i = 0; i < wayPointsEcef.size(); i++) {
        _position_waypoints.times[i] = times[i];
        for (int j = 0; j < 3; j++) {
            _position_waypoints.samples[i][j] = wayPointsEcef[i][j];
        }
    }

    _orientation_waypoints.times.resize(orientations.times.size());
    _orientation_waypoints.samples.resize(orientations.samples.size());

    for (size_t i = 0; i < orientations.samples.size(); i++) {
        _orientation_waypoints.times[i] = orientations.times[i];
        for (int j = 0; j < 3; j++) {
            _orientation_waypoints.samples[i][j] = orientations.samples[i][j];
        }
    }

}

WayPointsEcefTrajectoryFunctor::~WayPointsEcefTrajectoryFunctor() {

}


StereoVision::Geometry::RigidBodyTransform<ceres::Jet<ceres::Jet<double,1>,1>> WayPointsEcefTrajectoryFunctor::trajectory(ceres::Jet<ceres::Jet<double,1>,1> const& t_ext) const {

    D2Jet t = t_ext;

    std::array<D2Jet,3> pos = WayPointsSimulatedTrajectorySmoother::sampleTrajectory(t, _position_waypoints.samples, _position_waypoints.times, _smoothTime);
    Eigen::Matrix<InertialTrajectoryFunctor::D2_Jet,3,1> rot;
    if (_orientation_waypoints.samples.empty()) {
        for (int i = 0; i < 3; i++) {
            rot[i] = D2_Jet(0.0);
        }
    } else if ((_orientation_waypoints.samples.size() == 1)) {
        for (int i = 0; i < 3; i++) {
            rot[i] = _orientation_waypoints.samples[0][i];
        }
    } else {

        if (t <= _orientation_waypoints.times[0]) {
            for (int i = 0; i < 3; i++) {
                rot[i] = _orientation_waypoints.samples[0][i];
            }
        } else if ( t >= _orientation_waypoints.times.back()) {
            for (int i = 0; i < 3; i++) {
                rot[i] = _orientation_waypoints.samples.back()[i];
            }
        } else {
            for (size_t i = 0; i < _orientation_waypoints.samples.size()-1; i++) {
                D2_Jet t0 = _orientation_waypoints.times[i];
                D2_Jet t1 = _orientation_waypoints.times[i+1];
                D2_Jet dt = t1-t0;
                if (t >= t0 and t < t1) {
                    D2_Jet w0 = (t1-t)/dt;
                    D2_Jet w1 = (t-t0)/dt;

                    Eigen::Matrix<D2_Jet,3,1> r0;
                    Eigen::Matrix<D2_Jet,3,1> r1;
                    Eigen::Matrix<D2_Jet,3,1> zero;

                    for (size_t j = 0; j < 3; j++) {
                        r0[j] = _orientation_waypoints.samples[i][j];
                        r1[j] = _orientation_waypoints.samples[i+1][j];
                        zero[j] = D2_Jet(0);
                    }
                    StereoVision::Geometry::RigidBodyTransform<D2_Jet> t0(
                        r0,
                        zero);
                    StereoVision::Geometry::RigidBodyTransform<D2_Jet> t1(
                        r1,
                        zero);

                    StereoVision::Geometry::RigidBodyTransform<D2_Jet> interp =
                        StereoVision::Geometry::interpolateRigidBodyTransformOnManifold(w0,t0,w1,t1);

                    rot = interp.r;
                    break;
                }
            }
        }
    }
    //std::array<D2Jet,3> rot = WayPointsSimulatedTrajectorySmoother::sampleTrajectory(t, _orientation_waypoints.samples, _orientation_waypoints.times, _smoothTime);

    return StereoVision::Geometry::RigidBodyTransform<ceres::Jet<ceres::Jet<double,1>,1>>(
        Eigen::Matrix<ceres::Jet<ceres::Jet<double,1>,1>,3,1>(rot[0],rot[1],rot[2]),
        Eigen::Matrix<ceres::Jet<ceres::Jet<double,1>,1>,3,1>(pos[0],pos[1],pos[2]));

}

WayPointsSimulatedTrajectorySmoother::TimesWaypointsSet<double> WayPointsEcefTrajectoryFunctor::positionWaypoints() const {
    WayPointsSimulatedTrajectorySmoother::TimesWaypointsSet<double> ret;
    ret.times.reserve(_position_waypoints.times.size());
    ret.samples.reserve(_position_waypoints.samples.size());

    for (int i = 0; i < _position_waypoints.times.size(); i++) {
        ret.times.push_back(double(_position_waypoints.times[i]));
    }

    for (int i = 0; i < _position_waypoints.samples.size(); i++) {
        std::array<double,3> sample;
        for (int j = 0; j < 3; j++) {
            sample[j] = double(_position_waypoints.samples[i][j]);
        }
        ret.samples.push_back(sample);
    }

    return ret;
}
WayPointsSimulatedTrajectorySmoother::TimesWaypointsSet<double> WayPointsEcefTrajectoryFunctor::orientationWaypoints() const {
    WayPointsSimulatedTrajectorySmoother::TimesWaypointsSet<double> ret;
    ret.times.reserve(_orientation_waypoints.times.size());
    ret.samples.reserve(_orientation_waypoints.samples.size());

    for (int i = 0; i < _orientation_waypoints.times.size(); i++) {
        ret.times.push_back(double(_orientation_waypoints.times[i]));
    }

    for (int i = 0; i < _orientation_waypoints.samples.size(); i++) {
        std::array<double,3> sample;
        for (int j = 0; j < 3; j++) {
            sample[j] = double(_orientation_waypoints.samples[i][j]);
        }
        ret.samples.push_back(sample);
    }

    return ret;
}

} // namespace Simulation
} // namespace StereoVisionApp
