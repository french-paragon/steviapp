#include "eceftrajectoryinertialinstrumentssimulator.h"

#include "../geo/wgs84.h"

namespace StereoVisionApp {
namespace Simulation {

EcefTrajectoryFunctor::~EcefTrajectoryFunctor() {

}

EcefTrajectoryInertialInstrumentsSimulator::EcefTrajectoryInertialInstrumentsSimulator(EcefTrajectoryFunctor *trajectory,
                                                                                       StereoVision::Geometry::RigidBodyTransform<double> const& gps2body,
                                                                                       StereoVision::Geometry::RigidBodyTransform<double> const& ins2body) :
    _cachedMeasurementTime(std::nan("")),
    _functor(trajectory),
    _gps2body(gps2body),
    _ins2body(ins2body)
{

}

EcefTrajectoryInertialInstrumentsSimulator::~EcefTrajectoryInertialInstrumentsSimulator() {
    if (_functor != nullptr) {
        delete _functor;
    }
}

void EcefTrajectoryInertialInstrumentsSimulator::cacheMeasurement(double t) {

    _cachedMeasurementTime = t;

    if (_functor == nullptr) {
        return;
    }

    using D1_Jet = ceres::Jet<double,1>;
    class JetImpl : public ceres::Jet<ceres::Jet<double,1>,1> {
    public:
        JetImpl() : ceres::Jet<ceres::Jet<double,1>,1>() {

        }
        JetImpl(D1_Jet const& d1) : ceres::Jet<ceres::Jet<double,1>,1>(d1) {

        }
        JetImpl(ceres::Jet<ceres::Jet<double,1>,1> const& d2) : ceres::Jet<ceres::Jet<double,1>,1>(d2) {

        }
        JetImpl(JetImpl const& other) : ceres::Jet<ceres::Jet<double,1>,1>(other) {

        }
        JetImpl(double scalar) : ceres::Jet<ceres::Jet<double,1>,1>() {
            a.a = scalar;
            a.v[0] = 0;
            v[0].a = 0;
            v[0].v[0] = 0;
        }
        JetImpl(int scalar) : ceres::Jet<ceres::Jet<double,1>,1>() {
            a.a = scalar;
            a.v[0] = 0;
            v[0].a = 0;
            v[0].v[0] = 0;
        }

        bool operator< (double d) const {
            return a.a < d;
        }
        bool operator<= (double d) const {
            return a.a <= d;
        }
        bool operator> (double d) const {
            return a.a > d;
        }
        bool operator>= (double d) const {
            return a.a >= d;
        }

        bool operator< (int d) const {
            return a.a < d;
        }
        bool operator<= (int d) const {
            return a.a <= d;
        }
        bool operator> (int d) const {
            return a.a > d;
        }
        bool operator>= (int d) const {
            return a.a >= d;
        }

        operator double() const {
            return a.a;
        }
    };

    using D2_Jet = JetImpl;

    D2_Jet t_jet;
    t_jet.a.a = t;
    t_jet.a.v[0] = 1; //d t / dt = 1
    t_jet.v[0].a = 1; //d t / dt = 1
    t_jet.v[0].v[0] = 0; //d^2 t / dt^2 = 0

    D2_Jet zero_jet;
    zero_jet.a.a = 0;
    zero_jet.a.v[0] = 0; //d t / dt = 1
    zero_jet.v[0].a = 0; //d t / dt = 1
    zero_jet.v[0].v[0] = 0; //d^2 t / dt^2 = 0
    Eigen::Matrix<D2_Jet,3,1> zeros_jet(zero_jet,zero_jet,zero_jet);

    StereoVision::Geometry::RigidBodyTransform<double> inrt2ecef(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
    inrt2ecef.r.z() = -t*Geo::WGS84Ellipsoid::EarthRotationRate;
    StereoVision::Geometry::RigidBodyTransform<D2_Jet> ecef2inrt_jet(zeros_jet, zeros_jet);;
    ecef2inrt_jet.r.z() = t_jet*D2_Jet(D1_Jet(Geo::WGS84Ellipsoid::EarthRotationRate));

    auto tmp = _functor->trajectory(t_jet);
    StereoVision::Geometry::RigidBodyTransform<D2_Jet> body2ecef_jet;
    for (int i = 0; i < 3; i++) {
        body2ecef_jet.r[i] = tmp.r[i];
        body2ecef_jet.t[i] = tmp.t[i];
    }
    StereoVision::Geometry::RigidBodyTransform<D2_Jet> body2inrt_jet = ecef2inrt_jet*body2ecef_jet;

    Eigen::Vector3d accInertial;
    Eigen::Vector3d positionInertial;
    Eigen::Vector3d rotationInertial;
    Eigen::Vector3d orientationInertial;

    for (int i = 0; i < 3; i++) {
        accInertial[i] = body2inrt_jet.t[i].v[0].v[0];
        positionInertial[i] = body2inrt_jet.t[i].a.a;

        rotationInertial[i] = body2inrt_jet.r[i].v[0].a;
        orientationInertial[i] = body2inrt_jet.r[i].a.a;
    }

    //apply the jacobian of SO(3), so that R_future2inrt = R_current2inrt * dR, with dR = exp(rotationInertial*dt) = R_future2current
    rotationInertial = StereoVision::Geometry::diffRodriguezLieAlgebra(orientationInertial)*rotationInertial;

    StereoVision::Geometry::RigidBodyTransform<double> body2inrt(orientationInertial, positionInertial);
    _cachedMeasurement.body2ecef = inrt2ecef*body2inrt;

    Eigen::Vector3d accECEF = StereoVision::Geometry::angleAxisRotate(inrt2ecef.r, accInertial);
    Eigen::Vector3d rotationECEF = StereoVision::Geometry::angleAxisRotate(inrt2ecef.r, rotationInertial);

    _cachedMeasurement.gps = (_cachedMeasurement.body2ecef*_gps2body).t;

    Eigen::Vector3d speedEcef;
    for (int i = 0; i < 3; i++) {
        speedEcef[i] = body2ecef_jet.t[i].a.v[0];
    }
    _cachedMeasurement.gpsVelocity =
        StereoVision::Geometry::angleAxisRotate<double>(-_gps2body.r,
        StereoVision::Geometry::angleAxisRotate(inrt2ecef.r, speedEcef));

    auto gTmp = Geo::WGS84Ellipsoid::gravityEcefModel(_cachedMeasurement.body2ecef.t);
    Eigen::Vector3d gravity;

    for (int i = 0; i < 3; i++) {
        gravity[i] = gTmp[i];
    }

    double x = _cachedMeasurement.body2ecef.t.x();
    double y = _cachedMeasurement.body2ecef.t.y();
    double rotationRadiusEcef = sqrt(x*x+y*y);
    auto latlonh = Geo::WGS84Ellipsoid::Ecef2LatLonHeight(_cachedMeasurement.body2ecef.t);

    double earthRotGravEffect = Geo::WGS84Ellipsoid::EarthRotationRate*Geo::WGS84Ellipsoid::EarthRotationRate*rotationRadiusEcef;
    Eigen::Vector3d gDelta = Eigen::Vector3d(earthRotGravEffect*cos(latlonh[1]),earthRotGravEffect*sin(latlonh[1]),0); //compensate for centripedal acceleration already accounted for in gravity model
    gravity += gDelta;

    _cachedMeasurement.acc =
        StereoVision::Geometry::angleAxisRotate<double>(-_ins2body.r,
        StereoVision::Geometry::angleAxisRotate<double>(-_cachedMeasurement.body2ecef.r, accECEF + gravity));
    _cachedMeasurement.gyro =
        StereoVision::Geometry::angleAxisRotate<double>(-_ins2body.r,
        StereoVision::Geometry::angleAxisRotate<double>(-_cachedMeasurement.body2ecef.r, rotationECEF));

}

} // namespace Simulation
} // namespace StereoVisionApp
