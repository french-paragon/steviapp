#include <QtTest/QtTest>


#include "testutils/datablocks/generatedtrajectory.h"
#include "testutils/eceftrajectoryinertialinstrumentssimulator.h"
#include "testutils/waypointssimulatedtrajectorysmoother.h"

#include "geo/wgs84.h"

/*!
 * \brief The TestTestUtils class is a test to test the test utils (yes, a bit cumbersome, but better of checking the stuff you will use to test other stuff
 */
class TestTestUtils : public QObject
{
    Q_OBJECT
public:

private Q_SLOTS:

    void initTestCase();

    void testNestedJets();
    void testEcefTrajectoryInstrumentsSimulator();
    void testWayPointsEcefTrajectoryInterpolation();
    void testWayPointsEcefTrajectoryFunctor();
    void testWayPointsEcefTrajectoryFunctorInertialMeasurements();
    void testGeneratedTrajectory();

protected:

};

void TestTestUtils::initTestCase() {

    srand(time(nullptr));

}

void TestTestUtils::testNestedJets() {
    //we rely on nested jets for our eceftrajectoryinertialinstrumentssimulator class, this test check they work as expected
    using D2_Jet = ceres::Jet<ceres::Jet<double,1>,1>;

    std::vector<double> times = {4.2, 69, 0.33, -10, 3.1415926535};

    for (double t : times) {

        D2_Jet t_jet;
        t_jet.a.a = t;
        t_jet.a.v[0] = 1; //d t / dt = 1
        t_jet.v[0].a = 1; //d t / dt = 1
        t_jet.v[0].v[0] = 0; //d^2 t / dt^2 = 0

        D2_Jet t2_jet = t_jet*t_jet;

        QCOMPARE(t2_jet.a.a, t*t);
        QCOMPARE(t2_jet.a.v[0], 2*t);
        QCOMPARE(t2_jet.v[0].a, 2*t);
        QCOMPARE(t2_jet.v[0].v[0], 2);

        D2_Jet sint_jet = sin(t_jet);

        QCOMPARE(sint_jet.a.a, sin(t));
        QCOMPARE(sint_jet.a.v[0], cos(t));
        QCOMPARE(sint_jet.v[0].a, cos(t));
        QCOMPARE(sint_jet.v[0].v[0], -sin(t));

        D2_Jet cost_jet = cos(t_jet);

        QCOMPARE(cost_jet.a.a, cos(t));
        QCOMPARE(cost_jet.a.v[0], -sin(t));
        QCOMPARE(cost_jet.v[0].a, -sin(t));
        QCOMPARE(cost_jet.v[0].v[0], -cos(t));

        if (t > 0) {
            D2_Jet sqrtt_jet = sqrt(t_jet);

            QCOMPARE(sqrtt_jet.a.a, sqrt(t));
            QCOMPARE(sqrtt_jet.a.v[0], 0.5/sqrt(t));
            QCOMPARE(sqrtt_jet.v[0].a, 0.5/sqrt(t));
            QCOMPARE(sqrtt_jet.v[0].v[0], -0.25/(t*sqrt(t)));
        }

        D2_Jet expt_jet = exp(t_jet);

        QCOMPARE(expt_jet.a.a, exp(t));
        QCOMPARE(expt_jet.a.v[0], exp(t));
        QCOMPARE(expt_jet.v[0].a, exp(t));
        QCOMPARE(expt_jet.v[0].v[0], exp(t));

        if (t > 0) {
            D2_Jet logt_jet = log(t_jet);

            QCOMPARE(logt_jet.a.a, log(t));
            QCOMPARE(logt_jet.a.v[0], 1/t);
            QCOMPARE(logt_jet.v[0].a, 1/t);
            QCOMPARE(logt_jet.v[0].v[0], -1/(t*t));
        }
    }
}

void TestTestUtils::testEcefTrajectoryInstrumentsSimulator() {

    /*!
     * \brief The StaticPlatformTrajectory class represent a static object in ecef frame
     */
    class StaticPlatformTrajectory : public StereoVisionApp::Simulation::InertialTrajectoryFunctor {
    public:
        virtual StereoVision::Geometry::RigidBodyTransform<ceres::Jet<ceres::Jet<double,1>,1>> trajectory(ceres::Jet<ceres::Jet<double,1>,1> const& t) const override {
            ceres::Jet<ceres::Jet<double,1>,1> zero;
            zero.a.a = 0;
            Eigen::Matrix<ceres::Jet<ceres::Jet<double,1>,1>,3,1> zeros(zero,zero,zero);
            ceres::Jet<ceres::Jet<double,1>,1> earthRadius;
            earthRadius.a.a = StereoVisionApp::Geo::WGS84Ellipsoid::SemiMajorAxis;

            Eigen::Matrix<ceres::Jet<ceres::Jet<double,1>,1>,3,1> rot(zero, zero, zero);
            Eigen::Matrix<ceres::Jet<ceres::Jet<double,1>,1>,3,1> pos(earthRadius, zero, zero);

            return StereoVision::Geometry::RigidBodyTransform<ceres::Jet<ceres::Jet<double,1>,1>>(rot, pos);
        }

        static Eigen::Vector3d positionGt(double t) {
            return Eigen::Matrix<double,3,1>(StereoVisionApp::Geo::WGS84Ellipsoid::SemiMajorAxis, 0, 0);
        }

        static Eigen::Vector3d orientationGt(double t) {
            return Eigen::Matrix<double,3,1>(0, 0, 0);
        }

        static Eigen::Vector3d gpsGt(double t) {
            return Eigen::Matrix<double,3,1>(StereoVisionApp::Geo::WGS84Ellipsoid::SemiMajorAxis, 0, 0);
        }

        static Eigen::Vector3d gpsVelocityGt(double t) {
            return Eigen::Matrix<double,3,1>(0, 0, 0);
        }

        static Eigen::Vector3d gyroGt(double t) {
            return Eigen::Matrix<double,3,1>(0, 0, StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate);
        }

        static Eigen::Vector3d accGt(double t) {
            auto gTmp = StereoVisionApp::Geo::WGS84Ellipsoid::gravityEcefModel(positionGt(t));
            Eigen::Vector3d g;
            for (int i = 0; i < 3; i++) {
                g[i] = gTmp[i];
            }
            return g;
        }
    };

    /*!
     * \brief The InertialStaticPlatformTrajectory class represent a static object in static frame, which trajectory is expressed in ECEF
     */
    class InertialStaticPlatformTrajectory : public StereoVisionApp::Simulation::InertialTrajectoryFunctor {
    public:
        virtual StereoVision::Geometry::RigidBodyTransform<ceres::Jet<ceres::Jet<double,1>,1>> trajectory(ceres::Jet<ceres::Jet<double,1>,1> const& t) const override {
            ceres::Jet<ceres::Jet<double,1>,1> zero;
            zero.a.a = 0;
            Eigen::Matrix<ceres::Jet<ceres::Jet<double,1>,1>,3,1> zeros(zero,zero,zero);
            ceres::Jet<ceres::Jet<double,1>,1> earthRadius;
            earthRadius.a.a = StereoVisionApp::Geo::WGS84Ellipsoid::SemiMajorAxis;
            ceres::Jet<ceres::Jet<double,1>,1> rotRate;
            rotRate.a.a = StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate;

            ceres::Jet<ceres::Jet<double,1>,1> angle = -t*rotRate;

            Eigen::Matrix<ceres::Jet<ceres::Jet<double,1>,1>,3,1> rot(zero, zero, angle);
            Eigen::Matrix<ceres::Jet<ceres::Jet<double,1>,1>,3,1> pos(cos(angle)*earthRadius, sin(angle)*earthRadius, zero);

            return StereoVision::Geometry::RigidBodyTransform<ceres::Jet<ceres::Jet<double,1>,1>>(rot, pos);
        }

        static Eigen::Vector3d positionGt(double t) {
            double radius = StereoVisionApp::Geo::WGS84Ellipsoid::SemiMajorAxis;
            double rotRate = StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate;
            double angle = -t*rotRate;
            return Eigen::Matrix<double,3,1>(cos(angle)*radius, sin(angle)*radius, 0);
        }

        static Eigen::Vector3d orientationGt(double t) {
            return Eigen::Matrix<double,3,1>(0, 0, -t*StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate);
        }

        static Eigen::Vector3d gpsGt(double t) {
            return positionGt(t);
        }

        static Eigen::Vector3d gpsVelocityGt(double t) {
            double rotRate = StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate;
            double earthRadius = StereoVisionApp::Geo::WGS84Ellipsoid::SemiMajorAxis;

            double angle = -t*rotRate;
            double speed = -rotRate*earthRadius;

            Eigen::Matrix<double,3,1> speed_ecef(-sin(angle)*speed, cos(angle)*speed, 0);

            constexpr char wgs84_ecef[] = "EPSG:4978";
            const StereoVisionApp::Geo::TopocentricConvention topoConv = static_cast<StereoVisionApp::Geo::TopocentricConvention>(
                StereoVisionApp::Simulation::EcefTrajectoryInertialInstrumentsSimulator::LocalFrameDefinitionUsed);
            std::optional<StereoVision::Geometry::RigidBodyTransform<double>> local2ecefOpt =
                StereoVisionApp::Geo::getLTPC2ECEF(positionGt(t), wgs84_ecef, topoConv);

            StereoVision::Geometry::RigidBodyTransform<double> local2ecef(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

            if (local2ecefOpt.has_value()) {
                local2ecef = local2ecefOpt.value();
            }

            return StereoVision::Geometry::angleAxisRotate<double>(-local2ecef.r, speed_ecef);
        }

        static Eigen::Vector3d gyroGt(double t) {
            return Eigen::Matrix<double,3,1>(0, 0, 0);
        }

        static Eigen::Vector3d accGt(double t) {
            auto gTmp = StereoVisionApp::Geo::WGS84Ellipsoid::gravityEcefModel(positionGt(t));
            Eigen::Vector3d g;
            for (int i = 0; i < 3; i++) {
                g[i] = gTmp[i];
            }

            Eigen::Vector3d posEcef = positionGt(t);

            double x = posEcef.x();
            double y = posEcef.y();

            double rotRateScale = StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate*StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate;
            Eigen::Vector3d gDelta = Eigen::Vector3d(rotRateScale*x,rotRateScale*y,0); //compensate for centripedal acceleration already accounted for in gravity model

            g += gDelta;

            Eigen::Matrix<double,3,1> r_ecef_2_local(0, 0, t*StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate);

            return StereoVision::Geometry::angleAxisRotate(r_ecef_2_local, g);
        }
    };

    constexpr double move_north_speed = 200;

    class MoveNorthTrajectory : public StereoVisionApp::Simulation::InertialTrajectoryFunctor {
    public:
        virtual StereoVision::Geometry::RigidBodyTransform<ceres::Jet<ceres::Jet<double,1>,1>> trajectory(ceres::Jet<ceres::Jet<double,1>,1> const& t) const override {
            ceres::Jet<ceres::Jet<double,1>,1> zero;
            zero.a.a = 0;
            Eigen::Matrix<ceres::Jet<ceres::Jet<double,1>,1>,3,1> zeros(zero,zero,zero);
            ceres::Jet<ceres::Jet<double,1>,1> earthRadius;
            earthRadius.a.a = StereoVisionApp::Geo::WGS84Ellipsoid::SemiMajorAxis;

            ceres::Jet<ceres::Jet<double,1>,1> speed;
            speed.a.a = move_north_speed; //200m/s

            ceres::Jet<ceres::Jet<double,1>,1> dist = t*speed;
            ceres::Jet<ceres::Jet<double,1>,1> angle = dist / earthRadius;

            Eigen::Matrix<ceres::Jet<ceres::Jet<double,1>,1>,3,1> rot(zero, angle, zero);
            Eigen::Matrix<ceres::Jet<ceres::Jet<double,1>,1>,3,1> pos(cos(angle)*earthRadius, zero, sin(angle)*earthRadius);

            return StereoVision::Geometry::RigidBodyTransform<ceres::Jet<ceres::Jet<double,1>,1>>(rot, pos);
        }

        static Eigen::Vector3d positionGt(double t) {
            constexpr double earthRadius = StereoVisionApp::Geo::WGS84Ellipsoid::SemiMajorAxis;
            double dist = t*move_north_speed;
            double angle = dist / earthRadius;
            return Eigen::Matrix<double,3,1>(cos(angle)*earthRadius, 0, sin(angle)*earthRadius);
        }

        static Eigen::Vector3d orientationGt(double t) {
            constexpr double earthRadius = StereoVisionApp::Geo::WGS84Ellipsoid::SemiMajorAxis;
            double dist = t*move_north_speed;
            double angle = dist / earthRadius;
            return Eigen::Matrix<double,3,1>(0, angle, 0);
        }

        static Eigen::Vector3d gpsGt(double t) {
            return positionGt(t);
        }

        static Eigen::Vector3d gpsVelocityGt(double t) {
            constexpr double earthRadius = StereoVisionApp::Geo::WGS84Ellipsoid::SemiMajorAxis;
            double dist = t*move_north_speed;
            double angle = dist / earthRadius;

            Eigen::Matrix<double,3,1> speed_ecef(-sin(angle)*move_north_speed, 0, cos(angle)*move_north_speed);

            constexpr char wgs84_ecef[] = "EPSG:4978";
            const StereoVisionApp::Geo::TopocentricConvention topoConv = static_cast<StereoVisionApp::Geo::TopocentricConvention>(
                StereoVisionApp::Simulation::EcefTrajectoryInertialInstrumentsSimulator::LocalFrameDefinitionUsed);
            std::optional<StereoVision::Geometry::RigidBodyTransform<double>> local2ecefOpt =
                StereoVisionApp::Geo::getLTPC2ECEF(positionGt(t), wgs84_ecef, topoConv);

            StereoVision::Geometry::RigidBodyTransform<double> local2ecef(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

            if (local2ecefOpt.has_value()) {
                local2ecef = local2ecefOpt.value();
            }

            return StereoVision::Geometry::angleAxisRotate<double>(-local2ecef.r, speed_ecef);
        }

        static Eigen::Vector3d gyroGt(double t) {
            constexpr double earthRadius = StereoVisionApp::Geo::WGS84Ellipsoid::SemiMajorAxis;
            double angle_speed = move_north_speed / earthRadius;
            double dist = t*move_north_speed;
            double angle = dist / earthRadius;
            return Eigen::Matrix<double,3,1>(-sin(angle)*StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate,
                                               angle_speed,
                                               cos(angle)*StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate);
        }

        static Eigen::Vector3d accGt(double t) {
            auto gTmp = StereoVisionApp::Geo::WGS84Ellipsoid::gravityEcefModel(positionGt(t));
            Eigen::Vector3d acc;
            for (int i = 0; i < 3; i++) {
                acc[i] = gTmp[i];
            }

            constexpr double earthRadius = StereoVisionApp::Geo::WGS84Ellipsoid::SemiMajorAxis;
            double angle_speed = move_north_speed / earthRadius;
            double dist = t*move_north_speed;
            double angle = dist / earthRadius;

            Eigen::Vector3d accRot = Eigen::Vector3d(-cos(angle)*angle_speed*angle_speed*earthRadius,0,-sin(angle)*angle_speed*angle_speed*earthRadius);
            Eigen::Vector3d accCoriolis = -2*Eigen::Vector3d(0,0,StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate).cross(
                Eigen::Vector3d(sin(angle)*move_north_speed,0,cos(angle)*move_north_speed));

            Eigen::Matrix<double,3,1> r_ecef_2_local(0, -angle, 0);

            return StereoVision::Geometry::angleAxisRotate<double>(r_ecef_2_local, acc + accRot + accCoriolis);
        }
    };

    //test trajectories

    StaticPlatformTrajectory* staticTraj = new StaticPlatformTrajectory();

    StereoVisionApp::Simulation::EcefTrajectoryInertialInstrumentsSimulator staticTrajSimulator(staticTraj);

    std::array<double,3> testTimes{0.,6*3600.,12*3600.};

    for (double t : testTimes) {

        Eigen::Vector3d gpsGt = StaticPlatformTrajectory::gpsGt(t);
        Eigen::Vector3d gpsPred = staticTrajSimulator.gps(t);

        for (int i = 0; i < 3; i++) {
            QCOMPARE(gpsPred[i],gpsGt[i]);
        }

        Eigen::Vector3d gpsVelocityGt = StaticPlatformTrajectory::gpsVelocityGt(t);
        Eigen::Vector3d gpsVelocityPred = staticTrajSimulator.gpsVelocity(t);

        for (int i = 0; i < 3; i++) {
            QCOMPARE(gpsVelocityPred[i],gpsVelocityGt[i]);
        }

        Eigen::Vector3d gyroGt = StaticPlatformTrajectory::gyroGt(t);
        Eigen::Vector3d gyroPred = staticTrajSimulator.gyro(t);

        for (int i = 0; i < 3; i++) {
            QCOMPARE(gyroPred[i],gyroGt[i]);
        }

        Eigen::Vector3d accGt = StaticPlatformTrajectory::accGt(t);
        Eigen::Vector3d accPred = staticTrajSimulator.acc(t);

        for (int i = 0; i < 3; i++) {
            QCOMPARE(accPred[i],accGt[i]);
        }

    }

    InertialStaticPlatformTrajectory* inertialStaticTraj = new InertialStaticPlatformTrajectory();

    StereoVisionApp::Simulation::EcefTrajectoryInertialInstrumentsSimulator inertialStaticTrajSimulator(inertialStaticTraj);

    for (double t : testTimes) {

        Eigen::Vector3d gpsGt = InertialStaticPlatformTrajectory::gpsGt(t);
        Eigen::Vector3d gpsPred = inertialStaticTrajSimulator.gps(t);

        for (int i = 0; i < 3; i++) {
            QCOMPARE(gpsPred[i],gpsGt[i]);
        }

        Eigen::Vector3d gpsVelocityGt = InertialStaticPlatformTrajectory::gpsVelocityGt(t);
        Eigen::Vector3d gpsVelocityPred = inertialStaticTrajSimulator.gpsVelocity(t);

        for (int i = 0; i < 3; i++) {
            QCOMPARE(gpsVelocityPred[i],gpsVelocityGt[i]);
        }

        Eigen::Vector3d gyroGt = InertialStaticPlatformTrajectory::gyroGt(t);
        Eigen::Vector3d gyroPred = inertialStaticTrajSimulator.gyro(t);

        for (int i = 0; i < 3; i++) {
            QCOMPARE(gyroPred[i],gyroGt[i]);
        }

        Eigen::Vector3d accGt = InertialStaticPlatformTrajectory::accGt(t);
        Eigen::Vector3d accPred = inertialStaticTrajSimulator.acc(t);

        for (int i = 0; i < 3; i++) {
            QCOMPARE(accPred[i],accGt[i]);
        }

    }

    MoveNorthTrajectory* moveNorthTraj = new MoveNorthTrajectory();

    StereoVisionApp::Simulation::EcefTrajectoryInertialInstrumentsSimulator moveNorthTrajSimulator(moveNorthTraj);

    testTimes = {0.,4000000/move_north_speed,8000000/move_north_speed};

    for (double t : testTimes) {

        Eigen::Vector3d gpsGt = MoveNorthTrajectory::gpsGt(t);
        Eigen::Vector3d gpsPred = moveNorthTrajSimulator.gps(t);

        for (int i = 0; i < 3; i++) {
            QCOMPARE(gpsPred[i],gpsGt[i]);
        }

        Eigen::Vector3d gpsVelocityGt = MoveNorthTrajectory::gpsVelocityGt(t);
        Eigen::Vector3d gpsVelocityPred = moveNorthTrajSimulator.gpsVelocity(t);

        for (int i = 0; i < 3; i++) {
            QCOMPARE(gpsVelocityPred[i],gpsVelocityGt[i]);
        }

        Eigen::Vector3d gyroGt = MoveNorthTrajectory::gyroGt(t);
        Eigen::Vector3d gyroPred = moveNorthTrajSimulator.gyro(t);

        for (int i = 0; i < 3; i++) {
            QVERIFY(abs(gyroPred[i]-gyroGt[i]) < 1e-10);
        }

        Eigen::Vector3d accGt = MoveNorthTrajectory::accGt(t);
        Eigen::Vector3d accPred = moveNorthTrajSimulator.acc(t);

        for (int i = 0; i < 3; i++) {
            QCOMPARE(accPred[i],accGt[i]);
        }

    }

}
void TestTestUtils::testWayPointsEcefTrajectoryInterpolation() {

    double timeRadius = 3;
    double t0 = 0;
    double t1 = 4*timeRadius;
    double t = 2*timeRadius;
    double v = 42;

    std::vector<double> ts{t0,t1};
    std::vector<double> ys{v,v};

    double interpolated = StereoVisionApp::Simulation::WayPointsSimulatedTrajectorySmoother::evaluateSmoothedCurve(t,ys,ts,timeRadius);

    timeRadius = 3;
    t0 = 0;
    t1 = 4*timeRadius;
    t = 0;
    v = 42;

    ts = {t0,t1};
    ys = {v,v};

    interpolated = StereoVisionApp::Simulation::WayPointsSimulatedTrajectorySmoother::evaluateSmoothedCurve(t,ys,ts,timeRadius);

    QCOMPARE(interpolated, v);

    t = t1;

    interpolated = StereoVisionApp::Simulation::WayPointsSimulatedTrajectorySmoother::evaluateSmoothedCurve(t,ys,ts,timeRadius);

    QCOMPARE(interpolated, v);

    timeRadius = 2.5;
    t0 = 0;
    t1 = 50;
    t = 12;
    v = 27;

    ts = {t0,t1};
    ys = {v,v};

    interpolated = StereoVisionApp::Simulation::WayPointsSimulatedTrajectorySmoother::evaluateSmoothedCurve(t,ys,ts,timeRadius);

    QCOMPARE(interpolated, v);

    ys = {t0,t1};

    interpolated = StereoVisionApp::Simulation::WayPointsSimulatedTrajectorySmoother::evaluateSmoothedCurve(t,ys,ts,timeRadius);

    QCOMPARE(interpolated, t);

    t = t0;
    interpolated = StereoVisionApp::Simulation::WayPointsSimulatedTrajectorySmoother::evaluateSmoothedCurve(t,ys,ts,timeRadius);

    QCOMPARE(interpolated, t);

    t = t1;
    interpolated = StereoVisionApp::Simulation::WayPointsSimulatedTrajectorySmoother::evaluateSmoothedCurve(t,ys,ts,timeRadius);

    QCOMPARE(interpolated, t);

    double t025 = t-timeRadius/2;
    double t075 = t+timeRadius/2;
    ts = {t0,t025,t075,t1};
    double v0 = 33;
    double v1 = 69;
    v = (v0 + v1)/2;
    ys = {v0,v0,v1,v1};

    interpolated = StereoVisionApp::Simulation::WayPointsSimulatedTrajectorySmoother::evaluateSmoothedCurve(t,ys,ts,timeRadius);

    QCOMPARE(interpolated, v);

}
void TestTestUtils::testWayPointsEcefTrajectoryFunctor() {
    std::vector<std::array<double,3>> waypointsLatLonHeight = {
        {46.03565599595825, 7.089615022981861, 1799.531696490634},
        {46.02236597934959, 7.100801500967546, 2079.217496895602},
        {46.02146192956894, 7.098090353995183, 2151.688920219349},
        {46.03492177897298, 7.087220353061962, 1878.258324937554},
        {46.03415309208045, 7.084722826378558, 1970.722415288687},
        {46.02075179505419, 7.095398910133803, 2248.271703936763},
        {46.01940077040931, 7.092369767149918, 2135.304159701515},
        {46.03315085745567, 7.082298945193353, 2079.826442641198}
    };

    std::vector<std::array<double,3>> waypointsECEF(waypointsLatLonHeight.size());

    for (size_t i = 0; i < waypointsLatLonHeight.size(); i++) {
        waypointsECEF[i] = StereoVisionApp::Geo::WGS84Ellipsoid::LatLonHeight2ECEF(waypointsLatLonHeight[i]);
    }

    double speed = 10; // m/s
    double smoothTime = 3; // s

    StereoVisionApp::Simulation::WayPointsEcefTrajectoryFunctor trajFunctor =
        StereoVisionApp::Simulation::WayPointsEcefTrajectoryFunctor::fromWGS84WayPoints(waypointsLatLonHeight,
                                                             speed,
                                                             smoothTime);


    StereoVisionApp::Simulation::WayPointsSimulatedTrajectorySmoother::TimesWaypointsSet<double> positionWaypoints =
        trajFunctor.positionWaypoints();
    StereoVisionApp::Simulation::WayPointsSimulatedTrajectorySmoother::TimesWaypointsSet<double> orientationWaypoints =
        trajFunctor.orientationWaypoints();

    QCOMPARE(positionWaypoints.samples.size(), waypointsLatLonHeight.size());
    QCOMPARE(positionWaypoints.times.size(), positionWaypoints.samples.size());
    QCOMPARE(orientationWaypoints.times.size(), orientationWaypoints.samples.size());

    std::vector<double> const& nodeTimes = positionWaypoints.times;

    for (size_t i = 0; i < nodeTimes.size()-1; i++) {
        QVERIFY(nodeTimes[i] < nodeTimes[i+1]);
    }

    std::vector<std::array<double,3>> const& nodeSamples = positionWaypoints.samples;

    using D2Jet = StereoVisionApp::Simulation::WayPointsEcefTrajectoryFunctor::D2Jet;

    for (size_t i = 0; i < nodeTimes.size()-1; i++) {

        double midPoint = (nodeTimes[i] + nodeTimes[i+1])/2;

        D2Jet jet_t;
        jet_t.a.a = midPoint;
        jet_t.a.v[0] = 1;
        jet_t.v[0].a = 1;

        auto pose = trajFunctor.trajectory(jet_t);

        std::array<double, 3> interpolated;
        std::array<double, 3> pos;

        std::array<double, 3> orient;

        std::array<double, 3> d_pos;
        std::array<double, 3> d_orient;

        std::array<double, 3> d_pos_alt;
        std::array<double, 3> d_orient_alt;

        std::array<double, 3> d2_pos;
        std::array<double, 3> d2_orient;

        for (int j = 0; j < 3; j++) {
            interpolated[j] = (nodeSamples[i][j] + nodeSamples[i+1][j])/2;
            pos[j] = pose.t[j].a.a;
            orient[j] = pose.r[j].a.a;

            d_pos[j] = pose.t[j].v[0].a;
            d_orient[j] = pose.r[j].v[0].a;

            d_pos_alt[j] = pose.t[j].a.v[0];
            d_orient_alt[j] = pose.r[j].a.v[0];

            d2_pos[j] = pose.t[j].v[0].v[0];
            d2_orient[j] = pose.r[j].v[0].v[0];
        }

        for (int j = 0; j < 3; j++) {
            constexpr double tol = 1e-6;
            QVERIFY(std::abs(interpolated[j] - pos[j]) < tol);

            QVERIFY(std::isfinite(orient[j]));

            QVERIFY(std::isfinite(d_pos[j]));
            QVERIFY(std::isfinite(d_orient[j]));

            QCOMPARE(d_pos[j], d_pos_alt[j]);
            QCOMPARE(d_orient[j], d_orient_alt[j]);

            QVERIFY(std::isfinite(d2_pos[j]));
            QVERIFY(std::isfinite(d2_orient[j]));
        }
    }
}


void TestTestUtils::testWayPointsEcefTrajectoryFunctorInertialMeasurements() {
    std::vector<std::array<double,3>> waypointsLatLonHeightFixed = {
        {0., 0., 0.}
    };

    std::vector<std::array<double,3>> waypointsECEFFixed(waypointsLatLonHeightFixed.size());

    for (size_t i = 0; i < waypointsLatLonHeightFixed.size(); i++) {
        waypointsECEFFixed[i] = StereoVisionApp::Geo::WGS84Ellipsoid::LatLonHeight2ECEF(waypointsLatLonHeightFixed[i]);
    }

    double speed = 10; // m/s
    double smoothTime = 3; // s

    StereoVisionApp::Simulation::WayPointsEcefTrajectoryFunctor* trajFunctor = new
        StereoVisionApp::Simulation::WayPointsEcefTrajectoryFunctor(waypointsECEFFixed, speed, smoothTime);

    StereoVisionApp::Simulation::EcefTrajectoryInertialInstrumentsSimulator simulator(trajFunctor);

    double t0 = 0;
    double t1 = 1;

    StereoVision::Geometry::RigidBodyTransform<double> pose0 = simulator.body2ecef(t0);
    Eigen::Vector3d gps0 = simulator.gps(t0);
    Eigen::Vector3d gpsv0 = simulator.gpsVelocity(t0);
    Eigen::Vector3d gyro0 = simulator.gyro(t0);
    Eigen::Vector3d acc0 = simulator.acc(t0);

    StereoVision::Geometry::RigidBodyTransform<double> pose1 = simulator.body2ecef(t1);
    Eigen::Vector3d gps1 = simulator.gps(t1);
    Eigen::Vector3d gpsv1 = simulator.gpsVelocity(t1);
    Eigen::Vector3d gyro1 = simulator.gyro(t1);
    Eigen::Vector3d acc1 = simulator.acc(t1);

    Eigen::Vector3d gyroGt(0, 0, StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate);
    auto gTmp = StereoVisionApp::Geo::WGS84Ellipsoid::gravityEcefModel(waypointsECEFFixed[0]);
    Eigen::Vector3d g;
    for (int i = 0; i < 3; i++) {
        g[i] = gTmp[i];
    }

    for (int i = 0; i < 3; i++) {
        QCOMPARE(pose0.t[i], waypointsECEFFixed[0][i]);
        QCOMPARE(pose0.r[i], 0);
        QCOMPARE(gps0[i], waypointsECEFFixed[0][i]);
        QCOMPARE(gpsv0[i], 0);
        QCOMPARE(gyro0[i], gyroGt[i]);
        QCOMPARE(acc0[i], g[i]);

        QCOMPARE(pose1.t[i], waypointsECEFFixed[0][i]);
        QCOMPARE(pose1.r[i], 0);
        QCOMPARE(gps1[i], waypointsECEFFixed[0][i]);
        QCOMPARE(gpsv1[i], 0);
        QCOMPARE(gyro1[i], gyroGt[i]);
        QCOMPARE(acc1[i], g[i]);
    }
}

void TestTestUtils::testGeneratedTrajectory() {

    StereoVisionApp::GeneratedTrajectory traj;

    constexpr int nPosSteps = 12;
    constexpr int nAccSteps = 102;

    double t0 = 0;
    double tf = 10;

    double dtIns = (tf - t0)/(nAccSteps-2);
    dtIns -= dtIns/(nAccSteps+1);
    double dtPos = (tf - t0)/(nPosSteps-2);
    dtPos -= dtPos/(nPosSteps+1);

    Eigen::Vector3d x0 = Eigen::Vector3d::Random();
    Eigen::Vector3d xf = Eigen::Vector3d::Random();
    Eigen::Vector3d r0 = Eigen::Vector3d::Random();

    StereoVisionApp::GeneratedTrajectory::configureStandardNonAccelaratingTrajectory(t0,
                                                                                     tf,
                                                                                     dtIns,
                                                                                     dtPos,
                                                                                     x0,
                                                                                     xf,
                                                                                     r0,
                                                                                     &traj);


    auto acc = traj.loadAccelerationSequence();

    QVERIFY(acc.isValid());
    QCOMPARE(acc.value().nPoints(), nAccSteps);

    QVERIFY(acc.value().sequenceEndTime() >= tf);
    QVERIFY(acc.value().sequenceStartTime() <= t0);

    for (int i = 0; i < nAccSteps-1; i++) {
        QCOMPARE(acc.value()[i+1].time - acc.value()[i].time,dtIns);
    }

    for (int i = 0; i < nAccSteps; i++) {
        QCOMPARE(acc.value()[i].val.norm(),0);
    }

    auto gyro = traj.loadAngularSpeedSequence();

    QVERIFY(gyro.isValid());
    QCOMPARE(gyro.value().nPoints(), nAccSteps);

    QVERIFY(gyro.value().sequenceEndTime() >= tf);
    QVERIFY(gyro.value().sequenceStartTime() <= t0);

    for (int i = 0; i < nAccSteps-1; i++) {
        QCOMPARE(gyro.value()[i+1].time - gyro.value()[i].time,dtIns);
    }

    for (int i = 0; i < nAccSteps; i++) {
        QCOMPARE(gyro.value()[i].val.norm(),0);
    }


    auto trajData = traj.loadTrajectoryProjectLocalFrameSequence();

    QVERIFY(trajData.isValid());
    QCOMPARE(trajData.value().nPoints(), nPosSteps);

    QVERIFY(trajData.value().sequenceEndTime() >= tf);
    QVERIFY(trajData.value().sequenceStartTime() <= t0);

    for (int i = 0; i < nPosSteps-1; i++) {
        QCOMPARE(trajData.value()[i+1].time - trajData.value()[i].time,dtPos);
    }

    for (int i = 0; i < nPosSteps; i++) {
        QCOMPARE((trajData.value()[i].val.r - r0).norm(),0);
    }

    for (int i = 0; i < nPosSteps; i++) {
        double t = trajData.value()[i].time;
        QCOMPARE((trajData.value()[i].val.t - ((tf - t)*x0 + (t - t0)*xf)/(tf-t0)).norm(),0);
    }
}

QTEST_MAIN(TestTestUtils);
#include "main.moc"
