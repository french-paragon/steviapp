#include <QtTest/QtTest>


#include "testutils/datablocks/generatedtrajectory.h"
#include "testutils/eceftrajectoryinertialinstrumentssimulator.h"

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
     * \brief The StaticPlatformTrajectory class represent a
     */
    class StaticPlatformTrajectory : public StereoVisionApp::Simulation::EcefTrajectoryFunctor {
    public:
        virtual StereoVision::Geometry::RigidBodyTransform<ceres::Jet<ceres::Jet<double,1>,1>> trajectory(ceres::Jet<ceres::Jet<double,1>,1> const& t) const override {
            ceres::Jet<ceres::Jet<double,1>,1> zero;
            zero.a.a = 0;
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
