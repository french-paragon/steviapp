#include <QtTest/QtTest>


#include "testutils/datablocks/generatedtrajectory.h"

/*!
 * \brief The TestTestUtils class is a test to test the test utils (yes, a bit cumbersome, but better of checking the stuff you will use to test other stuff
 */
class TestTestUtils : public QObject
{
    Q_OBJECT
public:

private Q_SLOTS:

    void initTestCase();

    void testGeneratedTrajectory();

protected:

};

void TestTestUtils::initTestCase() {

    srand(time(nullptr));

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
