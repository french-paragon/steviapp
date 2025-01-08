#include <QtTest/QtTest>

#include <QMap>

#include "datablocks/trajectory.h"

#include <random>

class TestTrajectory : public QObject {
    Q_OBJECT
public:

private Q_SLOTS:

    void initTestCase();

    void testTrajectorySubsampling();

protected:
};


void TestTrajectory::initTestCase() {

    std::random_device rd;
    int random = rd();

    srand(random);

}

void TestTrajectory::testTrajectorySubsampling() {

    //this test just take two elements from a sequence of poses and use the guided
    //trajectory resampling function of the Trajectory class to try to reconstruct
    //the values in between (the values should be reconstructed pretty accuratly
    //since the two values are basically taken from the guide.

    using TimedBlock = StereoVisionApp::Trajectory::TimeTrajectoryBlock;
    using Transform = StereoVision::Geometry::RigidBodyTransform<double>;

    int nElements = 12;

    std::vector<TimedBlock> initial(2);
    std::vector<TimedBlock> guide(nElements);

    for (int i = 0; i < nElements; i++) {
        guide[i].time = i;
        guide[i].val.r.setRandom();
        guide[i].val.t.setRandom();
    }

    initial[0] = guide[1];
    initial[1] = guide[nElements-2];

    auto resampledOpt = StereoVisionApp::Trajectory::guidedSubsampleTrajectory(initial, guide);

    QVERIFY2(resampledOpt.isValid(), "Invalid subsampled trajectory");

    std::vector<TimedBlock> resampled = resampledOpt.value();

    QCOMPARE(resampled.size(), nElements-2);

    for (int i = 1; i < nElements-1; i++) {
        auto delta = resampled[i-1].val.inverse()*guide[i].val;

        QVERIFY2(delta.t.norm() < 1e-5, "Position alignement error");
        QVERIFY2(delta.r.norm() < 1e-5, "Rotation alignement error");
    }

    Transform rand;
    rand.r.setRandom();
    rand.t.setRandom();

    std::vector<TimedBlock> shiftedGuide = guide;

    for (int i = 0; i < guide.size(); i++) {
        shiftedGuide[i].val = rand*guide[i].val;
    }

    resampledOpt = StereoVisionApp::Trajectory::guidedSubsampleTrajectory(initial, shiftedGuide);

    QVERIFY2(resampledOpt.isValid(), "Invalid subsampled trajectory");

    resampled = resampledOpt.value();

    QCOMPARE(resampled.size(), nElements-2);

    for (int i = 1; i < nElements-1; i++) {
        auto delta = resampled[i-1].val.inverse()*guide[i].val;

        QVERIFY2(delta.t.norm() < 1e-5, "Position alignement error");
        QVERIFY2(delta.r.norm() < 1e-5, "Rotation alignement error");
    }

    rand.r.setRandom();
    rand.t.setRandom();

    std::vector<TimedBlock> linearShiftedGuide = guide;

    for (int i = 0; i < guide.size(); i++) {
        double scale = double(i - 1) / double(guide.size() - 3);

        //need to follow the geodesic in the inverse space, as this is the geodesic guidedSubsampleTrajectory will follow.
        Transform tmp = rand.inverse();
        tmp = scale*tmp;
        Transform scaled = tmp.inverse();

        linearShiftedGuide[i].val = scaled*guide[i].val;
    }

    resampledOpt = StereoVisionApp::Trajectory::guidedSubsampleTrajectory(initial, linearShiftedGuide);

    QVERIFY2(resampledOpt.isValid(), "Invalid subsampled trajectory");

    resampled = resampledOpt.value();

    QCOMPARE(resampled.size(), nElements-2);

    for (int i = 1; i < nElements-1; i++) {
        auto delta = resampled[i-1].val.inverse()*guide[i].val;

        QVERIFY2(delta.t.norm() < 1e-5, "Position alignement error");
        QVERIFY2(delta.r.norm() < 1e-5, "Rotation alignement error");
    }

}

QTEST_MAIN(TestTrajectory);
#include "main.moc"
