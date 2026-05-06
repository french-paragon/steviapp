#include <QtTest/QtTest>

#include <QMap>

#include "datablocks/trajectory.h"

#include <random>
#include <functional>

#include <Eigen/Core>
#include <QDebug>

#include "sparsesolver/costfunctors/imustepcost.h"

using TrajectoryFunction = std::function<Eigen::Vector3d(double)>; //vector as a function of time, allow to express orientation as axis angles and positions

Q_DECLARE_METATYPE(TrajectoryFunction)
Q_DECLARE_METATYPE(Eigen::Vector3d)
Q_DECLARE_METATYPE(Eigen::Matrix3d)

class TestTrajectory : public QObject {
    Q_OBJECT
public:

private Q_SLOTS:

    void initTestCase();

    void testTrajectorySubsampling();

    void testGyroIntegration_data();
    void testGyroIntegration();
    void testAccIntegration_data();
    void testAccIntegration();

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


void TestTrajectory::testGyroIntegration_data() {

    QTest::addColumn<TrajectoryFunction>("orientationTrajectory"); //body2world, as a function of time
    QTest::addColumn<double>("time");
    QTest::addColumn<double>("time_interval");

    QTest::addRow("Constant orientation") << TrajectoryFunction([] (double t) -> Eigen::Vector3d {
        return Eigen::Vector3d(0.27,0.42,0.33);
    }) << 1.0 << 0.01;

    QTest::addRow("Constant rate") << TrajectoryFunction([] (double t) -> Eigen::Vector3d {
        return Eigen::Vector3d(0.27,0.42,0.33)*t;
    }) << 1.0 << 0.01;

    QTest::addRow("Constant rate (negative)") << TrajectoryFunction([] (double t) -> Eigen::Vector3d {
        return Eigen::Vector3d(-0.27,-0.42,-0.33)*t;
    }) << 1.0 << 0.01;

}
void TestTrajectory::testGyroIntegration() {

    QFETCH(TrajectoryFunction, orientationTrajectory);
    QFETCH(double, time);
    QFETCH(double, time_interval);

    using TimeSeq = StereoVisionApp::IndexedTimeSequence<Eigen::Vector3d,double>;

    std::vector<TimeSeq::TimedElement> elements;
    int expectedN = std::ceil(time/time_interval);
    elements.reserve(expectedN+1);

    for (double t = 0; true; t += time_interval) {
        double ti = t-time_interval;
        double tf = t+time_interval;
        Eigen::Vector3d ri = orientationTrajectory(ti);
        Eigen::Vector3d rf = orientationTrajectory(tf);

        bool check = ri.allFinite();

        if (!check) {
            qWarning() << "Non finite ri in trajectory at time " << ti << ", values = [" << ri.x() << " " << ri.y() << " " << ri.z() << "]";
            QFAIL("Non finite element in trajectory data!");
        }

        check = rf.allFinite();

        if (!check) {
            qWarning() << "Non finite rf in trajectory at time " << tf << ", values = [" << rf.x() << " " << rf.y() << " " << rf.z() << "]";
            QFAIL("Non finite element in trajectory data!");
        }

        Eigen::Vector3d dr = StereoVision::Geometry::inverseRodriguezFormula<double>(
            StereoVision::Geometry::rodriguezFormula(ri).transpose()*
            StereoVision::Geometry::rodriguezFormula(rf)
            );

        check = dr.allFinite();

        if (!check) {
            qWarning() << "Non finite element in trajectory at time " << t << ", values = [" << dr.x() << " " << dr.y() << " " << dr.z() << "]";
            QFAIL("Non finite element in trajectory data!");
        }

        TimeSeq::TimedElement elem;
        elem.time = t;
        elem.val = dr/(2*time_interval);
        elements.push_back(elem);

        //qInfo() << "Element at time " << t << ": [" << elem.val.x() << " " << elem.val.y() << " " << elem.val.z() << "]";

        if (t > time) {
            break;
        }
    }

    for (int i = 0; i < elements.size(); i++) {
        TimeSeq::TimedElement& e = elements[i];
        bool check = e.val.allFinite();

        if (!check) {
            qWarning() << "Non finite element in trajectory at element " << i << ", values = [" << e.val.x() << " " << e.val.y() << " " << e.val.z() << "]";
            QFAIL("Non finite element in trajectory data!");
        }
    }

    TimeSeq seq(std::move(elements));

    /*for (int i = 0; i < seq.nPoints(); i++) {
        TimeSeq::TimedElement& e = seq[i];
        qInfo() << "Element at index " << i << ", time " << e.time << ": [" << e.val.x() << " " << e.val.y() << " " << e.val.z() << "]";
    }*/

    constexpr double t0 = 0;
    double tf = time;

    constexpr bool wBias = false;
    constexpr bool wScales = false;
    auto integratedGyro = StereoVisionApp::GyroStepCostBase::preIntegrateGyroSegment<wBias, wScales>(seq, t0, tf);

    Eigen::Vector3d dr = StereoVision::Geometry::inverseRodriguezFormula(integratedGyro.attitudeDelta);

    Eigen::Vector3d r0 = orientationTrajectory(t0);
    Eigen::Vector3d rf = orientationTrajectory(tf);

    qInfo() << "Integrated dr: [" << dr.x() << " " << dr.y() << " " << dr.z() << "]";
    qInfo() << "r0: [" << r0.x() << " " << r0.y() << " " << r0.z() << "]";
    qInfo() << "rf: [" << rf.x() << " " << rf.y() << " " << rf.z() << "]";

    Eigen::Vector3d integrationError = StereoVision::Geometry::inverseRodriguezFormula<double>(integratedGyro.attitudeDelta*(
                                           StereoVision::Geometry::rodriguezFormula(rf).transpose()*
                                           StereoVision::Geometry::rodriguezFormula(r0)));

    qInfo() << "Integration error: [" << integrationError.x() << " " << integrationError.y() << " " << integrationError.z() << "]";
    QVERIFY(integrationError.norm() < 1e-4);

}
void TestTrajectory::testAccIntegration_data() {

    QTest::addColumn<TrajectoryFunction>("orientationTrajectory"); //body2world r, as a function of time
    QTest::addColumn<TrajectoryFunction>("positionTrajectory"); //body2world t, as a function of time
    QTest::addColumn<double>("time");
    QTest::addColumn<double>("time_interval");

    QTest::addRow("Constant pose") << TrajectoryFunction([] (double t) -> Eigen::Vector3d {
        return Eigen::Vector3d(0.27,0.42,0.33);
    }) << TrajectoryFunction([] (double t) -> Eigen::Vector3d {
        return Eigen::Vector3d(0.27,0.42,0.33);
    }) << 1.0 << 0.01;

    QTest::addRow("Constant speed") << TrajectoryFunction([] (double t) -> Eigen::Vector3d {
        return Eigen::Vector3d(0.27,0.42,0.33);
    }) << TrajectoryFunction([] (double t) -> Eigen::Vector3d {
        return Eigen::Vector3d(0.27,0.42,0.33)*t;
    }) << 1.0 << 0.01;

    QTest::addRow("Constant acceleration") << TrajectoryFunction([] (double t) -> Eigen::Vector3d {
        return Eigen::Vector3d(0.27,0.42,0.33);
    }) << TrajectoryFunction([] (double t) -> Eigen::Vector3d {
        return Eigen::Vector3d(0.27,0.42,0.33)*t*t;
    }) << 1.0 << 0.01;

    QTest::addRow("Circle fixed orientation") << TrajectoryFunction([] (double t) -> Eigen::Vector3d {
        return Eigen::Vector3d(0,0,0);
    }) << TrajectoryFunction([] (double t) -> Eigen::Vector3d {
        double r = 0.42;
        return Eigen::Vector3d(std::cos(t)*r,std::sin(t)*r,0.33);
    }) << 1.0 << 0.01;

    QTest::addRow("Circle variable orientation") << TrajectoryFunction([] (double t) -> Eigen::Vector3d {
        return Eigen::Vector3d(0,0,t);
    }) << TrajectoryFunction([] (double t) -> Eigen::Vector3d {
        double r = 0.42;
        return Eigen::Vector3d(std::cos(t)*r,std::sin(t)*r,0.33);
    }) << 1.0 << 0.01;

    QTest::addRow("Circle variable orientation unsync") << TrajectoryFunction([] (double t) -> Eigen::Vector3d {
        return Eigen::Vector3d(0,0,2*t);
    }) << TrajectoryFunction([] (double t) -> Eigen::Vector3d {
        double r = 0.42;
        return Eigen::Vector3d(std::cos(t)*r,std::sin(t)*r,0.33*t);
    }) << 1.0 << 0.01;

}
void TestTrajectory::testAccIntegration() {

    QFETCH(TrajectoryFunction, orientationTrajectory);
    QFETCH(TrajectoryFunction, positionTrajectory);
    QFETCH(double, time);
    QFETCH(double, time_interval);

    using TimeSeq = StereoVisionApp::IndexedTimeSequence<Eigen::Vector3d,double>;

    std::vector<TimeSeq::TimedElement> gyroElements;
    std::vector<TimeSeq::TimedElement> accElements;

    int expectedN = std::ceil(time/time_interval);

    gyroElements.reserve(expectedN+1);
    accElements.reserve(expectedN+1);

    for (double t = 0; true; t += time_interval) {
        double ti = t-time_interval;
        double tf = t+time_interval;
        Eigen::Vector3d ri = orientationTrajectory(ti);
        Eigen::Vector3d r0 = orientationTrajectory(t);
        Eigen::Vector3d rf = orientationTrajectory(tf);

        bool check = ri.allFinite();

        if (!check) {
            qWarning() << "Non finite ri in trajectory at time " << ti << ", values = [" << ri.x() << " " << ri.y() << " " << ri.z() << "]";
            QFAIL("Non finite element in trajectory data!");
        }

        check = rf.allFinite();

        if (!check) {
            qWarning() << "Non finite rf in trajectory at time " << tf << ", values = [" << rf.x() << " " << rf.y() << " " << rf.z() << "]";
            QFAIL("Non finite element in trajectory data!");
        }

        Eigen::Vector3d dr = StereoVision::Geometry::inverseRodriguezFormula<double>(
            StereoVision::Geometry::rodriguezFormula(ri).transpose()*
            StereoVision::Geometry::rodriguezFormula(rf)
            );

        check = dr.allFinite();

        if (!check) {
            qWarning() << "Non finite element in trajectory at time " << t << ", values = [" << dr.x() << " " << dr.y() << " " << dr.z() << "]";
            QFAIL("Non finite element in trajectory data!");
        }

        TimeSeq::TimedElement gyroElem;
        gyroElem.time = t;
        gyroElem.val = dr/(2*time_interval);
        gyroElements.push_back(gyroElem);

        //qInfo() << "Gyro element at time " << t << ": [" << gyroElem.val.x() << " " << gyroElem.val.y() << " " << gyroElem.val.z() << "]";

        Eigen::Vector3d xi = positionTrajectory(ti);
        Eigen::Vector3d x0 = positionTrajectory(t);
        Eigen::Vector3d xf = positionTrajectory(tf);

        Eigen::Vector3d acc_inertial = (xf-2*x0+xi)/(time_interval*time_interval);
        Eigen::Vector3d acc_body = StereoVision::Geometry::angleAxisRotate<double>(-r0,acc_inertial);

        TimeSeq::TimedElement accElem;
        accElem.time = t;
        accElem.val = acc_body;
        accElements.push_back(accElem);

        //qInfo() << "Acc element at time " << t << ": [" << accElem.val.x() << " " << accElem.val.y() << " " << accElem.val.z() << "]";

        if (t > time) {
            break;
        }

    }

    for (int i = 0; i < gyroElements.size(); i++) {
        TimeSeq::TimedElement& e = gyroElements[i];
        bool check = e.val.allFinite();

        if (!check) {
            qWarning() << "Non finite gyro element in trajectory at element " << i << ", values = [" << e.val.x() << " " << e.val.y() << " " << e.val.z() << "]";
            QFAIL("Non finite element in trajectory data!");
        }
    }

    for (int i = 0; i < accElements.size(); i++) {
        TimeSeq::TimedElement& e = accElements[i];
        bool check = e.val.allFinite();

        if (!check) {
            qWarning() << "Non finite acc element in trajectory at element " << i << ", values = [" << e.val.x() << " " << e.val.y() << " " << e.val.z() << "]";
            QFAIL("Non finite element in trajectory data!");
        }
    }

    TimeSeq gyroSeq(std::move(gyroElements));
    TimeSeq accSeq(std::move(accElements));

    constexpr double t0 = 0;
    double tm = time/2;
    double tf = time;

    constexpr bool wBias = false;
    constexpr bool wScales = false;
    auto integratedAcc = StereoVisionApp::AccelerometerStepCostBase::preIntegrateAccSegment(gyroSeq,accSeq,t0,tm,tf);

    Eigen::Vector3d dv = integratedAcc.speedDelta;

    Eigen::Vector3d rm = orientationTrajectory(tm);

    Eigen::Vector3d x0 = positionTrajectory(t0);
    Eigen::Vector3d xm = positionTrajectory(tm);
    Eigen::Vector3d xf = positionTrajectory(tf);

    Eigen::Vector3d v0 = (xm-x0)/tm;
    Eigen::Vector3d vf = (xf-xm)/tm;

    Eigen::Vector3d dv_expected = StereoVision::Geometry::angleAxisRotate<double>(-rm,vf - v0);

    Eigen::Vector3d error = dv_expected - dv;

    qInfo() << "delta speed: [" << dv.x() << " " << dv.y() << " " << dv.z() << "]";

    qInfo() << "delta speed expected: [" << dv_expected.x() << " " << dv_expected.y() << " " << dv_expected.z() << "]";

    qInfo() << "Integration error: [" << error.x() << " " << error.y() << " " << error.z() << "]";
    QVERIFY(error.norm() < 1e-4);

}

QTEST_MAIN(TestTrajectory);
#include "main.moc"
