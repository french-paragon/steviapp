#include <iostream>

#include "sparsesolver/costfunctors/imustepcost.h"
#include "vision/indexed_timed_sequence.h"

#include <random>

#include <QTest>

using Timeline = StereoVisionApp::IndexedTimeSequence<Eigen::Vector3d, double>;

using TrajectoryFunction = std::function<Eigen::Vector3d(double)>; //vector as a function of time, allow to express orientation as axis angles and positions

Q_DECLARE_METATYPE(Eigen::Vector3f)
Q_DECLARE_METATYPE(Eigen::Vector2f)
Q_DECLARE_METATYPE(Eigen::Vector3d)
Q_DECLARE_METATYPE(Eigen::Vector2d)
Q_DECLARE_METATYPE(TrajectoryFunction)

void runGyroExperiment(
        Eigen::Vector3d const& totalAxisInitial,
        Eigen::Vector3d const& totalAxisFinal,
        Eigen::Vector3d const& stepVariability,
        int nSteps,
        Eigen::Vector3d const& gains,
        Eigen::Vector3d const& bias,
        std::default_random_engine & randomEngine) {

    std::normal_distribution<double> distribution(0,1);

    Eigen::Vector3d inv_gains;
    Eigen::Vector3d inv_bias;

    for (int i = 0; i < 3; i++) {
        inv_gains[i] = 1/gains[i];
        inv_bias[i] = -inv_gains[i]*bias[i];
    }

    double s = 1.0/(nSteps-1);
    std::vector<Timeline::TimedElement> elements(nSteps);
    std::vector<Timeline::TimedElement> elementsNoisy(nSteps);
    std::vector<Timeline::TimedElement> elementsNoisyNoBiasGains(nSteps);

    for (int i = 0; i < nSteps; i++) {

        double si = double(nSteps-1-i)/(nSteps-1) * s;
        double sf = double(i)/(nSteps-1) * s;

        Eigen::Vector3d step = si*totalAxisInitial + sf*totalAxisFinal;
        Eigen::Vector3d step_noisy = step;
        Eigen::Vector3d step_noisy_no_bias = step;

        for (int i = 0; i < 3; i++) {
            step_noisy[i] *= gains[i];
            step_noisy[i] += bias[i];

            double pureNoise = stepVariability[i]*distribution(randomEngine);
            step_noisy[i] += pureNoise;
            step_noisy_no_bias[i] += pureNoise;
        }

        elements[i].val = step;
        elements[i].time = i;

        elementsNoisy[i].val = step_noisy;
        elementsNoisy[i].time = i;

        elementsNoisyNoBiasGains[i].val = step_noisy_no_bias;
        elementsNoisyNoBiasGains[i].time = i;
    }

    Timeline timeline(elements);
    Timeline timelineNoisy(elementsNoisy);
    Timeline timelineNoisyNoBias(elementsNoisyNoBiasGains);
    double t0 = 0;
    double t1 = nSteps-1;

    StereoVisionApp::GyroStepCostBase::IntegratedGyroSegment integrated =
            StereoVisionApp::GyroStepCostBase::preIntegrateGyroSegment<true, true>(timeline, t0, t1);

    StereoVisionApp::GyroStepCostBase::IntegratedGyroSegment integratedNoisy =
            StereoVisionApp::GyroStepCostBase::preIntegrateGyroSegment<true, true>(timelineNoisy, t0, t1);

    StereoVisionApp::GyroStepCostBase::IntegratedGyroSegment integratedNoisyNoBias =
            StereoVisionApp::GyroStepCostBase::preIntegrateGyroSegment<true, true>(timelineNoisyNoBias, t0, t1);

    Eigen::Matrix3d& expected = integrated.attitudeDelta;

    Eigen::Vector3d delta = integratedNoisy.gainJacobian * (inv_gains-Eigen::Vector3d::Ones()) + integratedNoisy.biasJacobian * inv_bias;
    Eigen::Matrix3d computed = integratedNoisy.attitudeDelta *
            StereoVision::Geometry::rodriguezFormula(delta);

    Eigen::Matrix3d diff_raw = expected*integratedNoisy.attitudeDelta.transpose();
    Eigen::Matrix3d diff_no_bias = expected*integratedNoisyNoBias.attitudeDelta.transpose();
    Eigen::Matrix3d diff = expected*computed.transpose();
    Eigen::Matrix3d diffWRTNoBias = integratedNoisyNoBias.attitudeDelta*computed.transpose();

    Eigen::Vector3d axis_gt = StereoVision::Geometry::inverseRodriguezFormula(integrated.attitudeDelta);
    Eigen::Vector3d axis_no_bias = StereoVision::Geometry::inverseRodriguezFormula(integratedNoisyNoBias.attitudeDelta);
    Eigen::Vector3d axis_raw = StereoVision::Geometry::inverseRodriguezFormula(integratedNoisy.attitudeDelta);
    Eigen::Vector3d axis_computed = StereoVision::Geometry::inverseRodriguezFormula(computed);
    Eigen::Vector3d axis_diff_raw = StereoVision::Geometry::inverseRodriguezFormula(diff_raw);
    Eigen::Vector3d axis_diff_no_bias = StereoVision::Geometry::inverseRodriguezFormula(diff_no_bias);
    Eigen::Vector3d axis_diff = StereoVision::Geometry::inverseRodriguezFormula(diff);
    Eigen::Vector3d axis_diff_wrt_no_bias = StereoVision::Geometry::inverseRodriguezFormula(diffWRTNoBias);

    std::cout << "Gyro pre-integration with gains and bias: axis angle = " << axis_gt[0] << " " << axis_gt[1] << " " << axis_gt[2];
    std::cout << " gains = " << gains[0] << " " << gains[1] << " " << gains[2];
    std::cout << " bias = " << bias[0] << " " << bias[1] << " " << bias[2];
    std::cout << " std_dev = " << stepVariability[0] << " " << stepVariability[1] << " " << stepVariability[2] << "\n";

    std::cout << "\tComputed (raw) = " << axis_raw[0] << " " << axis_raw[1] << " " << axis_raw[2];
    std::cout << " Error = " << axis_diff_raw[0] << " " << axis_diff_raw[1] << " " << axis_diff_raw[2] << "\n";

    std::cout << "\tComputed (no bias) = " << axis_no_bias[0] << " " << axis_no_bias[1] << " " << axis_no_bias[2];
    std::cout << " Error = " << axis_diff_no_bias[0] << " " << axis_diff_no_bias[1] << " " << axis_diff_no_bias[2] << "\n";

    std::cout << "\tComputed (corrected) = " << axis_computed[0] << " " << axis_computed[1] << " " << axis_computed[2];
    std::cout << " Error = " << axis_diff[0] << " " << axis_diff[1] << " " << axis_diff[2];
    std::cout << " Error w.r.t no bias = " << axis_diff_wrt_no_bias[0] << " " << axis_diff_wrt_no_bias[1] << " " << axis_diff_wrt_no_bias[2] << "\n";

    std::cout.flush();
}

void runAccelerometerExperiment(Eigen::Vector3d acc0,
                                Eigen::Vector3d acc1,
                                Eigen::Vector3d const& accVariability,
                                Eigen::Vector3d const& totalAxisInitial,
                                Eigen::Vector3d const& totalAxisFinal,
                                Eigen::Vector3d const& gyroVariability,
                                int pSteps,
                                Eigen::Vector3d const& acc_gains,
                                Eigen::Vector3d const& acc_bias,
                                Eigen::Vector3d const& gyro_gains,
                                Eigen::Vector3d const& gyro_bias,
                                std::default_random_engine & randomEngine) {


    std::normal_distribution<double> distribution(0,1);

    Eigen::Vector3d acc_inv_gains;
    Eigen::Vector3d acc_inv_bias;
    Eigen::Vector3d gyro_inv_gains;
    Eigen::Vector3d gyro_inv_bias;

    for (int i = 0; i < 3; i++) {
        acc_inv_gains[i] = 1/acc_gains[i];
        acc_inv_bias[i] = -acc_inv_gains[i]*acc_bias[i];
        gyro_inv_gains[i] = 1/gyro_gains[i];
        gyro_inv_bias[i] = -gyro_inv_gains[i]*gyro_bias[i];
    }

    int nSteps = 2*pSteps;

    double s = 1.0/(nSteps-1);
    std::vector<Timeline::TimedElement> elements(nSteps);
    std::vector<Timeline::TimedElement> elementsNoisy(nSteps);
    std::vector<Timeline::TimedElement> elementsNoisyNoBiasGains(nSteps);

    std::vector<Timeline::TimedElement> elements_gyro(nSteps);
    std::vector<Timeline::TimedElement> elements_gyroNoisy(nSteps);
    std::vector<Timeline::TimedElement> elements_gyroNoisyNoBiasGains(nSteps);

    for (int i = 0; i < nSteps; i++) {

        double si = double(nSteps-1-i)/(nSteps-1) * s;
        double sf = double(i)/(nSteps-1) * s;

        Eigen::Vector3d step_gyro = si*totalAxisInitial + sf*totalAxisFinal;
        Eigen::Vector3d step_gyro_noisy = step_gyro;
        Eigen::Vector3d step_gyro_noisy_no_bias = step_gyro;

        Eigen::Vector3d step = si*acc0 + sf*acc1;
        Eigen::Vector3d step_noisy = step;
        Eigen::Vector3d step_noisy_no_bias = step;

        for (int i = 0; i < 3; i++) {
            step_noisy[i] *= acc_gains[i];
            step_noisy[i] += acc_bias[i];

            double pureNoise = accVariability[i]*distribution(randomEngine);
            step_noisy[i] += pureNoise;
            step_noisy_no_bias[i] += pureNoise;

            step_gyro_noisy[i] *= gyro_gains[i];
            step_gyro_noisy[i] += gyro_bias[i];

            pureNoise = gyroVariability[i]*distribution(randomEngine);
            step_gyro_noisy[i] += pureNoise;
            step_gyro_noisy_no_bias[i] += pureNoise;
        }

        elements[i].val = step;
        elements[i].time = i;

        elementsNoisy[i].val = step_noisy;
        elementsNoisy[i].time = i;

        elementsNoisyNoBiasGains[i].val = step_noisy_no_bias;
        elementsNoisyNoBiasGains[i].time = i;

        elements_gyro[i].val = step_gyro;
        elements_gyro[i].time = i;

        elements_gyroNoisy[i].val = step_gyro_noisy;
        elements_gyroNoisy[i].time = i;

        elements_gyroNoisyNoBiasGains[i].val = step_gyro_noisy_no_bias;
        elements_gyroNoisyNoBiasGains[i].time = i;
    }

    Timeline timeline(elements);
    Timeline timelineNoisy(elementsNoisy);
    Timeline timelineNoisyNoBias(elementsNoisyNoBiasGains);

    Timeline timeline_gyro(elements_gyro);
    Timeline timeline_gyroNoisy(elements_gyroNoisy);
    Timeline timeline_gyroNoisyNoBias(elements_gyroNoisyNoBiasGains);

    double t0 = 0;
    double t2 = nSteps-1;
    double t1 = t2/2;

    StereoVisionApp::AccelerometerStepCostBase::IntegratedAccSegment integrated =
            StereoVisionApp::AccelerometerStepCostBase::preIntegrateAccSegment(timeline_gyro,
                                                                               timeline,
                                                                               t0, t1, t2);

    StereoVisionApp::AccelerometerStepCostBase::IntegratedAccSegment integratedNoisy =
            StereoVisionApp::AccelerometerStepCostBase::preIntegrateAccSegment(timeline_gyroNoisy,
                                                                               timelineNoisy,
                                                                               t0, t1, t2);

    StereoVisionApp::AccelerometerStepCostBase::IntegratedAccSegment integratedNoisyNoBias =
            StereoVisionApp::AccelerometerStepCostBase::preIntegrateAccSegment(timeline_gyroNoisyNoBias,
                                                                               timelineNoisyNoBias,
                                                                               t0, t1, t2);

    Eigen::Vector3d delta = integratedNoisy.accGainJacobian * (acc_inv_gains-Eigen::Vector3d::Ones())
            + integratedNoisy.accBiasJacobian * acc_inv_bias
            + integratedNoisy.gyroGainJacobian * (gyro_inv_gains-Eigen::Vector3d::Ones())
            + integratedNoisy.gyroBiasJacobian * gyro_inv_bias;

    Eigen::Vector3d speedDelta_gt = integrated.speedDelta;
    Eigen::Vector3d speedDelta_no_bias = integratedNoisyNoBias.speedDelta;
    Eigen::Vector3d speedDelta_raw = integratedNoisy.speedDelta;
    Eigen::Vector3d speedDelta_computed = speedDelta_raw + delta;
    Eigen::Vector3d diff_raw = speedDelta_raw - speedDelta_gt;
    Eigen::Vector3d diff_no_bias = speedDelta_no_bias - speedDelta_gt;
    Eigen::Vector3d diff = speedDelta_computed - speedDelta_gt;
    Eigen::Vector3d diff_wrt_no_bias = speedDelta_computed - speedDelta_no_bias;

    std::cout << "Accelerometer pre-integration with gains and bias: speed delta = " << speedDelta_gt[0] << " " << speedDelta_gt[1] << " " << speedDelta_gt[2];
    std::cout << " gains accelerometer = " << acc_gains[0] << " " << acc_gains[1] << " " << acc_gains[2];
    std::cout << " bias accelerometer = " << acc_bias[0] << " " << acc_bias[1] << " " << acc_bias[2];
    std::cout << " std_dev accelerometer = " << accVariability[0] << " " << accVariability[1] << " " << accVariability[2] << "\n";
    std::cout << " gains gyro = " << gyro_gains[0] << " " << gyro_gains[1] << " " << gyro_gains[2];
    std::cout << " bias gyro = " << gyro_bias[0] << " " << gyro_bias[1] << " " << gyro_bias[2];
    std::cout << " std_dev gyro = " << gyroVariability[0] << " " << gyroVariability[1] << " " << gyroVariability[2] << "\n";

    std::cout << "\tComputed (raw) = " << speedDelta_raw[0] << " " << speedDelta_raw[1] << " " << speedDelta_raw[2];
    std::cout << " Error = " << diff_raw[0] << " " << diff_raw[1] << " " << diff_raw[2] << "\n";

    std::cout << "\tComputed (no bias) = " << speedDelta_no_bias[0] << " " << speedDelta_no_bias[1] << " " << speedDelta_no_bias[2];
    std::cout << " Error = " << diff_no_bias[0] << " " << diff_no_bias[1] << " " << diff_no_bias[2] << "\n";

    std::cout << "\tComputed (corrected) = " << speedDelta_computed[0] << " " << speedDelta_computed[1] << " " << speedDelta_computed[2];
    std::cout << " Error = " << diff[0] << " " << diff[1] << " " << diff[2];
    std::cout << " Error w.r.t no bias = " << diff_wrt_no_bias[0] << " " << diff_wrt_no_bias[1] << " " << diff_wrt_no_bias[2] << "\n";

    std::cout.flush();

}

/*int main(int argc, char** argv) {

    std::default_random_engine re;
    re.seed(std::random_device()());

    std::uniform_real_distribution<double> distributionGyro(-M_PI,M_PI);
    std::uniform_real_distribution<double> distributionAcc(-1,1);

    Eigen::Vector3d axisInitial;
    Eigen::Vector3d axisFinal;

    for (int i = 0; i < 3; i++) {
        axisInitial[i] = distributionGyro(re);
        axisFinal[i] = distributionGyro(re);
    }

    Eigen::Vector3d accInitial;
    Eigen::Vector3d accFinal;

    for (int i = 0; i < 3; i++) {
        accInitial[i] = distributionAcc(re);
        accFinal[i] = distributionAcc(re);
    }

    Eigen::Vector3d stepStdGyro(0.005,0.005,0.005);
    Eigen::Vector3d stepStdAcc(0.005,0.005,0.005);
    int nSteps = 10;

    Eigen::Vector3d gainsGyro(1.01,1.01,1.01);
    Eigen::Vector3d biasGyro(0.01,0.01,0.01);

    Eigen::Vector3d gainsAcc(1.01,1.01,1.01);
    Eigen::Vector3d biasAcc(0.01,0.01,0.01);

    runGyroExperiment(axisInitial, axisFinal, stepStdGyro, nSteps, gainsGyro, biasGyro, re);

    runAccelerometerExperiment(accInitial, accFinal, stepStdAcc,
                               axisInitial, axisFinal, stepStdGyro,
                               nSteps,
                               gainsAcc, biasAcc,
                               gainsGyro, biasGyro,
                               re);

}*/

class INSPreintegrationBenchmark : public QObject {
    Q_OBJECT
public:

private Q_SLOTS:

    void initTestCase();

    void benchmarkLinearizeGyroExp_data();
    void benchmarkLinearizeGyroExp();

    void benchmarkLinearizeJacobianGyroExp_data();
    void benchmarkLinearizeJacobianGyroExp();

    void benchmarkGyro_data();
    void benchmarkGyro();

    void benchmarkAcc_data();
    void benchmarkAcc();

private:

    std::default_random_engine _re;
};

void INSPreintegrationBenchmark::initTestCase() {
    _re.seed(4269); //fixed seed for consistency
}


void INSPreintegrationBenchmark::benchmarkLinearizeGyroExp_data() {

    QTest::addColumn<double>("angle_delta");

    QTest::newRow("Delta 1e-6") << 1e-6;
    QTest::newRow("Delta 1e-5") << 1e-5;
    QTest::newRow("Delta 1e-4") << 1e-4;
    QTest::newRow("Delta 1e-3") << 1e-3;
    QTest::newRow("Delta 1e-2") << 1e-2;
    QTest::newRow("Delta 1e-1") << 1e-1;

}
void INSPreintegrationBenchmark::benchmarkLinearizeGyroExp() {

    QFETCH(double, angle_delta);

    std::uniform_real_distribution<double> distributionGyro(-M_PI,M_PI);
    std::normal_distribution<double> distribution(0,1);

    Eigen::Vector3d axisInitial;
    Eigen::Vector3d axisDelta;

    for (int i = 0; i < 3; i++) {
        axisInitial[i] = distributionGyro(_re);
        axisDelta[i] = distributionGyro(_re);
    }

    axisDelta.normalize();
    axisDelta *= angle_delta;

    Eigen::Matrix3d J = StereoVision::Geometry::diffRodriguezLieAlgebra(axisInitial);

    Eigen::Matrix3d composed = StereoVision::Geometry::rodriguezFormula<double>(axisInitial+axisDelta);
    Eigen::Matrix3d linearized = StereoVision::Geometry::rodriguezFormula<double>(axisInitial)*StereoVision::Geometry::rodriguezFormula<double>(J*axisDelta);

    Eigen::Vector3d error = StereoVision::Geometry::inverseRodriguezFormula<double>(composed.transpose()*linearized);

    qInfo() << "Error = " << error[0] << " " << error[1] << " " << error[2];

    double thresh = angle_delta*angle_delta;
    QVERIFY(error.norm() < thresh);
}


void INSPreintegrationBenchmark::benchmarkLinearizeJacobianGyroExp_data() {

    QTest::addColumn<int>("nSteps");
    QTest::addColumn<double>("angle_incr");
    QTest::addColumn<double>("gain");
    QTest::addColumn<double>("bias");

    QTest::newRow("1 steps, Incr 1deg, Gain 1, Bias 1e-6") << 2 << (1./180*M_PI) << 1. << 1e-6;
    QTest::newRow("1 steps, Incr 1deg, Gain 1, Bias 1e-5") << 2 << (1./180*M_PI) << 1. << 1e-5;
    QTest::newRow("1 steps, Incr 1deg, Gain 1, Bias 1e-4") << 2 << (1./180*M_PI) << 1. << 1e-4;
    QTest::newRow("1 steps, Incr 1deg, Gain 1, Bias 1e-3") << 2 << (1./180*M_PI) << 1. << 1e-3;
    QTest::newRow("1 steps, Incr 1deg, Gain 1, Bias 1e-2") << 2 << (1./180*M_PI) << 1. << 1e-2;
    QTest::newRow("1 steps, Incr 1deg, Gain 1, Bias 1e-1") << 2 << (1./180*M_PI) << 1. << 1e-1;

    QTest::newRow("5 steps, Incr 1deg, Gain 1, Bias 1e-6") << 6 << (1./180*M_PI) << 1. << 1e-6;
    QTest::newRow("5 steps, Incr 1deg, Gain 1, Bias 1e-5") << 6 << (1./180*M_PI) << 1. << 1e-5;
    QTest::newRow("5 steps, Incr 1deg, Gain 1, Bias 1e-4") << 6 << (1./180*M_PI) << 1. << 1e-4;
    QTest::newRow("5 steps, Incr 1deg, Gain 1, Bias 1e-3") << 6 << (1./180*M_PI) << 1. << 1e-3;
    QTest::newRow("5 steps, Incr 1deg, Gain 1, Bias 1e-2") << 6 << (1./180*M_PI) << 1. << 1e-2;
    QTest::newRow("5 steps, Incr 1deg, Gain 1, Bias 1e-1") << 6 << (1./180*M_PI) << 1. << 1e-1;

}
void INSPreintegrationBenchmark::benchmarkLinearizeJacobianGyroExp() {

    QFETCH(int, nSteps);
    QFETCH(double, angle_incr);
    QFETCH(double, gain);
    QFETCH(double, bias);

    std::uniform_real_distribution<double> distributionGyro(-M_PI,M_PI);
    std::normal_distribution<double> distribution(0,1);

    Eigen::Vector3d rprev;
    Eigen::Vector3d r;
    Eigen::Vector3d biasVec;
    Eigen::Vector3d gainVec;

    for (int i = 0; i < 3; i++) {
        r[i] = distributionGyro(_re);
        biasVec[i] = distributionGyro(_re);
        gainVec[i] = gain-1;
    }

    r.normalize();
    r *= angle_incr;

    biasVec.normalize();
    biasVec *= bias;

    Eigen::Matrix3d JGainSum = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d JBiasSum = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d RAccumulated = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d RAccumulatedBiased = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d rightDeltaLinearized = Eigen::Matrix3d::Identity();

    for (int i = 0; i < nSteps; i++) {
        Eigen::Matrix3d J = StereoVision::Geometry::diffRodriguezLieAlgebra(r);

        Eigen::Matrix3d R = StereoVision::Geometry::rodriguezFormula<double>(r);

        rightDeltaLinearized = StereoVision::Geometry::rodriguezFormula<double>(R.transpose()*StereoVision::Geometry::inverseRodriguezFormula<double>(rightDeltaLinearized))*
                               StereoVision::Geometry::rodriguezFormula<double>(J*((gain-1)*r + biasVec));

        JGainSum = R.transpose()*JGainSum + J*Eigen::DiagonalMatrix<double, 3>(r[0], r[1], r[2]);
        JBiasSum = R.transpose()*JBiasSum + J;

        RAccumulated = RAccumulated*R;
        RAccumulatedBiased = RAccumulatedBiased*StereoVision::Geometry::rodriguezFormula<double>(gain*r + biasVec);

        rprev = r;
        for (int i = 0; i < 3; i++) {
            r[i] = distributionGyro(_re);
        }

        r.normalize();
        r *= angle_incr;
        //r = 0.9*r + 0.1*rprev;
    }

    Eigen::Matrix3d rightDelta = RAccumulated.transpose()*RAccumulatedBiased;
    Eigen::Matrix3d rightDeltaLinearizedSummed = StereoVision::Geometry::rodriguezFormula<double>(JGainSum*gainVec + JBiasSum*biasVec);

    Eigen::Vector3d errorLinearized = StereoVision::Geometry::inverseRodriguezFormula<double>(rightDeltaLinearized.transpose()*rightDelta);
    Eigen::Vector3d errorLinearizedSummed = StereoVision::Geometry::inverseRodriguezFormula<double>(rightDeltaLinearizedSummed.transpose()*rightDelta);

    qInfo() << "Error Linearized = " << errorLinearized[0] << " " << errorLinearized[1] << " " << errorLinearized[2];
    qInfo() << "Error Linearized Summed = " << errorLinearizedSummed[0] << " " << errorLinearizedSummed[1] << " " << errorLinearizedSummed[2];

}

void INSPreintegrationBenchmark::benchmarkGyro_data() {

    QTest::addColumn<double>("angle_delta");
    QTest::addColumn<int>("nSteps");
    QTest::addColumn<Eigen::Vector3d>("axisSigmas");
    QTest::addColumn<Eigen::Vector3d>("gains");
    QTest::addColumn<Eigen::Vector3d>("bias");

    QTest::newRow("Angle 1deg, 10 Steps, Noise 0, Gain 0, Bias 1e-5") << (1./180*M_PI) << 10 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1,1,1) << Eigen::Vector3d(1e-5,1e-5,1e-5);
    QTest::newRow("Angle 5deg, 10 Steps, Noise 0, Gain 0, Bias 1e-5") << (5./180*M_PI) << 10 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1,1,1) << Eigen::Vector3d(1e-5,1e-5,1e-5);
    QTest::newRow("Angle 10deg, 10 Steps, Noise 0, Gain 0, Bias 1e-5") << (10./180*M_PI) << 10 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1,1,1) << Eigen::Vector3d(1e-5,1e-5,1e-5);

    QTest::newRow("Angle 1deg, 10 Steps, Noise 0, Gain 0, Bias 1e-3") << (1./180*M_PI) << 10 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1,1,1) << Eigen::Vector3d(1e-3,1e-3,1e-3);
    QTest::newRow("Angle 5deg, 10 Steps, Noise 0, Gain 0, Bias 1e-3") << (5./180*M_PI) << 10 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1,1,1) << Eigen::Vector3d(1e-3,1e-3,1e-3);
    QTest::newRow("Angle 10deg, 10 Steps, Noise 0, Gain 0, Bias 1e-3") << (10./180*M_PI) << 10 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1,1,1) << Eigen::Vector3d(1e-3,1e-3,1e-3);


}
void INSPreintegrationBenchmark::benchmarkGyro() {

    QFETCH(double, angle_delta);
    QFETCH(int, nSteps);
    QFETCH(Eigen::Vector3d, axisSigmas);
    QFETCH(Eigen::Vector3d, gains);
    QFETCH(Eigen::Vector3d, bias);

    std::uniform_real_distribution<double> distributionGyro(-M_PI,M_PI);
    std::normal_distribution<double> distribution(0,1);

    Eigen::Vector3d axisInitial;
    Eigen::Vector3d axisDelta;
    Eigen::Vector3d axisFinal;

    for (int i = 0; i < 3; i++) {
        axisInitial[i] = distributionGyro(_re);
        axisDelta[i] = distributionGyro(_re);
    }

    axisDelta.normalize();
    axisDelta *= angle_delta;

    axisFinal = StereoVision::Geometry::angleAxisRotate(axisDelta, axisInitial);

    Eigen::Vector3d inv_gains;
    Eigen::Vector3d inv_bias;

    for (int i = 0; i < 3; i++) {
        inv_gains[i] = 1/gains[i];
        inv_bias[i] = -inv_gains[i]*bias[i];
    }

    QVERIFY(inv_gains.allFinite());
    QVERIFY(inv_bias.allFinite());

    double s = 1.0/(nSteps-1);
    std::vector<Timeline::TimedElement> elements(nSteps);
    std::vector<Timeline::TimedElement> elementsNoisy(nSteps);
    std::vector<Timeline::TimedElement> elementsNoisyNoBiasGains(nSteps);

    for (int i = 0; i < nSteps; i++) {

        double si = double(nSteps-1-i)/(nSteps-1) * s;
        double sf = double(i)/(nSteps-1) * s;

        Eigen::Vector3d step = si*axisInitial + sf*axisFinal;
        Eigen::Vector3d step_noisy = step;
        Eigen::Vector3d step_noisy_no_bias = step;

        for (int i = 0; i < 3; i++) {
            step_noisy[i] *= gains[i];
            step_noisy[i] += bias[i];

            double pureNoise = axisSigmas[i]*distribution(_re);
            step_noisy[i] += pureNoise;
            step_noisy_no_bias[i] += pureNoise;
        }

        elements[i].val = step;
        elements[i].time = i;

        elementsNoisy[i].val = step_noisy;
        elementsNoisy[i].time = i;

        elementsNoisyNoBiasGains[i].val = step_noisy_no_bias;
        elementsNoisyNoBiasGains[i].time = i;
    }

    Timeline timeline(elements);
    Timeline timelineNoisy(elementsNoisy);
    Timeline timelineNoisyNoBias(elementsNoisyNoBiasGains);
    double t0 = 0;
    double t1 = nSteps-1;

    StereoVisionApp::GyroStepCostBase::IntegratedGyroSegment integrated;

    integrated = StereoVisionApp::GyroStepCostBase::preIntegrateGyroSegment<true, true>(timeline, t0, t1);

    StereoVisionApp::GyroStepCostBase::IntegratedGyroSegment integratedNoisy =
        StereoVisionApp::GyroStepCostBase::preIntegrateGyroSegment<true, true>(timelineNoisy, t0, t1);

    StereoVisionApp::GyroStepCostBase::IntegratedGyroSegment integratedNoisyNoBias =
        StereoVisionApp::GyroStepCostBase::preIntegrateGyroSegment<true, true>(timelineNoisyNoBias, t0, t1);

    QVERIFY(integrated.attitudeDelta.allFinite());
    QVERIFY(integrated.biasJacobian.allFinite());
    QVERIFY(integrated.gainJacobian.allFinite());

    QVERIFY(integratedNoisy.attitudeDelta.allFinite());
    QVERIFY(integratedNoisy.biasJacobian.allFinite());
    QVERIFY(integratedNoisy.gainJacobian.allFinite());

    QVERIFY(integratedNoisyNoBias.attitudeDelta.allFinite());
    QVERIFY(integratedNoisyNoBias.biasJacobian.allFinite());
    QVERIFY(integratedNoisyNoBias.gainJacobian.allFinite());

    Eigen::Matrix3d& expected = integrated.attitudeDelta;

    Eigen::Vector3d delta = integratedNoisy.gainJacobian * (inv_gains-Eigen::Vector3d::Ones()) + integratedNoisy.biasJacobian * inv_bias;
    Eigen::Matrix3d computed = integratedNoisy.attitudeDelta *
                                   StereoVision::Geometry::rodriguezFormula(delta);

    Eigen::Matrix3d diff_raw = expected*integratedNoisy.attitudeDelta.transpose();
    Eigen::Matrix3d diff_no_bias = expected*integratedNoisyNoBias.attitudeDelta.transpose();
    Eigen::Matrix3d diff = expected*computed.transpose();
    Eigen::Matrix3d diffWRTNoBias = integratedNoisyNoBias.attitudeDelta*computed.transpose();

    Eigen::Vector3d axis_gt = StereoVision::Geometry::inverseRodriguezFormula(integrated.attitudeDelta);
    Eigen::Vector3d axis_no_bias = StereoVision::Geometry::inverseRodriguezFormula(integratedNoisyNoBias.attitudeDelta);
    Eigen::Vector3d axis_raw = StereoVision::Geometry::inverseRodriguezFormula(integratedNoisy.attitudeDelta);
    Eigen::Vector3d axis_computed = StereoVision::Geometry::inverseRodriguezFormula(computed);
    Eigen::Vector3d axis_diff_raw = StereoVision::Geometry::inverseRodriguezFormula(diff_raw);
    Eigen::Vector3d axis_diff_no_bias = StereoVision::Geometry::inverseRodriguezFormula(diff_no_bias);
    Eigen::Vector3d axis_diff = StereoVision::Geometry::inverseRodriguezFormula(diff);
    Eigen::Vector3d axis_diff_wrt_no_bias = StereoVision::Geometry::inverseRodriguezFormula(diffWRTNoBias);

    qInfo() << "Gyro pre-integration with gains and bias: axis angle = " << axis_gt[0] << " " << axis_gt[1] << " " << axis_gt[2]
        << " gains = " << gains[0] << " " << gains[1] << " " << gains[2]
        << " bias = " << bias[0] << " " << bias[1] << " " << bias[2]
        << " std_dev = " << axisSigmas[0] << " " << axisSigmas[1] << " " << axisSigmas[2];

    qInfo() << "Computed (raw) = " << axis_raw[0] << " " << axis_raw[1] << " " << axis_raw[2]
        << " Error = " << axis_diff_raw[0] << " " << axis_diff_raw[1] << " " << axis_diff_raw[2];

    qInfo() << "Computed (no bias) = " << axis_no_bias[0] << " " << axis_no_bias[1] << " " << axis_no_bias[2]
        << " Error = " << axis_diff_no_bias[0] << " " << axis_diff_no_bias[1] << " " << axis_diff_no_bias[2];

    qInfo() << "Computed (corrected) = " << axis_computed[0] << " " << axis_computed[1] << " " << axis_computed[2]
        << " Error = " << axis_diff[0] << " " << axis_diff[1] << " " << axis_diff[2]
        << " Error w.r.t no bias = " << axis_diff_wrt_no_bias[0] << " " << axis_diff_wrt_no_bias[1] << " " << axis_diff_wrt_no_bias[2];

}

void INSPreintegrationBenchmark::benchmarkAcc_data() {

    QTest::addColumn<TrajectoryFunction>("gyroTrajectory"); //ground truth gyro measurements functions
    QTest::addColumn<TrajectoryFunction>("accTrajectory"); //grount truth accelerometer measurement functions
    QTest::addColumn<double>("time"); //integration time
    QTest::addColumn<int>("nSteps"); //number of integration steps
    QTest::addColumn<Eigen::Vector3d>("axisSigmasGyro");
    QTest::addColumn<Eigen::Vector3d>("gainsGyro");
    QTest::addColumn<Eigen::Vector3d>("biasGyro");
    QTest::addColumn<Eigen::Vector3d>("axisSigmasAcc");
    QTest::addColumn<Eigen::Vector3d>("gainsAcc");
    QTest::addColumn<Eigen::Vector3d>("biasAcc");

    TrajectoryFunction constantAccelerationGyro = [] (double t) {
        Q_UNUSED(t);
        return Eigen::Vector3d(0,0,0);
    };

    TrajectoryFunction constantAccelerationAcc = [] (double t) {
        Q_UNUSED(t);
        return Eigen::Vector3d(0,0,9.81);
    };

    QTest::newRow("Constant acc, 10 steps, gyro gains 1e-4 only") << constantAccelerationGyro << constantAccelerationAcc << 1. << 10
                                                                  << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1e-4,1e-4,1e-4) << Eigen::Vector3d(0,0,0) //gyro sigma, gain, bias
                                                                  << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0); //acc sigma, gain, bias

    QTest::newRow("Constant acc, 10 steps, gyro bias 1e-4 only") << constantAccelerationGyro << constantAccelerationAcc << 1. << 10
                                                                 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1e-4,1e-4,1e-4) //gyro sigma, gain, bias
                                                                 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0); //acc sigma, gain, bias

    QTest::newRow("Constant acc, 10 steps, acc gains 1e-4 only") << constantAccelerationGyro << constantAccelerationAcc << 1. << 10
                                                                 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) //gyro sigma, gain, bias
                                                                 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1e-4,1e-4,1e-4) << Eigen::Vector3d(0,0,0); //acc sigma, gain, bias

    QTest::newRow("Constant acc, 10 steps, acc bias 1e-4 only") << constantAccelerationGyro << constantAccelerationAcc << 1. << 10
                                                                << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) //gyro sigma, gain, bias
                                                                << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1e-4,1e-4,1e-4); //acc sigma, gain, bias

    /*QTest::newRow("Base template") << constantAccelerationGyro << constantAccelerationAcc << 1. << 10
                                                                  << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) //gyro sigma, gain, bias
                                                                  << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) //acc sigma, gain, bias;*/

    constexpr double const_w = 1;
    constexpr double const_r = 1;

    TrajectoryFunction constantRotationGyro = [const_w] (double t) {
        Q_UNUSED(t);
        return Eigen::Vector3d(0,0,const_w);
    };

    TrajectoryFunction constantRotationAcc = [const_w, const_r] (double t) {
        Q_UNUSED(t);
        return Eigen::Vector3d(-const_w*const_w*const_r,0,0);
    };

    QTest::newRow("Constant rot, 10 steps, gyro gains 1e-4 only") << constantRotationGyro << constantRotationAcc << 1. << 10
                                                                  << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1e-4,1e-4,1e-4) << Eigen::Vector3d(0,0,0) //gyro sigma, gain, bias
                                                                  << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0); //acc sigma, gain, bias

    QTest::newRow("Constant rot, 10 steps, gyro bias 1e-4 only") << constantRotationGyro << constantRotationAcc << 1. << 10
                                                                 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1e-4,1e-4,1e-4) //gyro sigma, gain, bias
                                                                 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0); //acc sigma, gain, bias

    QTest::newRow("Constant rot, 10 steps, acc gains 1e-4 only") << constantRotationGyro << constantRotationAcc << 1. << 10
                                                                 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) //gyro sigma, gain, bias
                                                                 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1e-4,1e-4,1e-4) << Eigen::Vector3d(0,0,0); //acc sigma, gain, bias

    QTest::newRow("Constant rot, 10 steps, acc bias 1e-4 only") << constantRotationGyro << constantRotationAcc << 1. << 10
                                                                << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) //gyro sigma, gain, bias
                                                                << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1e-4,1e-4,1e-4); //acc sigma, gain, bias

    TrajectoryFunction constantSpinGyro = [const_w] (double t) {
        Q_UNUSED(t);
        return Eigen::Vector3d(0,0,const_w);
    };

    TrajectoryFunction constantSpinAcc = [const_w] (double t) {
        return Eigen::Vector3d(std::cos(const_w*t),-std::sin(const_w*t),9.81);
    };

    QTest::newRow("Constant spin, 10 steps, gyro gains 1e-4 only") << constantSpinGyro << constantSpinAcc << 1. << 10
                                                                  << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1e-4,1e-4,1e-4) << Eigen::Vector3d(0,0,0) //gyro sigma, gain, bias
                                                                  << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0); //acc sigma, gain, bias

    QTest::newRow("Constant spin, 10 steps, gyro bias 1e-4 only") << constantSpinGyro << constantSpinAcc << 1. << 10
                                                                 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1e-4,1e-4,1e-4) //gyro sigma, gain, bias
                                                                 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0); //acc sigma, gain, bias

    QTest::newRow("Constant spin, 10 steps, acc gains 1e-4 only") << constantSpinGyro << constantSpinAcc << 1. << 10
                                                                 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) //gyro sigma, gain, bias
                                                                 << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1e-4,1e-4,1e-4) << Eigen::Vector3d(0,0,0); //acc sigma, gain, bias

    QTest::newRow("Constant spin, 10 steps, acc bias 1e-4 only") << constantSpinGyro << constantSpinAcc << 1. << 10
                                                                << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) //gyro sigma, gain, bias
                                                                << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(0,0,0) << Eigen::Vector3d(1e-4,1e-4,1e-4); //acc sigma, gain, bias

}
void INSPreintegrationBenchmark::benchmarkAcc() {

    QFETCH(TrajectoryFunction, gyroTrajectory);
    QFETCH(TrajectoryFunction, accTrajectory);

    QFETCH(double, time);
    QFETCH(int, nSteps);

    QFETCH(Eigen::Vector3d, axisSigmasGyro);
    QFETCH(Eigen::Vector3d, gainsGyro);
    QFETCH(Eigen::Vector3d, biasGyro);

    QFETCH(Eigen::Vector3d, axisSigmasAcc);
    QFETCH(Eigen::Vector3d, gainsAcc);
    QFETCH(Eigen::Vector3d, biasAcc);

    using TimeSeq = StereoVisionApp::IndexedTimeSequence<Eigen::Vector3d,double>;

    std::vector<TimeSeq::TimedElement> elementsGyroGt;
    std::vector<TimeSeq::TimedElement> elementsAccGt;

    std::vector<TimeSeq::TimedElement> elementsGyroBiased;
    std::vector<TimeSeq::TimedElement> elementsAccBiased;

    std::uniform_real_distribution<double> noiseDistribution(-1,1);

    elementsGyroGt.reserve(nSteps);
    elementsAccGt.reserve(nSteps);
    elementsGyroBiased.reserve(nSteps);
    elementsAccBiased.reserve(nSteps);

    for (int i = 0; i <= nSteps; i++) {
        double t = i*time/(nSteps);

        Eigen::Vector3d gyroGt = gyroTrajectory(t);
        Eigen::Vector3d accGt = accTrajectory(t);

        Eigen::Vector3d gyroNoise(axisSigmasGyro[0]*noiseDistribution(_re), axisSigmasGyro[1]*noiseDistribution(_re), axisSigmasGyro[2]*noiseDistribution(_re));
        Eigen::Vector3d accNoise(axisSigmasAcc[0]*noiseDistribution(_re), axisSigmasAcc[1]*noiseDistribution(_re), axisSigmasAcc[2]*noiseDistribution(_re));

        Eigen::Vector3d gyroBiased = (gainsGyro+Eigen::Vector3d::Ones()).cwiseProduct(gyroGt) + biasGyro + gyroNoise;
        Eigen::Vector3d accBiased = (gainsAcc+Eigen::Vector3d::Ones()).cwiseProduct(accGt) + biasAcc + accNoise;

        elementsGyroGt.push_back({t, gyroGt});
        elementsAccGt.push_back({t, accGt});

        elementsGyroBiased.push_back({t, gyroBiased});
        elementsAccBiased.push_back({t, accBiased});
    }

    TimeSeq seqGyroGt(std::move(elementsGyroGt));
    TimeSeq seqAccGt(std::move(elementsAccGt));

    TimeSeq seqGyroBiased(std::move(elementsGyroBiased));
    TimeSeq seqAccBiased(std::move(elementsAccBiased));

    constexpr double t0 = 0;
    double tm = time/2;
    double tf = time;

    auto integratedAccGt = StereoVisionApp::AccelerometerStepCostBase::preIntegrateAccSegment(seqGyroGt,seqAccGt,t0,tm,tf);
    auto integratedAccBiased = StereoVisionApp::AccelerometerStepCostBase::preIntegrateAccSegment(seqGyroBiased,seqAccBiased,t0,tm,tf);

    Eigen::Vector3d gainGyroInverse = (gainsGyro+Eigen::Vector3d::Ones()).cwiseInverse()-Eigen::Vector3d::Ones();
    Eigen::Vector3d biasGyroInverse = -(gainsGyro+Eigen::Vector3d::Ones()).cwiseInverse().cwiseProduct(biasGyro);

    Eigen::Vector3d gainAccInverse = (gainsAcc+Eigen::Vector3d::Ones()).cwiseInverse()-Eigen::Vector3d::Ones();
    Eigen::Vector3d biasAccInverse = -(gainsAcc+Eigen::Vector3d::Ones()).cwiseInverse().cwiseProduct(biasAcc);

    Eigen::Vector3d expected = integratedAccGt.speedDelta;
    Eigen::Vector3d biased = integratedAccBiased.speedDelta;

    Eigen::Vector3d gyroGainCorrection = integratedAccBiased.gyroGainJacobian*gainGyroInverse;
    Eigen::Vector3d gyroBiasCorrection = integratedAccBiased.gyroBiasJacobian*biasGyroInverse;

    Eigen::Vector3d accGainCorrection = integratedAccBiased.accGainJacobian*gainAccInverse;
    Eigen::Vector3d accBiasCorrection = integratedAccBiased.accBiasJacobian*biasAccInverse;

    Eigen::Vector3d corrected = integratedAccBiased.speedDelta
                                + gyroGainCorrection
                                + gyroBiasCorrection
                                + accGainCorrection
                                + accBiasCorrection;
    Eigen::Vector3d error_biased = expected - biased;
    Eigen::Vector3d error = expected - corrected;

    qInfo() << "expected = " << expected[0] << " " << expected[1] << " " << expected[2];
    qInfo() << " biased = " << biased[0] << " " << biased[1] << " " << biased[2];
    qInfo() << " Error biased = " << error_biased[0] << " " << error_biased[1] << " " << error_biased[2];
    qInfo() << " corrected = " << corrected[0] << " " << corrected[1] << " " << corrected[2];
    qInfo() << " Error w.r.t no bias = " << error[0] << " " << error[1] << " " << error[2];

}

QTEST_MAIN(INSPreintegrationBenchmark);
#include "main.moc"
