#include <iostream>

#include "sparsesolver/costfunctors/imustepcost.h"
#include "vision/indexed_timed_sequence.h"

#include <random>

using Timeline = StereoVisionApp::IndexedTimeSequence<Eigen::Vector3d, double>;

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

int main(int argc, char** argv) {

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

}
