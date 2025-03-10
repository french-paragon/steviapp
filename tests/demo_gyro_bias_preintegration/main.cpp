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

int main(int argc, char** argv) {

    std::default_random_engine re;
    re.seed(std::random_device()());

    std::uniform_real_distribution<double> distribution(-M_PI,M_PI);

    Eigen::Vector3d axisInitial;
    Eigen::Vector3d axisFinal;

    for (int i = 0; i < 3; i++) {
        axisInitial[i] = distribution(re);
        axisFinal[i] = distribution(re);
    }

    Eigen::Vector3d stepStd(0.005,0.005,0.005);
    int nSteps = 10;

    Eigen::Vector3d gains(1.01,1.01,1.01);
    Eigen::Vector3d bias(0.01,0.01,0.01);

    runGyroExperiment(axisInitial, axisFinal, stepStd, nSteps, gains, bias, re);

}
