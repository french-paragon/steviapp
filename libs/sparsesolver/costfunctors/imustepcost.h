#ifndef STEREOVISIONAPP_IMUSTEPCOST_H
#define STEREOVISIONAPP_IMUSTEPCOST_H

#include <Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>

#include "../../vision/indexed_timed_sequence.h"
#include "../../utils/functional.h"

#ifndef NDEBUG
#include <iostream>
#include <ceres/jet.h>
#endif

namespace StereoVisionApp {

template<bool WBias, bool WScale>
class GyroStepCost;

/*!
 * \brief The GyroStepCost class represent a measurement by an gyroscope between two poses
 */
class GyroStepCostBase {

public:

    struct IntegratedGyroSegment {
        Eigen::Matrix3d attitudeDelta;
        Eigen::Matrix3d biasJacobian;
        Eigen::Matrix3d gainJacobian;
        double dt;
    };

    template<bool WBias, bool WScale, typename Tgyro, typename TimeT>
    static IntegratedGyroSegment preIntegrateGyroSegment(IndexedTimeSequence<Tgyro, TimeT> const& GyroData,
                                                  TimeT t0,
                                                  TimeT t1);

    template<bool WBias, bool WScale, template <typename T> typename Decorator = IdentityDecorator, typename Tgyro, typename TimeT, typename ... DecoratorAdditionalArgT>
    static Decorator<GyroStepCost<WBias,WScale>>* getIntegratedIMUDiff(IndexedTimeSequence<Tgyro, TimeT> const& GyroData,
                                                                        TimeT t0,
                                                                        TimeT t1,
                                                                        DecoratorAdditionalArgT ... additionalArgs);

    GyroStepCostBase(Eigen::Matrix3d const& attitudeDelta,
                 double delta_t);

    template <bool WBias, bool WScale, typename T>
    bool computeGyroCostGeneric(const T* const r0,
                                const T* const r1,
                                const T* const gyroScale,
                                const T* const gyroBias,
                                T* residual,
                                Eigen::Matrix<double,3,3> const& scaleJacobian = Eigen::Matrix<double,3,3>::Identity(),
                                Eigen::Matrix<double,3,3> const& biasJacobian = Eigen::Matrix<double,3,3>::Identity()) const {

        using M3T = Eigen::Matrix<T,3,3>;
        using V3T = Eigen::Matrix<T,3,1>;

        V3T vr0(r0[0], r0[1], r0[2]);
        V3T vr1(r1[0], r1[1], r1[2]);

        V3T g(T(0), T(0), T(0));
        V3T b(T(0), T(0), T(0));

        if (gyroScale != nullptr) {
            g = V3T(gyroScale[0] - T(1), gyroScale[1] - T(1), gyroScale[2] - T(1));
        }

        if (gyroBias != nullptr) {
            b = V3T(gyroBias[0], gyroBias[1], gyroBias[2]);
        }

        M3T Rr0 = StereoVision::Geometry::rodriguezFormula<T>(vr0); //R body2world at time 0
        M3T Rr1 = StereoVision::Geometry::rodriguezFormula<T>(vr1); //R body2world at time 1

        M3T closure;

        if constexpr (WScale and WBias) {
            M3T Jg = scaleJacobian.cast<T>();
            M3T Jb = biasJacobian.cast<T>();

            M3T Rbg = StereoVision::Geometry::rodriguezFormula<T>(Jg*g + Jb*b);

            closure = Rr1.transpose() * Rr0 * Rbg*_attitudeDelta;
        } else if constexpr (WScale) {
            M3T Jg = scaleJacobian.cast<T>();

            M3T Rbg = StereoVision::Geometry::rodriguezFormula<T>(Jg*g);

            closure = Rr1.transpose() * Rr0 * Rbg*_attitudeDelta;
        } else if constexpr (WBias) {
            M3T Jb = biasJacobian.cast<T>();

            M3T Rbg = StereoVision::Geometry::rodriguezFormula<T>(Jb*b);

            closure = Rr1.transpose() * Rr0 * Rbg*_attitudeDelta;
        } else {
            closure = Rr1.transpose() * Rr0 * _attitudeDelta;
        }

        V3T res = StereoVision::Geometry::inverseRodriguezFormula<T>(closure);

        residual[0] = res[0];
        residual[1] = res[1];
        residual[2] = res[2];

#ifndef NDEBUG
        if (!ceres::isfinite(residual[0]) or !ceres::isfinite(residual[1]) or !ceres::isfinite(residual[2])) {
            std::cout << "Error in GyroStepCost cost computation" << std::endl;
        }
#endif

        return true;

    }

protected:

    Eigen::Matrix3d _attitudeDelta; // R from final pose to initial pose
    double _delta_t;
};

template<>
class GyroStepCost<false, false> : public GyroStepCostBase {

public:
    GyroStepCost(Eigen::Matrix3d const& attitudeDelta,
                 double delta_t) :
        GyroStepCostBase(attitudeDelta, delta_t)
    {

    }

    template <typename T>
    bool operator()(const T* const r0,
                    const T* const r1,
                    T* residual) const {

        return computeGyroCostGeneric<false, false, T>(r0, r1, nullptr, nullptr, residual);

    }
};

template<>
class GyroStepCost<false, true> : public GyroStepCostBase {

public:
    GyroStepCost(Eigen::Matrix3d const& attitudeDelta,
                 Eigen::Matrix3d const& gainJacobian,
                 double delta_t) :
        GyroStepCostBase(attitudeDelta, delta_t),
        _scaleJacobian(gainJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const r0,
                    const T* const r1,
                    const T* const gyroScale,
                    T* residual) const {

        return computeGyroCostGeneric<false, true, T>(r0, r1, gyroScale, nullptr, residual, _scaleJacobian);

    }

protected:

    Eigen::Matrix3d _scaleJacobian;
};

template<>
class GyroStepCost<true, false> : public GyroStepCostBase {

public:
    GyroStepCost(Eigen::Matrix3d const& attitudeDelta,
                 Eigen::Matrix3d const& biasJacobian,
                 double delta_t) :
        GyroStepCostBase(attitudeDelta, delta_t),
        _biasJacobian(biasJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const r0,
                    const T* const r1,
                    const T* const gyroBias,
                    T* residual) const {

        return computeGyroCostGeneric<true, false, T>(r0, r1, nullptr, gyroBias, residual, Eigen::Matrix3d::Identity(), _biasJacobian);

    }

protected:

    Eigen::Matrix3d _biasJacobian;
};

template<>
class GyroStepCost<true, true> : public GyroStepCostBase {

public:
    GyroStepCost(Eigen::Matrix3d const& attitudeDelta,
                 Eigen::Matrix3d const& gainJacobian,
                 Eigen::Matrix3d const& biasJacobian,
                 double delta_t) :
        GyroStepCostBase(attitudeDelta, delta_t),
        _scaleJacobian(gainJacobian),
        _biasJacobian(biasJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const r0,
                    const T* const r1,
                    const T* const gyroScale,
                    const T* const gyroBias,
                    T* residual) const {

        return computeGyroCostGeneric<true, true, T>(r0, r1, gyroScale, gyroBias, residual, _scaleJacobian, _biasJacobian);

    }

protected:

    Eigen::Matrix3d _biasJacobian;
    Eigen::Matrix3d _scaleJacobian;
};



template<bool WBias, bool WScale, typename Tgyro, typename TimeT>
GyroStepCostBase::IntegratedGyroSegment GyroStepCostBase::preIntegrateGyroSegment(IndexedTimeSequence<Tgyro, TimeT> const& GyroData,
                                                                TimeT t0,
                                                                TimeT t1) {

    double delta_t = t1 - t0;

    auto imuGyroData = GyroData.getValuesInBetweenTimes(t0, t1);

    IntegratedGyroSegment ret;

    ret.dt = delta_t;
    ret.attitudeDelta = Eigen::Matrix3d::Identity();

    ret.biasJacobian = Eigen::Matrix3d::Zero();
    ret.gainJacobian = Eigen::Matrix3d::Zero();

    /*
     * The management of the gyro bias an gain can be done by re-integrating the data everytime,
     * or by having a closed form of the pre-integrated delta as a function of the scale alpha and bias b.
     *
     * Here, we assume alpha is close to 1 and b close to 0, and we linearize the pre-integrated
     * orientation delta as a function of alpha and b on the manifold.
     *
     * The relations we use are
     * (1) Exp(w+dw) ~= Exp(w)Exp(J_w dw), with w an element of so(3),
     * Exp the exponential map from so(3) to SO(3) and dw a small increment in so(3) and J_w is
     * the right Jacobian of SO(3) at w.
     *
     * and:
     * (2) Exp(w) R = R Exp(R^t w), which derive from the relation R Exp(w) R^t = e^(R [w]_x R^t) = Exp(Rw),
     * where [w]_x is the skew symmetric matrix derived from w.
     *
     * First, we decompose each Exp((alpha*w - b)dt) as Exp(w dt)Exp(J_{wdt}((alpha-1)w - b)) using relation (1).
     * Then, with relation (2), we invert the order of alternated rotations and increments to group all the increments as
     * one large products. Finally, we assume that, since the increments are small, their Jacobian of SO(3)
     * are close to the identity. Using relation (1), we transform the product into a sum.
     *
     * We end up with a relation of the form DeltaR_corrected = DeltaR Exp(J_alpha * (alpha-1) + J_b b)
     */

    for (int j = 0; j < imuGyroData.size(); j++) {

        double dt = imuGyroData[j].dt;

        Eigen::Vector3d dr(imuGyroData[j].val[0], imuGyroData[j].val[1], imuGyroData[j].val[2]);

        dr *= dt;

        Eigen::Matrix3d dR = StereoVision::Geometry::rodriguezFormula(dr);

        Eigen::Matrix3d newRcurrent2initial = ret.attitudeDelta*dR;

        //re-constaint as R mat for numerical stability
        Eigen::Vector3d logR = StereoVision::Geometry::inverseRodriguezFormula(newRcurrent2initial);
        ret.attitudeDelta = StereoVision::Geometry::rodriguezFormula(logR);

        Eigen::Matrix3d JacobianSO3 = StereoVision::Geometry::diffRodriguezLieAlgebra(dr);

        Eigen::Matrix3d tmp = ret.attitudeDelta.transpose()*JacobianSO3;

        Eigen::Matrix3d diag = Eigen::Matrix3d::Zero();
        diag(0,0) = dr[0];
        diag(1,1) = dr[1];
        diag(2,2) = dr[2];

        if (WBias) {
            ret.biasJacobian += tmp*dt;
        }

        if (WScale) {
            ret.gainJacobian += tmp*diag;
        }
    }

    return ret;



}

template<bool WBias, bool WScale, template <typename T> typename Decorator, typename Tgyro, typename TimeT, typename ... DecoratorAdditionalArgT>
    Decorator<GyroStepCost<WBias, WScale>> *GyroStepCostBase::getIntegratedIMUDiff(IndexedTimeSequence<Tgyro, TimeT> const& GyroData,
                                                                               TimeT t0,
                                                                               TimeT t1,
                                                                               DecoratorAdditionalArgT ... additionalDecoratorArgs) {

    IntegratedGyroSegment pre_integrated = preIntegrateGyroSegment<WBias, WScale, Tgyro, TimeT>(GyroData, t0, t1);

    if constexpr (WBias and WScale) {
        return new Decorator<GyroStepCost<true, true>>(additionalDecoratorArgs ...,
                                                       pre_integrated.attitudeDelta,
                                                       pre_integrated.gainJacobian,
                                                       pre_integrated.biasJacobian,
                                                       pre_integrated.dt);
    } else if constexpr (WBias) {
        return new Decorator<GyroStepCost<true, false>>(additionalDecoratorArgs ...,
                                                        pre_integrated.attitudeDelta,
                                                        pre_integrated.biasJacobian,
                                                        pre_integrated.dt);
    } else if constexpr (WScale) {
        return new Decorator<GyroStepCost<false, true>>(additionalDecoratorArgs ...,
                                                        pre_integrated.attitudeDelta,
                                                        pre_integrated.gainJacobian,
                                                        pre_integrated.dt);
    } else {
        return new Decorator<GyroStepCost<false, false>>(additionalDecoratorArgs ...,
                                                         pre_integrated.attitudeDelta,
                                                         pre_integrated.dt);
    }


}

enum AccelerometerStepCostFlags {
    NoBias = 0,
    GyroBias = 1,
    GyroScale = 2,
    AccBias = 4,
    AccScale = 8
};

template <int flags>
struct AccelerometerStepCostTraits{
    constexpr static bool WGBias = flags & AccelerometerStepCostFlags::GyroBias;
    constexpr static bool WGScale = flags & AccelerometerStepCostFlags::GyroScale;
    constexpr static bool WABias = flags & AccelerometerStepCostFlags::AccBias;
    constexpr static bool WAScale = flags & AccelerometerStepCostFlags::AccScale;

    static constexpr int nAccCostParams() {

        int n = 5; //no bias or scale factors

        if (WGBias) {
            n += 1;
        }

        if (WGScale) {
            n += 1;
        }

        if (WABias) {
            n += 1;
        }

        if (WAScale) {
            n += 1;
        }

        return n;
    }

    static constexpr int refPosParamIdx() {
        return 2;
    }

    static constexpr int gravityParamIdx() {
        return nAccCostParams()-1; //gravity is last parameter
    }
};

template<int flags>
class AccelerometerStepCost;

/*!
 * \brief The AccelerometerStepCostBase class represent a measurement by an accelerometer between three poses.
 * It uses the average speed delta between the average speed in the two intervals.
 *
 * The benefits of doing so is that is that we do not need the speed as a learnable parameter, only the poses.
 * Also, it does not have numerical accuracy issue when computing the speed from the poses, only when
 * integrating the speed from the acceleration, which is less of an issue as the acceleration is sampled
 * with much higher frequency.
 *
 * The average speed in the interval t_i to t_j can be computed from \Zeta_i^j = int_{t_i}^{t_j} int_{t_0}^{t} int_n^{t'} w(t'') dt'' f(t) dt' dt
 * Such that \mean{v}_i^j = v_i + (tj - ti)/2 g + R_t^w \Zeta_i^j / (t_j - t_i).
 *
 * More crucially we also have \mean{v}_i^j = (p_j - p_i)/(t_j - t_i) (p being the platform position), and
 * \mean{v}_i^j = \mean{v}_j^i. This mean, from three poses p_0, p_1 and p_2 we can express a constraint:
 *
 * (p_2 - p_1)/(t_2 - t_1) - (p_1 - p_0)/(t_1 - t_0) = R_{t1}^w (\Zeta_1^2 / (t_2 - t_1) - \Zeta_1^0 / (t_1 - t_0)) + (t2 - t_0)/2 g
 *
 * the three poses under consideration are at t0, t1, t2. We use t1 as reference time, integrating from there twice cancel out the unknown initial speed.
 *
 * from which one get the cost:
 *
 * C(p_1, p_0, p2, R_{t1}^w) = (p_2 - p_1)/(t_2 - t_1) - (p_1 - p_0)/(t_1 - t_0) - R_{t1}^w (\Zeta_1^2 / (t_2 - t_1) - \Zeta_1^0 / (t_1 - t_0)) - (t2 - t_0)/2 g
 *
 * where v_{t} a speed expressed in world (local) frame at time t,
 * R_{t}^w is the attitude (R from body to world) at time t,
 * g is the gravity in world frame,
 * f(t) is the measured specific force at time t, equals R_w^t * (a(t) - g), with g the gravity acceleration and a(t) the acceleration in global frame.
 * int_{ti}^{tj} w(t') dt' is the estimated relative orientation between the pose at time ti and time tj, integrated from the gyro measurements.
 *
 * In the case a bias is present, the model becomes \Zeta_i^j = int_{t_i}^{t_j} int_{t_0}^{t} int_n^{t'} (s_g*w(t'') + b_g) dt'' (s_a*f(t) + b_a) dt' dt
 * We will assume the bias and scale parameters are small, and linearize. We get:
 * C(p_0, p_{-1}, p{1}, R_{t0}^w, s_g, b_g, s_a, b_a) = C(p_0, p_{-1}, p{1}, R_{t0}^w) + R_{t0}^w (J_{sg} s_g + J_{bg} b_g + J_{sa} s_a + J_{ba} b_a).
 * With J_{pi} the jacobian for parameter p_i. This is possible because the contributions of s_g, b_g, s_a, b_a to \Zeta_i^j do not depends on the value of the other parameters.
 * As such, it is possible to linearize \Zeta_i^j w.r.t s_g, b_g, s_a, b_a
 *
 * Poses are assumed to be the Body2World (or Body2Local if a local frame is used).
 *
 *
 * In addition the cost is configure with delta_t1 = t1 - t0 and delta_t2 = t2 - t1
 *
 * The class has different implementation of the cost, depending on the stochastic model is implemented in the child class.
 *
 *
 */
class AccelerometerStepCostBase {

public:

    struct IntegratedAccSegment {
        Eigen::Vector3d speedDelta;
        Eigen::Matrix3d accBiasJacobian;
        Eigen::Matrix3d accGainJacobian;
        Eigen::Matrix3d gyroBiasJacobian;
        Eigen::Matrix3d gyroGainJacobian;
        double dt1;
        double dt2;
    };

    template<typename Tgyro, typename Tacc, typename TimeT>
    static inline IntegratedAccSegment preIntegrateAccSegmentSingleHalf(
        IndexedTimeSequence<Tgyro, TimeT> const& GyroData,
        IndexedTimeSequence<Tacc, TimeT> const& AccData,
        TimeT t0,
        TimeT t1) {

        using DifferentialAcc = typename IndexedTimeSequence<Tacc, TimeT>::Differential;
        using DifferentialGyro = typename IndexedTimeSequence<Tgyro, TimeT>::Differential;

        double delta_t = t1 - t0;

        IntegratedAccSegment ret;
        ret.speedDelta = Eigen::Vector3d::Zero();
        ret.accBiasJacobian = Eigen::Matrix3d::Zero();
        ret.accGainJacobian = Eigen::Matrix3d::Zero();
        ret.gyroBiasJacobian = Eigen::Matrix3d::Zero();
        ret.gyroGainJacobian = Eigen::Matrix3d::Zero();
        ret.dt1 = delta_t;
        ret.dt2 = 0;

        std::vector<DifferentialAcc> imuAccData = AccData.getValuesInBetweenTimes(t0, t1);
        std::vector<DifferentialGyro> imuGyroData = GyroData.getValuesInBetweenTimes(t0, t1);

        double biasJacobiansScale = 1;

        if (t1 < t0) { //change the dts to be positive and put the -1 factor on the values.
            //this enable the rest of the function to work transparently.

            for (DifferentialAcc & differential : imuAccData) {
                differential.dt *= -1;
                differential.val *= -1;
            }

            for (DifferentialGyro & differential : imuGyroData) {
                differential.dt *= -1;
                differential.val *= -1;
            }

            biasJacobiansScale = -1; //scale are scaled by -1 above, but we need to do the same to the bias.

            ret.dt1 = -delta_t; //time should still be counted positive for averaging purposes, even if we start integrating from the end of the time interval.
        }

        Eigen::Vector3d current = Eigen::Vector3d::Zero();
        Eigen::Matrix3d Rcurrent2initial = Eigen::Matrix3d::Identity();

        Eigen::Matrix3d gyroSO3BiasJacobian = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d gyroSO3GainJacobian = Eigen::Matrix3d::Zero();

        Eigen::Matrix3d gyroBiasJacobian = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d gyroGainJacobian = Eigen::Matrix3d::Zero();

        Eigen::Matrix3d accBiasJacobian = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d accGainJacobian = Eigen::Matrix3d::Zero();

        size_t idxAcc = 0;
        size_t idxGyro = 0;
        double ddt_acc = 0;
        double ddt_gyro = 0;

        //integration loop

        while(idxAcc < imuAccData.size() and idxGyro < imuGyroData.size()) {

            //compute the time increment
            double dt_acc = imuAccData[idxAcc].dt - ddt_acc;
            double dt_gyro = imuGyroData[idxGyro].dt - ddt_gyro;

            double min_time_interval = std::min(imuAccData[idxAcc].dt, imuGyroData[idxGyro].dt);
            double dt_threshold = min_time_interval/100;
            double dt = std::min(dt_acc, dt_gyro); //do 100 intermediate steps for more precise integration.
            dt = std::min(dt, dt_threshold);

            //increments
            Eigen::Vector3d dr = imuGyroData[idxGyro].val;
            if (imuGyroData.size() > idxGyro+1 and dt > 0) {
                Eigen::Vector3d dr_next = imuGyroData[idxGyro+1].val;
                dr = dr*dt_gyro + dr_next*ddt_gyro;
                dr /= imuGyroData[idxGyro].dt;
            }
            dr *= dt;

            Eigen::Vector3d acceleration_local = imuAccData[idxAcc].val;
            if (imuAccData.size() > idxAcc+1 and dt > 0) {
                Eigen::Vector3d acceleration_local_next = imuAccData[idxAcc+1].val;
                acceleration_local = acceleration_local*dt_acc + acceleration_local_next*ddt_acc;
                acceleration_local /= imuAccData[idxAcc].dt;
            }
            Eigen::Vector3d df = acceleration_local;
            df *= dt;

            Eigen::Matrix3d diagF = Eigen::Matrix3d::Zero();
            for (int i = 0; i < 3; i++) {
                diagF(i,i) = df[i];
            }

            //compute the current rotation from the refence and associated jacobians
            Eigen::Matrix3d dR = StereoVision::Geometry::rodriguezFormula(dr);

            Eigen::Matrix3d rodriguezJac = Eigen::Matrix3d::Zero();

            rodriguezJac.block<3,1>(0,0) =
                StereoVision::Geometry::diffAngleAxisRotate<double>(Eigen::Vector3d::Zero(),
                                                                    acceleration_local,
                                                                    StereoVision::Geometry::Axis::X);
            rodriguezJac.block<3,1>(0,1) =
                StereoVision::Geometry::diffAngleAxisRotate<double>(Eigen::Vector3d::Zero(),
                                                                    acceleration_local,
                                                                    StereoVision::Geometry::Axis::Y);
            rodriguezJac.block<3,1>(0,2) =
                StereoVision::Geometry::diffAngleAxisRotate<double>(Eigen::Vector3d::Zero(),
                                                                    acceleration_local,
                                                                    StereoVision::Geometry::Axis::Z);

            gyroBiasJacobian += rodriguezJac*gyroSO3BiasJacobian*dt;
            gyroGainJacobian += rodriguezJac*gyroSO3GainJacobian*dt;

            Eigen::Matrix3d JacobianSO3 = StereoVision::Geometry::diffRodriguezLieAlgebra(dr);

            Eigen::Matrix3d tmp = Rcurrent2initial.transpose()*JacobianSO3;

            Eigen::Matrix3d diagR = Eigen::Matrix3d::Zero();
            diagR(0,0) = dr[0];
            diagR(1,1) = dr[1];
            diagR(2,2) = dr[2];

            gyroSO3BiasJacobian += tmp*dt;

            gyroSO3GainJacobian += tmp*diagR;

            Eigen::Matrix3d Rcurrent2initialAvgSegment = Rcurrent2initial;
            Rcurrent2initial = dR*Rcurrent2initial;
            Rcurrent2initialAvgSegment = 1*Rcurrent2initialAvgSegment + 1*Rcurrent2initial;
            Rcurrent2initialAvgSegment /= 2;

            //increment the accelerometers jacobians
            accBiasJacobian += Rcurrent2initialAvgSegment*dt;
            accGainJacobian += Rcurrent2initialAvgSegment*diagF;

            //increment current inner integrant value
            Eigen::Vector3d currentOld = current;
            current += Rcurrent2initialAvgSegment*df;

            //outer integration loop
            ret.speedDelta += (currentOld + current)/2 * dt; //use trapezoidal rule for the outer integration loop

            ret.accBiasJacobian += biasJacobiansScale*accBiasJacobian*dt;
            ret.accGainJacobian += accGainJacobian*dt;
            ret.gyroBiasJacobian += biasJacobiansScale*gyroBiasJacobian*dt;
            ret.gyroGainJacobian += gyroGainJacobian*dt;

            //update variables
            ddt_acc += dt;
            ddt_gyro += dt;

            if (ddt_acc >= imuAccData[idxAcc].dt) {
                idxAcc++;
                ddt_acc = 0;
            }

            if (ddt_gyro >= imuGyroData[idxGyro].dt) {
                idxGyro++;
                ddt_gyro = 0;
            }
        }

        return ret;

    }

    template<typename Tgyro, typename Tacc, typename TimeT>
    static inline IntegratedAccSegment preIntegrateAccSegment(IndexedTimeSequence<Tgyro, TimeT> const& GyroData,
                                                        IndexedTimeSequence<Tacc, TimeT> const& AccData,
                                                        TimeT t0,
                                                        TimeT t1,
                                                        TimeT t2) {

        //integrate with t1 as reference.
        IntegratedAccSegment segment1 = preIntegrateAccSegmentSingleHalf(GyroData,AccData,t1,t0);
        IntegratedAccSegment segment2 = preIntegrateAccSegmentSingleHalf(GyroData,AccData,t1,t2);

        IntegratedAccSegment ret;
        ret.speedDelta = segment2.speedDelta/segment2.dt1 - segment1.speedDelta/segment1.dt1;
        ret.accBiasJacobian = segment2.accBiasJacobian/segment2.dt1 - segment1.accBiasJacobian/segment1.dt1;
        ret.accGainJacobian = segment2.accGainJacobian/segment2.dt1 - segment1.accGainJacobian/segment1.dt1;
        ret.gyroBiasJacobian = segment2.gyroBiasJacobian/segment2.dt1 - segment1.gyroBiasJacobian/segment1.dt1;
        ret.gyroGainJacobian = segment2.gyroGainJacobian/segment2.dt1 - segment1.gyroGainJacobian/segment1.dt1;
        ret.dt1 = t1-t0;
        ret.dt2 = t2-t1;

        return ret;

    }

    template<int accCostFlags, template <typename T> typename Decorator = IdentityDecorator, typename Tgyro, typename Tacc, typename TimeT, typename ... DecoratorAdditionalArgsT>
    static Decorator<AccelerometerStepCost<accCostFlags>>* getIntegratedIMUDiff(IndexedTimeSequence<Tgyro, TimeT> const& GyroData,
                                                                                IndexedTimeSequence<Tacc, TimeT> const& AccData,
                                                                                TimeT t0,
                                                                                TimeT t1,
                                                                                TimeT t2,
                                                                                DecoratorAdditionalArgsT ... decoratorAdditionalArgs);

    AccelerometerStepCostBase(Eigen::Vector3d const& speedDelta,
                              double delta_t1,
                              double delta_t2);



    template <bool WGBias, bool WGScale, bool WABias, bool WAScale, typename T>
    bool computeResidualsGeneric(const T* const t0,
                                 const T* const r1,
                                 const T* const t1,
                                 const T* const t2,
                                 const T* const g,
                                 T* residual,
                                 const T* const gGain = nullptr,
                                 const T* const gBias = nullptr,
                                 const T* const aGain = nullptr,
                                 const T* const aBias = nullptr,
                                 Eigen::Matrix3d const& gyroBiasJacobian = Eigen::Matrix3d::Identity(),
                                 Eigen::Matrix3d const& gyroGainJacobian = Eigen::Matrix3d::Identity(),
                                 Eigen::Matrix3d const& accBiasJacobian = Eigen::Matrix3d::Identity(),
                                 Eigen::Matrix3d const& accGainJacobian = Eigen::Matrix3d::Identity()) const {

        using M3T = Eigen::Matrix<T,3,3>;
        using V3T = Eigen::Matrix<T,3,1>;

        V3T vr1(r1[0], r1[1], r1[2]);

        V3T vt0(t0[0], t0[1], t0[2]);
        V3T vt1(t1[0], t1[1], t1[2]);
        V3T vt2(t2[0], t2[1], t2[2]);

        V3T vg(g[0], g[1], g[2]);

        V3T vv0 = (vt1 - vt0)/T(_delta_t1); //avg speed in first interval
        V3T vv1 = (vt2 - vt1)/T(_delta_t2); //avg speed in second interval

        M3T Rr1 = StereoVision::Geometry::rodriguezFormula<T>(vr1); //attitude at reference point

        //platform2world orientation at (t0 + t1)/2

        T spdx(_speedDelta[0]);
        T spdy(_speedDelta[1]);
        T spdz(_speedDelta[2]);
        V3T vSpeedDelta(spdx, spdy, spdz);

        if (WGBias) {
            V3T GyroBias(gBias[0], gBias[1], gBias[2]);
            vSpeedDelta += gyroBiasJacobian.cast<T>()*GyroBias;
        }

        if (WGScale) {
            V3T GyroGain(gGain[0]-T(1), gGain[1]-T(1), gGain[2]-T(1));
            vSpeedDelta += gyroGainJacobian.cast<T>()*GyroGain;
        }

        if (WABias) {
            V3T AccBias(aBias[0], aBias[1], aBias[2]);
            vSpeedDelta += accBiasJacobian.cast<T>()*AccBias;
        }

        if (WAScale) {
            V3T AccGain(aGain[0]-T(1), aGain[1]-T(1), aGain[2]-T(1));
            vSpeedDelta += accGainJacobian.cast<T>()*AccGain;
        }

        V3T gCorr = vg*T(_delta_t2 + _delta_t1)/T(2);
        V3T alignedSpeedDelta = Rr1*vSpeedDelta;
        V3T corr = gCorr + alignedSpeedDelta;
        V3T deltaV = vv1 - vv0;
        V3T err = deltaV - corr;

        residual[0] = err[0];
        residual[1] = err[1];
        residual[2] = err[2];

#ifndef NDEBUG
        if (!ceres::isfinite(residual[0]) or !ceres::isfinite(residual[1]) or !ceres::isfinite(residual[2])) {
            std::cout << "Error in AccelerometerStepCost cost computation" << std::endl;
        }
#endif

        return true;

    }

protected:

    Eigen::Vector3d _speedDelta;
    double _delta_t1;
    double _delta_t2;

};

template<>
class AccelerometerStepCost<NoBias>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2)
    {

    }

    template <typename T>
    bool operator()(const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const g,
                    T* residual) const {
        return AccelerometerStepCostBase::computeResidualsGeneric<false, false, false, false, T>
                (t0, r1, t1, t2, g, residual,
                 nullptr, nullptr, nullptr, nullptr,
                 Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity());
    }

protected:
};

template<>
class AccelerometerStepCost<AccelerometerStepCostFlags::AccScale>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& aGainJacobian,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _aGainJacobian(aGainJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const ascale,
                    const T* const g,
                    T* residual) const {
        return AccelerometerStepCostBase::computeResidualsGeneric<false, false, false, true, T>
                (t0, r1, t1, t2, g, residual,
                 nullptr, nullptr, ascale, nullptr,
                 Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(), _aGainJacobian);
    }

protected:

    Eigen::Matrix3d _aGainJacobian;
};

template<>
class AccelerometerStepCost<AccelerometerStepCostFlags::AccBias>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& aBiasJacobian,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _aBiasJacobian(aBiasJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const abias,
                    const T* const g,
                    T* residual) const {
        return AccelerometerStepCostBase::computeResidualsGeneric<false, false, true, false, T>
                (t0, r1, t1, t2, g, residual,
                 nullptr, nullptr, nullptr, abias,
                 Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(), _aBiasJacobian, Eigen::Matrix3d::Identity());
    }

protected:

    Eigen::Matrix3d _aBiasJacobian;
};

template<>
class AccelerometerStepCost<AccelerometerStepCostFlags::AccBias|AccelerometerStepCostFlags::AccScale>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& aBiasJacobian,
                          Eigen::Matrix3d const& aGainJacobian,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _aBiasJacobian(aBiasJacobian),
        _aGainJacobian(aGainJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const ascale,
                    const T* const abias,
                    const T* const g,
                    T* residual) const {
        return AccelerometerStepCostBase::computeResidualsGeneric<false, false, true, true, T>
                (t0, r1, t1, t2, g, residual,
                 nullptr, nullptr, ascale, abias,
                 Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(), _aBiasJacobian, _aGainJacobian);
    }

protected:

    Eigen::Matrix3d _aBiasJacobian;
    Eigen::Matrix3d _aGainJacobian;
};

template<>
class AccelerometerStepCost<AccelerometerStepCostFlags::GyroScale>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& gGainJacobian,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _gGainJacobian(gGainJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const gscale,
                    const T* const g,
                    T* residual) const {
        return AccelerometerStepCostBase::computeResidualsGeneric<false, true, false, false, T>
                (t0, r1, t1, t2, g, residual,
                 gscale, nullptr, nullptr, nullptr,
                 Eigen::Matrix3d::Identity(), _gGainJacobian, Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity());
    }

protected:

    Eigen::Matrix3d _gGainJacobian;
};

template<>
class AccelerometerStepCost<AccelerometerStepCostFlags::GyroScale|AccelerometerStepCostFlags::AccBias>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& gGainJacobian,
                          Eigen::Matrix3d const& aBiasJacobian,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _gGainJacobian(gGainJacobian),
        _aBiasJacobian(aBiasJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const gscale,
                    const T* const abias,
                    const T* const g,
                    T* residual) const {
        return AccelerometerStepCostBase::computeResidualsGeneric<false, true, true, false, T>
                (t0, r1, t1, t2, g, residual,
                 gscale, nullptr, nullptr, abias,
                 Eigen::Matrix3d::Identity(), _gGainJacobian, _aBiasJacobian, Eigen::Matrix3d::Identity());
    }

protected:

    Eigen::Matrix3d _gGainJacobian;
    Eigen::Matrix3d _aBiasJacobian;
};

template<>
class AccelerometerStepCost<AccelerometerStepCostFlags::GyroScale|AccelerometerStepCostFlags::AccScale>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& gGainJacobian,
                          Eigen::Matrix3d const& aGainJacobian,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _gGainJacobian(gGainJacobian),
        _aGainJacobian(aGainJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const gscale,
                    const T* const ascale,
                    const T* const g,
                    T* residual) const {
        return AccelerometerStepCostBase::computeResidualsGeneric<false, true, false, true, T>
                (t0, r1, t1, t2, g, residual,
                 gscale, nullptr, ascale, nullptr,
                 Eigen::Matrix3d::Identity(), _gGainJacobian, Eigen::Matrix3d::Identity(), _aGainJacobian);
    }

protected:

    Eigen::Matrix3d _gGainJacobian;
    Eigen::Matrix3d _aGainJacobian;
};

template<>
class AccelerometerStepCost<AccelerometerStepCostFlags::GyroScale|
        AccelerometerStepCostFlags::AccScale|
        AccelerometerStepCostFlags::AccBias>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& gGainJacobian,
                          Eigen::Matrix3d const& aBiasJacobian,
                          Eigen::Matrix3d const& aGainJacobian,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _gGainJacobian(gGainJacobian),
        _aBiasJacobian(aBiasJacobian),
        _aGainJacobian(aGainJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const gscale,
                    const T* const ascale,
                    const T* const abias,
                    const T* const g,
                    T* residual) const {
        return AccelerometerStepCostBase::computeResidualsGeneric<false, true, true, true, T>
                (t0, r1, t1, t2, g, residual,
                 gscale, nullptr, ascale, abias,
                 Eigen::Matrix3d::Identity(), _gGainJacobian, _aBiasJacobian, _aGainJacobian);
    }

protected:

    Eigen::Matrix3d _gGainJacobian;
    Eigen::Matrix3d _aBiasJacobian;
    Eigen::Matrix3d _aGainJacobian;
};


template<>
class AccelerometerStepCost<AccelerometerStepCostFlags::GyroBias>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& gBiasJacobian,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _gBiasJacobian(gBiasJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const gbias,
                    const T* const g,
                    T* residual) const {
        return AccelerometerStepCostBase::computeResidualsGeneric<true, false, false, false, T>
                (t0, r1, t1, t2, g, residual,
                 nullptr, gbias, nullptr, nullptr,
                 _gBiasJacobian, Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity());
    }

protected:

    Eigen::Matrix3d _gBiasJacobian;
};

template<>
class AccelerometerStepCost<AccelerometerStepCostFlags::GyroBias|AccelerometerStepCostFlags::AccBias>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& gBiasJacobian,
                          Eigen::Matrix3d const& aBiasJacobian,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _gBiasJacobian(gBiasJacobian),
        _aBiasJacobian(aBiasJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const gbias,
                    const T* const abias,
                    const T* const g,
                    T* residual) const {
        return AccelerometerStepCostBase::computeResidualsGeneric<true, false, true, false, T>
                (t0, r1, t1, t2, g, residual,
                 nullptr, gbias, nullptr, abias,
                 _gBiasJacobian, Eigen::Matrix3d::Identity(), _aBiasJacobian, Eigen::Matrix3d::Identity());
    }

protected:

    Eigen::Matrix3d _gBiasJacobian;
    Eigen::Matrix3d _aBiasJacobian;
};

template<>
class AccelerometerStepCost<AccelerometerStepCostFlags::GyroBias|AccelerometerStepCostFlags::AccScale>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& gBiasJacobian,
                          Eigen::Matrix3d const& aGainJacobian,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _gBiasJacobian(gBiasJacobian),
        _aGainJacobian(aGainJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const gbias,
                    const T* const ascale,
                    const T* const g,
                    T* residual) const {
        return AccelerometerStepCostBase::computeResidualsGeneric<true, true, true, true, T>
                (t0, r1, t1, t2, g, residual,
                 nullptr, gbias, ascale, nullptr,
                 _gBiasJacobian, Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(), _aGainJacobian);
    }

protected:

    Eigen::Matrix3d _gBiasJacobian;
    Eigen::Matrix3d _aGainJacobian;
};

template<>
class AccelerometerStepCost<AccelerometerStepCostFlags::GyroBias|
        AccelerometerStepCostFlags::AccBias|
        AccelerometerStepCostFlags::AccScale>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& gBiasJacobian,
                          Eigen::Matrix3d const& aBiasJacobian,
                          Eigen::Matrix3d const& aGainJacobian,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _gBiasJacobian(gBiasJacobian),
        _aBiasJacobian(aBiasJacobian),
        _aGainJacobian(aGainJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const gbias,
                    const T* const ascale,
                    const T* const abias,
                    const T* const g,
                    T* residual) const {
        return AccelerometerStepCostBase::computeResidualsGeneric<true, false, true, true, T>
                (t0, r1, t1, t2, g, residual, nullptr, gbias, ascale, abias,
                 _gBiasJacobian, Eigen::Matrix3d::Identity(), _aBiasJacobian, _aGainJacobian);
    }

protected:

    Eigen::Matrix3d _gBiasJacobian;
    Eigen::Matrix3d _aBiasJacobian;
    Eigen::Matrix3d _aGainJacobian;
};


template<>
class AccelerometerStepCost<AccelerometerStepCostFlags::GyroBias|AccelerometerStepCostFlags::GyroScale>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& gBiasJacobian,
                          Eigen::Matrix3d const& gGainJacobian,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _gBiasJacobian(gBiasJacobian),
        _gGainJacobian(gGainJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const gscale,
                    const T* const gbias,
                    const T* const g,
                    T* residual) const {
        return AccelerometerStepCostBase::computeResidualsGeneric<true, true, false, false, T>
                (t0, r1, t1, t2, g, residual,
                 gscale, gbias, nullptr, nullptr,
                 _gBiasJacobian, _gGainJacobian, Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity());
    }

protected:

    Eigen::Matrix3d _gBiasJacobian;
    Eigen::Matrix3d _gGainJacobian;
};

template<>
class AccelerometerStepCost<AccelerometerStepCostFlags::GyroBias|
        AccelerometerStepCostFlags::GyroScale|
        AccelerometerStepCostFlags::AccBias>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& gBiasJacobian,
                          Eigen::Matrix3d const& gGainJacobian,
                          Eigen::Matrix3d const& aBiasJacobian,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _gBiasJacobian(gBiasJacobian),
        _gGainJacobian(gGainJacobian),
        _aBiasJacobian(aBiasJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const gscale,
                    const T* const gbias,
                    const T* const abias,
                    const T* const g,
                    T* residual) const {
        return AccelerometerStepCostBase::computeResidualsGeneric<true, true, true, false, T>
                (t0, r1, t1, t2, g, residual,
                 gscale, gbias, nullptr, abias,
                 _gBiasJacobian, _gGainJacobian, _aBiasJacobian, Eigen::Matrix3d::Identity());
    }

protected:

    Eigen::Matrix3d _gBiasJacobian;
    Eigen::Matrix3d _gGainJacobian;
    Eigen::Matrix3d _aBiasJacobian;
};

template<>
class AccelerometerStepCost<AccelerometerStepCostFlags::GyroBias|
        AccelerometerStepCostFlags::GyroScale|
        AccelerometerStepCostFlags::AccScale>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& gBiasJacobian,
                          Eigen::Matrix3d const& gGainJacobian,
                          Eigen::Matrix3d const& aGainJacobian,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _gBiasJacobian(gBiasJacobian),
        _gGainJacobian(gGainJacobian),
        _aGainJacobian(aGainJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const gscale,
                    const T* const gbias,
                    const T* const ascale,
                    const T* const g,
                    T* residual) const {
        return AccelerometerStepCostBase::computeResidualsGeneric<true, true, false, true, T>
                (t0, r1, t1, t2, g, residual,
                 gscale, gbias, ascale, nullptr,
                 _gBiasJacobian, _gGainJacobian, Eigen::Matrix3d::Identity(), _aGainJacobian);
    }

protected:

    Eigen::Matrix3d _gBiasJacobian;
    Eigen::Matrix3d _gGainJacobian;
    Eigen::Matrix3d _aGainJacobian;
};

template<>
class AccelerometerStepCost<AccelerometerStepCostFlags::GyroBias|
        AccelerometerStepCostFlags::GyroScale|
        AccelerometerStepCostFlags::AccBias|
        AccelerometerStepCostFlags::AccScale>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& gBiasJacobian,
                          Eigen::Matrix3d const& gGainJacobian,
                          Eigen::Matrix3d const& aBiasJacobian,
                          Eigen::Matrix3d const& aGainJacobian,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _gBiasJacobian(gBiasJacobian),
        _gGainJacobian(gGainJacobian),
        _aBiasJacobian(aBiasJacobian),
        _aGainJacobian(aGainJacobian)
    {

    }

    template <typename T>
    bool operator()(const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const gscale,
                    const T* const gbias,
                    const T* const ascale,
                    const T* const abias,
                    const T* const g,
                    T* residual) const {
        return AccelerometerStepCostBase::computeResidualsGeneric<true, true, true, true, T>
                (t0, r1, t1, t2, g, residual,
                 gscale, gbias, ascale, abias,
                 _gBiasJacobian, _gGainJacobian, _aBiasJacobian, _aGainJacobian);
    }

protected:

    Eigen::Matrix3d _gBiasJacobian;
    Eigen::Matrix3d _gGainJacobian;
    Eigen::Matrix3d _aBiasJacobian;
    Eigen::Matrix3d _aGainJacobian;
};

template<int accCostFlags, template <typename T> typename Decorator, typename Tgyro, typename Tacc, typename TimeT, typename ... DecoratorAdditionalArgsT>
Decorator<AccelerometerStepCost<accCostFlags>>* AccelerometerStepCostBase::getIntegratedIMUDiff(IndexedTimeSequence<Tgyro, TimeT> const& GyroData,
                                                                                                IndexedTimeSequence<Tacc, TimeT> const& AccData,
                                                                                                TimeT t0,
                                                                                                TimeT t1,
                                                                                                TimeT t2,
                                                                                                DecoratorAdditionalArgsT ... decoratorAdditionalArgs) {

    using Traits = AccelerometerStepCostTraits<accCostFlags>;

    IntegratedAccSegment integrated = preIntegrateAccSegment(GyroData, AccData, t0, t1, t2);

    if constexpr (Traits::WGBias) {
        if constexpr (Traits::WGScale) {

            if constexpr (Traits::WABias) {
                if constexpr (Traits::WAScale) {
                    return new Decorator<AccelerometerStepCost<AccelerometerStepCostFlags::GyroBias|
                                                               AccelerometerStepCostFlags::GyroScale|
                                                               AccelerometerStepCostFlags::AccBias|
                                                               AccelerometerStepCostFlags::AccScale>>(
                        decoratorAdditionalArgs ... ,
                        integrated.speedDelta,
                        integrated.gyroBiasJacobian,
                        integrated.gyroGainJacobian,
                        integrated.accBiasJacobian,
                        integrated.accGainJacobian,
                        integrated.dt1,
                        integrated.dt2);
                } else {
                    return new Decorator<AccelerometerStepCost<AccelerometerStepCostFlags::GyroBias|
                                                               AccelerometerStepCostFlags::GyroScale|
                                                               AccelerometerStepCostFlags::AccBias>>(
                        decoratorAdditionalArgs ... ,
                        integrated.speedDelta,
                        integrated.gyroBiasJacobian,
                        integrated.gyroGainJacobian,
                        integrated.accBiasJacobian,
                        integrated.dt1,
                        integrated.dt2);
                }
            } else {
                if constexpr (Traits::WAScale) {
                    return new Decorator<AccelerometerStepCost<AccelerometerStepCostFlags::GyroBias|
                                                               AccelerometerStepCostFlags::GyroScale|
                                                               AccelerometerStepCostFlags::AccScale>>(
                        decoratorAdditionalArgs ... ,
                        integrated.speedDelta,
                        integrated.gyroBiasJacobian,
                        integrated.gyroGainJacobian,
                        integrated.accGainJacobian,
                        integrated.dt1,
                        integrated.dt2);
                } else {
                    return new Decorator<AccelerometerStepCost<AccelerometerStepCostFlags::GyroBias|
                                                               AccelerometerStepCostFlags::GyroScale>>(
                        decoratorAdditionalArgs ... ,
                        integrated.speedDelta,
                        integrated.gyroBiasJacobian,
                        integrated.gyroGainJacobian,
                        integrated.dt1,
                        integrated.dt2);
                }
            }

        } else {

            if constexpr (Traits::WABias) {
                if constexpr (Traits::WAScale) {
                    return new Decorator<AccelerometerStepCost<AccelerometerStepCostFlags::GyroBias|
                                                               AccelerometerStepCostFlags::AccBias|
                                                               AccelerometerStepCostFlags::AccScale>>(
                        decoratorAdditionalArgs ... ,
                        integrated.speedDelta,
                        integrated.gyroBiasJacobian,
                        integrated.accBiasJacobian,
                        integrated.accGainJacobian,
                        integrated.dt1,
                        integrated.dt2);
                } else {
                    return new Decorator<AccelerometerStepCost<AccelerometerStepCostFlags::GyroBias|
                                                               AccelerometerStepCostFlags::AccBias>>(
                        decoratorAdditionalArgs ... ,
                        integrated.speedDelta,
                        integrated.gyroBiasJacobian,
                        integrated.accBiasJacobian,
                        integrated.dt1,
                        integrated.dt2);
                }
            } else {
                if constexpr (Traits::WAScale) {
                    return new Decorator<AccelerometerStepCost<AccelerometerStepCostFlags::GyroBias|
                                                               AccelerometerStepCostFlags::AccScale>>(
                        decoratorAdditionalArgs ... ,
                        integrated.speedDelta,
                        integrated.gyroBiasJacobian,
                        integrated.accGainJacobian,
                        integrated.dt1,
                        integrated.dt2);
                } else {
                    return new Decorator<AccelerometerStepCost<AccelerometerStepCostFlags::GyroBias>>
                            (
                        decoratorAdditionalArgs ... ,
                        integrated.speedDelta,
                        integrated.gyroBiasJacobian,
                        integrated.dt1,
                        integrated.dt2);
                }
            }

        }
    } else {
        if constexpr (Traits::WGScale) {

            if constexpr (Traits::WABias) {
                if constexpr (Traits::WAScale) {
                    return new Decorator<AccelerometerStepCost<AccelerometerStepCostFlags::GyroScale|
                                                               AccelerometerStepCostFlags::AccBias|
                                                               AccelerometerStepCostFlags::AccScale>>(
                        decoratorAdditionalArgs ... ,
                        integrated.speedDelta,
                        integrated.gyroGainJacobian,
                        integrated.accBiasJacobian,
                        integrated.accGainJacobian,
                        integrated.dt1,
                        integrated.dt2);
                } else {
                    return new Decorator<AccelerometerStepCost<AccelerometerStepCostFlags::GyroScale|
                                                               AccelerometerStepCostFlags::AccBias>>(
                        decoratorAdditionalArgs ... ,
                        integrated.speedDelta,
                        integrated.gyroGainJacobian,
                        integrated.accBiasJacobian,
                        integrated.dt1,
                        integrated.dt2);
                }
            } else {
                if constexpr (Traits::WAScale) {
                    return new Decorator<AccelerometerStepCost<AccelerometerStepCostFlags::GyroScale|
                                                               AccelerometerStepCostFlags::AccScale>>(
                        decoratorAdditionalArgs ... ,
                        integrated.speedDelta,
                        integrated.gyroGainJacobian,
                        integrated.accGainJacobian,
                        integrated.dt1,
                        integrated.dt2);
                } else {
                    return new Decorator<AccelerometerStepCost<AccelerometerStepCostFlags::GyroScale>>
                            (
                        decoratorAdditionalArgs ... ,
                        integrated.speedDelta,
                        integrated.gyroGainJacobian,
                        integrated.dt1,
                        integrated.dt2);
                }
            }

        } else {

            if constexpr (Traits::WABias) {
                if constexpr (Traits::WAScale) {
                    return new Decorator<AccelerometerStepCost<AccelerometerStepCostFlags::AccBias|
                                                               AccelerometerStepCostFlags::AccScale>>(
                        decoratorAdditionalArgs ... ,
                        integrated.speedDelta,
                        integrated.accBiasJacobian,
                        integrated.accGainJacobian,
                        integrated.dt1,
                        integrated.dt2);
                } else {
                    return new Decorator<AccelerometerStepCost<AccelerometerStepCostFlags::AccBias>>
                            (
                        decoratorAdditionalArgs ... ,
                        integrated.speedDelta,
                        integrated.accBiasJacobian,
                        integrated.dt1,
                        integrated.dt2);
                }
            } else {
                if constexpr (Traits::WAScale) {
                    return new Decorator<AccelerometerStepCost<AccelerometerStepCostFlags::AccScale>>
                        (
                        decoratorAdditionalArgs ... ,
                        integrated.speedDelta,
                        integrated.accGainJacobian,
                        integrated.dt1,
                        integrated.dt2);
                } else {
                    return new Decorator<AccelerometerStepCost<AccelerometerStepCostFlags::NoBias>>
                            (
                        decoratorAdditionalArgs ... ,
                        integrated.speedDelta,
                        integrated.dt1,
                        integrated.dt2);
                }
            }
        }
    }

}

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMUSTEPCOST_H
