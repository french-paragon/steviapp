#ifndef STEREOVISIONAPP_IMUSTEPCOST_H
#define STEREOVISIONAPP_IMUSTEPCOST_H

#include <Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>

#include "../../vision/indexed_timed_sequence.h"

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

    template<bool WBias, bool WScale, typename Tgyro, typename TimeT>
    static GyroStepCost<WBias,WScale>* getIntegratedIMUDiff(IndexedTimeSequence<Tgyro, TimeT> const& GyroData,
                                              TimeT t0,
                                              TimeT t1);

    GyroStepCostBase(Eigen::Matrix3d const& attitudeDelta,
                 double delta_t);

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

        using M3T = Eigen::Matrix<T,3,3>;
        using V3T = Eigen::Matrix<T,3,1>;

        V3T vr0(r0[0], r0[1], r0[2]);
        V3T vr1(r1[0], r1[1], r1[2]);

        M3T Rr0 = StereoVision::Geometry::rodriguezFormula<T>(vr0); //R body2world at time 0
        M3T Rr1 = StereoVision::Geometry::rodriguezFormula<T>(vr1); //R body2world at time 1

        M3T closure = Rr1.transpose() * Rr0 * _attitudeDelta;
        V3T res = StereoVision::Geometry::inverseRodriguezFormula<T>(closure);

        residual[0] = res[0];
        residual[1] = res[1];
        residual[2] = res[2];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1]) or !ceres::IsFinite(residual[2])) {
            std::cout << "Error in GyroStepCost cost computation" << std::endl;
        }
#endif

        return true;

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

        using M3T = Eigen::Matrix<T,3,3>;
        using V3T = Eigen::Matrix<T,3,1>;

        V3T vr0(r0[0], r0[1], r0[2]);
        V3T vr1(r1[0], r1[1], r1[2]);

        V3T g(gyroScale[0] - T(1), gyroScale[1] - T(1), gyroScale[2] - T(1));

        M3T Rr0 = StereoVision::Geometry::rodriguezFormula<T>(vr0); //R body2world at time 0
        M3T Rr1 = StereoVision::Geometry::rodriguezFormula<T>(vr1); //R body2world at time 1

        M3T Jg = _scaleJacobian.cast<T>();
        M3T Rg = StereoVision::Geometry::rodriguezFormula<T>(Jg*g);

        M3T closure = Rr1.transpose() * Rr0 * _attitudeDelta*Rg;
        V3T res = StereoVision::Geometry::inverseRodriguezFormula<T>(closure);

        residual[0] = res[0];
        residual[1] = res[1];
        residual[2] = res[2];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1]) or !ceres::IsFinite(residual[2])) {
            std::cout << "Error in GyroStepCost cost computation" << std::endl;
        }
#endif

        return true;

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

        using M3T = Eigen::Matrix<T,3,3>;
        using V3T = Eigen::Matrix<T,3,1>;

        V3T vr0(r0[0], r0[1], r0[2]);
        V3T vr1(r1[0], r1[1], r1[2]);

        V3T b(gyroBias[0], gyroBias[1], gyroBias[2]);

        M3T Rr0 = StereoVision::Geometry::rodriguezFormula<T>(vr0); //R body2world at time 0
        M3T Rr1 = StereoVision::Geometry::rodriguezFormula<T>(vr1); //R body2world at time 1

        M3T Jb = _biasJacobian.cast<T>();
        M3T Rb = StereoVision::Geometry::rodriguezFormula<T>(Jb*b);

        M3T closure = Rr1.transpose() * Rr0 * _attitudeDelta*Rb;
        V3T res = StereoVision::Geometry::inverseRodriguezFormula<T>(closure);

        residual[0] = res[0];
        residual[1] = res[1];
        residual[2] = res[2];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1]) or !ceres::IsFinite(residual[2])) {
            std::cout << "Error in GyroStepCost cost computation" << std::endl;
        }
#endif

        return true;

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

        using M3T = Eigen::Matrix<T,3,3>;
        using V3T = Eigen::Matrix<T,3,1>;

        V3T vr0(r0[0], r0[1], r0[2]);
        V3T vr1(r1[0], r1[1], r1[2]);

        V3T g(gyroScale[0] - T(1), gyroScale[1] - T(1), gyroScale[2] - T(1));
        V3T b(gyroBias[0], gyroBias[1], gyroBias[2]);

        M3T Rr0 = StereoVision::Geometry::rodriguezFormula<T>(vr0); //R body2world at time 0
        M3T Rr1 = StereoVision::Geometry::rodriguezFormula<T>(vr1); //R body2world at time 1

        M3T Jg = _scaleJacobian.cast<T>();
        M3T Jb = _biasJacobian.cast<T>();

        M3T Rbg = StereoVision::Geometry::rodriguezFormula<T>(Jg*g + Jb*b);

        M3T closure = Rr1.transpose() * Rr0 * _attitudeDelta*Rbg;
        V3T res = StereoVision::Geometry::inverseRodriguezFormula<T>(closure);

        residual[0] = res[0];
        residual[1] = res[1];
        residual[2] = res[2];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1]) or !ceres::IsFinite(residual[2])) {
            std::cout << "Error in GyroStepCost cost computation" << std::endl;
        }
#endif

        return true;

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

        Eigen::Matrix3d newRcurrent2initial = dR*ret.attitudeDelta;

        //re-constaint as R mat for numerical stability
        Eigen::Vector3d logR = StereoVision::Geometry::inverseRodriguezFormula(newRcurrent2initial);
        ret.attitudeDelta = StereoVision::Geometry::rodriguezFormula(logR);
    }

    return ret;



}

template<bool WBias, bool WScale, typename Tgyro, typename TimeT>
GyroStepCost<WBias,WScale>* GyroStepCostBase::getIntegratedIMUDiff(IndexedTimeSequence<Tgyro, TimeT> const& GyroData,
                                              TimeT t0,
                                              TimeT t1) {

    GyroStepCostBase* ret;

    IntegratedGyroSegment pre_integrated = preIntegrateGyroSegment<WBias, WScale, Tgyro, TimeT>(GyroData, t0, t1);

    if (WBias and WScale) {
            ret = new StereoVisionApp::GyroStepCost<true, true>(pre_integrated.attitudeDelta,
                                                                pre_integrated.gainJacobian,
                                                                pre_integrated.biasJacobian,
                                                                pre_integrated.dt);
            return static_cast<GyroStepCost<WBias, WScale>*>(ret);
        } else if (WBias) {
            ret = new StereoVisionApp::GyroStepCost<true, false>(pre_integrated.attitudeDelta,
                                                                 pre_integrated.biasJacobian,
                                                                 pre_integrated.dt);
            return static_cast<GyroStepCost<WBias, WScale>*>(ret);
        } else if (WScale) {
            ret = new StereoVisionApp::GyroStepCost<false, true>(pre_integrated.attitudeDelta,
                                                                 pre_integrated.gainJacobian,
                                                                 pre_integrated.dt);
            return static_cast<GyroStepCost<WBias, WScale>*>(ret);
        }

    ret = new StereoVisionApp::GyroStepCost<false, false>(pre_integrated.attitudeDelta,
                                                          pre_integrated.dt);

    return static_cast<GyroStepCost<WBias, WScale>*>(ret);


}

template<bool WBias, bool WScale>
class AccelerometerStepCost;

/*!
 * \brief The AccelerometerStepCostBase class represent a measurement by an accelerometer between three poses.
 *
 * The benefits of doing so is that is that we do not need the speed as a learnable parameter, only the pose.
 *
 * The speed delta is assumed to be int_n^{n+1} int_n^{t} w(t') dt' f(t) dt
 * Such that v_{n+1} = v{n} - g dt + R_n int_n^{n+1} int_n^{t} w(t') dt' f(t) dt
 *
 * where v_{n+1} and v{n} a speed expressed in world (local) frame,
 * R_n is the attitude (R from body to world) at time n,
 * g is the gravity,
 * f(t) is the measured specific force and
 * int_n^{t} w(t') dt' is the estimated relative orientation integrated from the gyro measurements.
 *
 * In the case a bias is present, the model becomes int_n^{n+1} int_n^{t} w(t') dt' (f(t) + b) dt.
 * By linearity, the correction can be isolated and simplified as int_n^{n+1} int_n^{t} w(t') dt' dt b.
 * int_n^{n+1} int_n^{t} w(t') dt' dt is refered to as the _biasAvgOrientAndScale.
 *
 * Poses are assumed to be the Body2World (or Body2Local if a local frame is used).
 *
 * the three poses under consideration are at t0, t1, t2.
 * The speeds will be computed at time (t0 + t1)/2 = n and (t1 + t2)/2 = n+1
 * The integration has to be done between these times.
 * The getIntegratedIMUDiff function is here to help.
 *
 * In addition the cost is configure with delta_t1 = t1 - t0 and delta_t2 = t2 - t1
 *
 * The class has different implementation of the cost, depending on the stochastic model is implemented in the child class.
 *
 *
 */
class AccelerometerStepCostBase {

public:

    template<bool WBias, bool WScale, typename Tgyro, typename Tacc, typename TimeT>
    static AccelerometerStepCost<WBias, WScale>* getIntegratedIMUDiff(IndexedTimeSequence<Tgyro, TimeT> const& GyroData,
                                                        IndexedTimeSequence<Tacc, TimeT> const& AccData,
                                                        TimeT t0,
                                                        TimeT t1,
                                                        TimeT t2) {

        double delta_t1 = t1 - t0;
        double delta_t2 = t2 - t1;

        Eigen::Vector3d speedDelta = Eigen::Vector3d::Zero();

        double ti = (t1 + t0)/2;
        double tf = (t2 + t1)/2;

        auto imuAccData = AccData.getValuesInBetweenTimes(ti, tf);
        auto imuGyroData = GyroData.getValuesInBetweenTimes(ti, tf);

        Eigen::Matrix3d Rcurrent2initial = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d BiasAvgOrientAndScale = Eigen::Matrix3d::Zero();

        double t_acc = 0;
        double t_gyro = 0;

        int idx_acc = 0;

        for (int j = 0; j < imuGyroData.size(); j++) {

            double dt = imuGyroData[j].dt;

            while (t_acc < t_gyro + dt/2) {
                double dt_acc = imuAccData[idx_acc].dt;

                Eigen::Vector3d acceleration_local(imuAccData[idx_acc].val[0], imuAccData[idx_acc].val[1], imuAccData[idx_acc].val[2]);

                speedDelta += Rcurrent2initial*acceleration_local*dt_acc;

                t_acc += dt_acc;
                idx_acc++;
            }

            Eigen::Vector3d dr(imuGyroData[j].val[0], imuGyroData[j].val[1], imuGyroData[j].val[2]);
            dr *= dt;

            Eigen::Matrix3d dR = StereoVision::Geometry::rodriguezFormula(dr);

            Rcurrent2initial = dR*Rcurrent2initial;
            BiasAvgOrientAndScale += Rcurrent2initial*dt;

            //re-constaint as R mat for numerical stability
            Eigen::Vector3d logR = StereoVision::Geometry::inverseRodriguezFormula(Rcurrent2initial);
            Rcurrent2initial = StereoVision::Geometry::rodriguezFormula(logR);

            t_gyro += dt;

        }

        #ifndef NDEBUG
        bool speedDeltaFinite = speedDelta.array().isFinite().all();

        bool biasAvgOrientAndScaleFinite = BiasAvgOrientAndScale.array().isFinite().all();

        if (!biasAvgOrientAndScaleFinite or !speedDeltaFinite) {
            std::cerr << "Error when pre-integrating Accelerometer" << std::endl;
        }
        #endif

        AccelerometerStepCostBase* ret;

        if (WBias) {
            ret = new StereoVisionApp::AccelerometerStepCost<true, WScale>(speedDelta, BiasAvgOrientAndScale,  delta_t1, delta_t2);
        } else {
            ret = new StereoVisionApp::AccelerometerStepCost<false, WScale>(speedDelta,  delta_t1, delta_t2);
        }

        return static_cast<StereoVisionApp::AccelerometerStepCost<WBias, WScale>*>(ret);

    }

    AccelerometerStepCostBase(Eigen::Vector3d const& speedDelta,
                              double delta_t1,
                              double delta_t2);

protected:

    Eigen::Vector3d _speedDelta; // int_n^{n+1} int_n^{t} w(t') dt' f(t) dt
    double _delta_t1;
    double _delta_t2;
    double _delta_tv;

};

template<>
class AccelerometerStepCost<false, false>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2) {

    }

    template <typename T>
    bool operator()(const T* const r0,
                    const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const g,
                    T* residual) const {


        using M3T = Eigen::Matrix<T,3,3>;
        using V3T = Eigen::Matrix<T,3,1>;

        V3T vr0(r0[0], r0[1], r0[2]);
        V3T vr1(r1[0], r1[1], r1[2]);

        V3T vt0(t0[0], t0[1], t0[2]);
        V3T vt1(t1[0], t1[1], t1[2]);
        V3T vt2(t2[0], t2[1], t2[2]);

        V3T vg(g[0], g[1], g[2]);

        V3T vv0 = (vt1 - vt0)/T(_delta_t1); //avg speed in first interval
        V3T vv1 = (vt2 - vt1)/T(_delta_t2); //avg speed in second interval

        M3T Rr0 = StereoVision::Geometry::rodriguezFormula<T>(vr0);
        M3T Rr1 = StereoVision::Geometry::rodriguezFormula<T>(vr1);
        M3T deltaR = Rr1*Rr0.transpose();
        V3T vrv0 = StereoVision::Geometry::inverseRodriguezFormula<T>(deltaR)*T(0.5);
        M3T Rrv0 = StereoVision::Geometry::rodriguezFormula<T>(vrv0)*Rr0;

        //platform2world orientation at (t0 + t1)/2

        T spdx(_speedDelta[0]);
        T spdy(_speedDelta[1]);
        T spdz(_speedDelta[2]);
        V3T vSpeedDelta(spdx, spdy, spdz);
        V3T gCorr = vg*T(_delta_tv);
        V3T alignedSpeedDelta = Rrv0*vSpeedDelta;
        V3T corr = gCorr + alignedSpeedDelta;
        V3T pred = vv0 + corr;
        V3T err = vv1 - pred;

        residual[0] = err[0];
        residual[1] = err[1];
        residual[2] = err[2];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1]) or !ceres::IsFinite(residual[2])) {
            std::cout << "Error in AccelerometerStepCost cost computation" << std::endl;
        }
#endif

        return true;

    }
};

template<>
class AccelerometerStepCost<false, true>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2) {

    }

    template <typename T>
    bool operator()(const T* const r0,
                    const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const g,
                    const T* const scale,
                    T* residual) const {


        using M3T = Eigen::Matrix<T,3,3>;
        using V3T = Eigen::Matrix<T,3,1>;

        V3T vr0(r0[0], r0[1], r0[2]);
        V3T vr1(r1[0], r1[1], r1[2]);

        V3T vt0(t0[0], t0[1], t0[2]);
        V3T vt1(t1[0], t1[1], t1[2]);
        V3T vt2(t2[0], t2[1], t2[2]);

        V3T vg(g[0], g[1], g[2]);

        V3T vv0 = (vt1 - vt0)/T(_delta_t1); //avg speed in first interval
        V3T vv1 = (vt2 - vt1)/T(_delta_t2); //avg speed in second interval

        M3T Rr0 = StereoVision::Geometry::rodriguezFormula<T>(vr0);
        M3T Rr1 = StereoVision::Geometry::rodriguezFormula<T>(vr1);
        M3T deltaR = Rr1*Rr0.transpose();
        V3T vrv0 = StereoVision::Geometry::inverseRodriguezFormula<T>(deltaR)*T(0.5);
        M3T Rrv0 = StereoVision::Geometry::rodriguezFormula<T>(vrv0)*Rr0;

        //platform2world orientation at (t0 + t1)/2

        T spdx(_speedDelta[0]);
        T spdy(_speedDelta[1]);
        T spdz(_speedDelta[2]);
        V3T vSpeedDelta(spdx, spdy, spdz);
        V3T gCorr = vg*T(_delta_tv);
        V3T alignedSpeedDelta = (*scale)*Rrv0*vSpeedDelta;
        V3T corr = gCorr + alignedSpeedDelta;
        V3T pred = vv0 + corr;
        V3T err = vv1 - pred;

        residual[0] = err[0];
        residual[1] = err[1];
        residual[2] = err[2];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1]) or !ceres::IsFinite(residual[2])) {
            std::cout << "Error in AccelerometerStepCost cost computation" << std::endl;
        }
#endif

        return true;

    }
};

template<>
class AccelerometerStepCost<true, false>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& biasAvgOrientAndScale,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _biasAvgOrientAndScale(biasAvgOrientAndScale)
    {

    }

    template <typename T>
    bool operator()(const T* const r0,
                    const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const g,
                    const T* const bias,
                    T* residual) const {


        using M3T = Eigen::Matrix<T,3,3>;
        using V3T = Eigen::Matrix<T,3,1>;

        V3T vr0(r0[0], r0[1], r0[2]);
        V3T vr1(r1[0], r1[1], r1[2]);

        V3T vt0(t0[0], t0[1], t0[2]);
        V3T vt1(t1[0], t1[1], t1[2]);
        V3T vt2(t2[0], t2[1], t2[2]);

        V3T vg(g[0], g[1], g[2]);

        V3T vBias(bias[0], bias[1], bias[2]);

        V3T vv0 = (vt1 - vt0)/T(_delta_t1); //avg speed in first interval
        V3T vv1 = (vt2 - vt1)/T(_delta_t2); //avg speed in second interval

        M3T Rr0 = StereoVision::Geometry::rodriguezFormula<T>(vr0);
        M3T Rr1 = StereoVision::Geometry::rodriguezFormula<T>(vr1);
        M3T deltaR = Rr1*Rr0.transpose();
        V3T vrv0 = StereoVision::Geometry::inverseRodriguezFormula<T>(deltaR)*T(0.5);
        M3T Rrv0 = StereoVision::Geometry::rodriguezFormula<T>(vrv0)*Rr0;

        M3T biasTransform;

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                biasTransform(i,j) = T(_biasAvgOrientAndScale(i,j));
            }
        }

        //platform2world orientation at (t0 + t1)/2

        T spdx(_speedDelta[0]);
        T spdy(_speedDelta[1]);
        T spdz(_speedDelta[2]);
        V3T vSpeedDelta(spdx, spdy, spdz);
        V3T gCorr = vg*T(_delta_tv);
        V3T alignedSpeedDelta = Rrv0*(vSpeedDelta - biasTransform*vBias);
        V3T corr = gCorr + alignedSpeedDelta;
        V3T pred = vv0 + corr;
        V3T err = vv1 - pred;

        residual[0] = err[0];
        residual[1] = err[1];
        residual[2] = err[2];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1]) or !ceres::IsFinite(residual[2])) {
            std::cout << "Error in AccelerometerStepCost cost computation" << std::endl;
        }
#endif

        return true;

    }

protected:
    Eigen::Matrix3d _biasAvgOrientAndScale; // int_n^{n+1} int_n^{t} w(t') dt' dt
};

template<>
class AccelerometerStepCost<true, true>: public AccelerometerStepCostBase {

public:

    AccelerometerStepCost(Eigen::Vector3d const& speedDelta,
                          Eigen::Matrix3d const& biasAvgOrientAndScale,
                          double delta_t1,
                          double delta_t2) :
        AccelerometerStepCostBase(speedDelta, delta_t1, delta_t2),
        _biasAvgOrientAndScale(biasAvgOrientAndScale)
    {

    }

    template <typename T>
    bool operator()(const T* const r0,
                    const T* const t0,
                    const T* const r1,
                    const T* const t1,
                    const T* const t2,
                    const T* const g,
                    const T* const scale,
                    const T* const bias,
                    T* residual) const {


        using M3T = Eigen::Matrix<T,3,3>;
        using V3T = Eigen::Matrix<T,3,1>;

        V3T vr0(r0[0], r0[1], r0[2]);
        V3T vr1(r1[0], r1[1], r1[2]);

        V3T vt0(t0[0], t0[1], t0[2]);
        V3T vt1(t1[0], t1[1], t1[2]);
        V3T vt2(t2[0], t2[1], t2[2]);

        V3T vg(g[0], g[1], g[2]);

        V3T vBias(bias[0], bias[1], bias[2]);

        V3T vv0 = (vt1 - vt0)/T(_delta_t1); //avg speed in first interval
        V3T vv1 = (vt2 - vt1)/T(_delta_t2); //avg speed in second interval

        M3T Rr0 = StereoVision::Geometry::rodriguezFormula<T>(vr0);
        M3T Rr1 = StereoVision::Geometry::rodriguezFormula<T>(vr1);
        M3T deltaR = Rr1*Rr0.transpose();
        V3T vrv0 = StereoVision::Geometry::inverseRodriguezFormula<T>(deltaR)*T(0.5);
        M3T Rrv0 = StereoVision::Geometry::rodriguezFormula<T>(vrv0)*Rr0;

        M3T biasTransform;

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                biasTransform(i,j) = T(_biasAvgOrientAndScale(i,j));
            }
        }

        //platform2world orientation at (t0 + t1)/2

        T spdx(_speedDelta[0]);
        T spdy(_speedDelta[1]);
        T spdz(_speedDelta[2]);
        V3T vSpeedDelta(spdx, spdy, spdz);
        V3T gCorr = vg*T(_delta_tv);
        V3T alignedSpeedDelta = (*scale)*Rrv0*(vSpeedDelta - biasTransform*vBias);
        V3T corr = gCorr + alignedSpeedDelta;
        V3T pred = vv0 + corr;
        V3T err = vv1 - pred;

        residual[0] = err[0];
        residual[1] = err[1];
        residual[2] = err[2];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1]) or !ceres::IsFinite(residual[2])) {
            std::cout << "Error in AccelerometerStepCost cost computation" << std::endl;
        }
#endif

        return true;

    }

protected:
    Eigen::Matrix3d _biasAvgOrientAndScale; // int_n^{n+1} int_n^{t} w(t') dt' dt
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMUSTEPCOST_H
