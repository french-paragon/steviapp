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

/*!
 * \brief The ImuStepCost class represent a measurement by an IMU (gyro + accelerometer) between two poses
 *
 * In robotics this might be called IMU integration handler -> ask Jan for the paper
 *
 * This cost function assume arbitrary measurements of acceleration and angular speed.
 *
 * The orientation delta, given as an axis angle, is assumed to be int_n^{n+1} w(t) dt where w(t) is the angular speed
 * (it is assumed that the rotation is integrated on the manifold, so that the delta correspond to a rotation between the two poses)
 *
 * The position speed delta is assumed to be int_n^{n+1} int_n^t f(t') dt' dt
 * Such that t_{n+1} = t{n} + v{n} * \Delta t - g * \Delta t^2 + int_n^{n+1} int_n^t f(t') dt' dt
 *
 * The speed delta is assumed to be int_n^{n+1} f(t) dt
 * Such that v_{n+1} = v{n} - g * \Delta t + int_n^{n+1} f(t) dt
 *
 * Pose are assumed to be the Body2World (or Body2Local if a locale frame is used).
 */
class ImuStepCost
{
public:

    /*!
     * \brief getIntegratedIMUDiff buid an IMU diff cost from data sequences
     * \param GyroData the gyroscope speed, as rotation axis speed
     * \param AccData the accellerometer data
     * \param t0 the time for the initial pose
     * \param tf the time for the final pose
     * \return an ImuStepCost cost
     */
    template<typename Tgyro, typename Tacc, typename TimeT>
    static ImuStepCost* getIntegratedIMUDiff(IndexedTimeSequence<Tgyro, TimeT> const& GyroData,
                                             IndexedTimeSequence<Tacc, TimeT> const& AccData,
                                             TimeT t0,
                                             TimeT tf) {

        double delta_t = tf - t0;

        Eigen::Vector3d orientationDelta = Eigen::Vector3d::Zero();
        Eigen::Vector3d posSpeedDelta = Eigen::Vector3d::Zero();
        Eigen::Vector3d speedDelta = Eigen::Vector3d::Zero();

        auto imuAccData = AccData.getValuesInBetweenTimes(t0, tf);
        auto imuGyroData = GyroData.getValuesInBetweenTimes(t0, tf);

        Eigen::Matrix3d Rcurrent2initial = Eigen::Matrix3d::Identity();

        double t_acc = 0;
        double t_gyro = 0;

        int idx_acc = 0;

        for (int j = 0; j < imuGyroData.size(); j++) {

            double dt = imuGyroData[j].dt;

            while (t_acc < t_gyro + dt/2) {
                double dt_acc = imuAccData[idx_acc].dt;

                Eigen::Vector3d da_local(imuGyroData[idx_acc].val[0], imuGyroData[idx_acc].val[1], imuGyroData[idx_acc].val[2]);

                speedDelta += Rcurrent2initial*da_local*dt_acc;
                posSpeedDelta += speedDelta*dt_acc;

                t_acc += dt_acc;
                idx_acc++;
            }

            Eigen::Vector3d dr(imuGyroData[j].val[0], imuGyroData[j].val[1], imuGyroData[j].val[2]);
            dr *= dt;

            Eigen::Matrix3d dR = StereoVision::Geometry::rodriguezFormula(dr);

            Rcurrent2initial = dR*Rcurrent2initial;

            //re-constaint as R mat for numerical stability
            Eigen::Vector3d logR = StereoVision::Geometry::inverseRodriguezFormula(Rcurrent2initial);
            Rcurrent2initial = StereoVision::Geometry::rodriguezFormula(logR);

            t_gyro += dt;

        }

        while (idx_acc < imuAccData.size()) {
            double dt_acc = imuAccData[idx_acc].dt;

            Eigen::Vector3d da_local(imuGyroData[idx_acc].val[0], imuGyroData[idx_acc].val[1], imuGyroData[idx_acc].val[2]);

            speedDelta += Rcurrent2initial*da_local*dt_acc;
            posSpeedDelta += speedDelta*dt_acc;

            t_acc += dt_acc;
            idx_acc++;
        }

        orientationDelta = StereoVision::Geometry::inverseRodriguezFormula(Rcurrent2initial);

        return new StereoVisionApp::ImuStepCost(orientationDelta, posSpeedDelta, speedDelta, delta_t);
    }

    /*!
     * \brief ImuStepCost constructor
     * \param orientationDelta int_n^{n+1} w(t) dt
     * \param posSpeedDelta int_n^{n+1} int_n^t f(t') dt' dt
     * \param speedDelta int_n^{n+1} f(t) dt
     * \param delta_t the time step that was used for integration for obtaining the values above.
     */
    ImuStepCost(Eigen::Vector3d const& orientationDelta,
                Eigen::Vector3d const& posSpeedDelta,
                Eigen::Vector3d const& speedDelta,
                double delta_t);

    template <typename T>
    bool operator()(const T* const r1,
                    const T* const t1,
                    const T* const v1,
                    const T* const r2,
                    const T* const t2,
                    const T* const v2,
                    const T* const g,
                    T* residual) const {


        using M3T = Eigen::Matrix<T,3,3>;
        using V3T = Eigen::Matrix<T,3,1>;

        V3T vr1(r1[0], r1[1], r1[2]);
        M3T R1 = StereoVision::Geometry::rodriguezFormula<T>(vr1);

        V3T vt1(t1[0], t1[1], t1[2]);

        V3T vv1(v1[0], v1[1], v1[2]);

        V3T vr2(r2[0], r2[1], r2[2]);
        M3T R2 = StereoVision::Geometry::rodriguezFormula<T>(vr2);

        V3T vt2(t2[0], t2[1], t2[2]);

        V3T vv2(v2[0], v2[1], v2[2]);

        V3T vg(g[0], g[1], g[2]);

        M3T R1p1 = _orientationDelta.cast<T>()*R1;

        T dt(_delta_t);

        V3T vt1p1 = vt1 + vv1*dt - vg*dt*dt + R1*_posSpeedDelta;
        V3T vv1p1 = vv1 - vg*dt + R1*_speedDelta;

        V3T errR = StereoVision::Geometry::inverseRodriguezFormula<T>(R1p1.transpose()*R2);
        V3T errt = vt1p1 - vt2;
        V3T errv = vv1p1 - vv2;

        residual[0] = errR[0];
        residual[1] = errR[1];
        residual[2] = errR[2];

        residual[3] = errt[0];
        residual[4] = errt[1];
        residual[5] = errt[2];

        residual[6] = errv[0];
        residual[7] = errv[1];
        residual[8] = errv[2];


#ifndef NDEBUG
        for (int i = 0; i < 9; i++) {
            if (!ceres::IsFinite(residual[i])) {
                std::cout << "Error in ImuStepCost cost computation" << std::endl;
            }
        }
#endif

        return true;
    }

protected:

    Eigen::Matrix3d _orientationDelta; // int_n^{n+1} w(t) dt
    Eigen::Vector3d _posSpeedDelta; // int_n^{n+1} int_n^t a(t') dt' dt (in world or local frame)
    Eigen::Vector3d _speedDelta; // int_n^{n+1} a(t) dt (in world or local frame)
    double _delta_t;
};

/*!
 * \brief The GyroStepCost class represent a measurement by an gyroscope between two poses
 */
class GyroStepCost {

public:

    template<typename Tgyro, typename TimeT>
    static GyroStepCost* getIntegratedIMUDiff(IndexedTimeSequence<Tgyro, TimeT> const& GyroData,
                                              TimeT t0,
                                              TimeT t1) {

        double delta_t = t1 - t0;

        auto imuGyroData = GyroData.getValuesInBetweenTimes(t0, t1);

        Eigen::Matrix3d Rcurrent2initial = Eigen::Matrix3d::Identity();


        for (int j = 0; j < imuGyroData.size(); j++) {

            double dt = imuGyroData[j].dt;

            Eigen::Vector3d dr(imuGyroData[j].val[0], imuGyroData[j].val[1], imuGyroData[j].val[2]);
            dr *= dt;

            Eigen::Matrix3d dR = StereoVision::Geometry::rodriguezFormula(dr);

            Eigen::Matrix3d newRcurrent2initial = dR*Rcurrent2initial;

            //re-constaint as R mat for numerical stability
            Eigen::Vector3d logR = StereoVision::Geometry::inverseRodriguezFormula(newRcurrent2initial);
            Rcurrent2initial = StereoVision::Geometry::rodriguezFormula(logR);

        }

        return new StereoVisionApp::GyroStepCost(Rcurrent2initial, delta_t);


    }

    GyroStepCost(Eigen::Matrix3d const& attitudeDelta,
                 double delta_t);

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

protected:

    Eigen::Matrix3d _attitudeDelta; // R from final pose to initial pose
    double _delta_t;
};

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
