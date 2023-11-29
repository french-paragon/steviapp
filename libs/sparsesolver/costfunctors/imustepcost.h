#ifndef STEREOVISIONAPP_IMUSTEPCOST_H
#define STEREOVISIONAPP_IMUSTEPCOST_H

#include <Eigen/Core>

#include <LibStevi/geometry/core.h>
#include <LibStevi/geometry/rotations.h>

#include "../../vision/indexed_timed_sequence.h"

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

        return true;
    }

protected:

    Eigen::Matrix3d _orientationDelta; // int_n^{n+1} w(t) dt
    Eigen::Vector3d _posSpeedDelta; // int_n^{n+1} int_n^t a(t') dt' dt (in world or local frame)
    Eigen::Vector3d _speedDelta; // int_n^{n+1} a(t) dt (in world or local frame)
    double _delta_t;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMUSTEPCOST_H
