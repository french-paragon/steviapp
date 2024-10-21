#ifndef TRAJECTORYIMUPREINTEGRATION_H
#define TRAJECTORYIMUPREINTEGRATION_H

#include <Eigen/Core>

#include <StereoVision/geometry/rotations.h>

#include "./indexed_timed_sequence.h"

namespace StereoVisionApp {

/*!
 * \brief PreIntegrateGyro pre integrate the gyro
 * \param gyro the gyro sequence
 * \param time1 the time to start the integration at
 * \param time2 the time to end the integration at
 * \return the matrix representing the rotation from time1 to time2 (Pose2 to Pose1, in term of coordinate systems transformation)
 */
template<typename posT, typename timeT>
inline Eigen::Matrix<posT,3,3> PreIntegrateGyro(
        IndexedTimeSequence<Eigen::Matrix<posT,3,1>, timeT> const& gyro,
        timeT time1,
        timeT time2) {

    auto angularIntegrationVariables = gyro.getValuesInBetweenTimes(time1, time2);

    Eigen::Matrix<posT,3,3> GyroR2to1 = Eigen::Matrix<posT,3,3>::Identity();

    for (auto& differential : angularIntegrationVariables) {
        Eigen::Vector3d dr = differential.val*differential.dt;
        Eigen::Matrix3d dR = StereoVision::Geometry::rodriguezFormula(dr);
        GyroR2to1 = dR*GyroR2to1;
    }

    return GyroR2to1;

}

/*!
 * \brief PreIntegrateAccelerometer pre integrate the accelerometer
 * \param accelerometer the accelerometer sequence
 * \param gyro the gyro sequence
 * \param time1 the time to start the integration at
 * \param time2 the time to end the integration at
 * \return the difference in speed, when speed is expressed in the reference frame of the platform at time 1
 */
template<typename posT, typename timeT>
inline Eigen::Matrix<posT,3,1> PreIntegrateAccelerometer(
        IndexedTimeSequence<Eigen::Matrix<posT,3,1>, timeT> const& accelerometer,
        IndexedTimeSequence<Eigen::Matrix<posT,3,1>, timeT> const& gyro,
        timeT time1,
        timeT time2) {

    auto angularIntegrationVariables = gyro.getValuesInBetweenTimes(time1, time2);
    auto accIntegrationVariables = accelerometer.getValuesInBetweenTimes(time1, time2);

    Eigen::Matrix3d Rcurrent2initial = Eigen::Matrix3d::Identity();

    Eigen::Vector3d speedDeltaObs = Eigen::Vector3d::Zero();

    double t_acc = 0;
    double t_gyro = 0;

    int idx_acc = 0;

    for (int j = 0; j < angularIntegrationVariables.size(); j++) {

        double dt = angularIntegrationVariables[j].dt;

        while (t_acc < t_gyro + dt/2) {
            double dt_acc = accIntegrationVariables[idx_acc].dt;

            Eigen::Vector3d acceleration_local(
                        accIntegrationVariables[idx_acc].val[0],
                    accIntegrationVariables[idx_acc].val[1],
                    accIntegrationVariables[idx_acc].val[2]);

            speedDeltaObs += Rcurrent2initial*acceleration_local*dt_acc;

            t_acc += dt_acc;
            idx_acc++;
        }

        Eigen::Vector3d dr(angularIntegrationVariables[j].val[0],
                angularIntegrationVariables[j].val[1],
                angularIntegrationVariables[j].val[2]);
        dr *= dt;

        Eigen::Matrix3d dR = StereoVision::Geometry::rodriguezFormula(dr);

        Rcurrent2initial = dR*Rcurrent2initial;

        //re-constaint as R mat for numerical stability
        Eigen::Vector3d logR = StereoVision::Geometry::inverseRodriguezFormula(Rcurrent2initial);
        Rcurrent2initial = StereoVision::Geometry::rodriguezFormula(logR);

        t_gyro += dt;

    }

    return speedDeltaObs;

}


} // namespace StereoVisionApp

#endif // TRAJECTORYIMUPREINTEGRATION_H
