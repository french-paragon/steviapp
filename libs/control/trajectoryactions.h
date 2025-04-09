#ifndef STEREOVISIONAPP_TRAJECTORYACTIONS_H
#define STEREOVISIONAPP_TRAJECTORYACTIONS_H

#include <QString>

#include <StereoVision/geometry/rotations.h>

namespace StereoVisionApp {

class Trajectory;

void viewTrajectory(Trajectory* traj, bool optimized);

void setLeverArm(Trajectory* traj);
void setAccelerometerMounting(Trajectory* traj);
void setGyroMounting(Trajectory* traj);

/*!
 * \brief checkTrajectoryConsistency used to evaluate if the mounting has been set correctly
 * \param traj the trajectory to analyse.
 */
void checkTrajectoryConsistency(Trajectory* traj);

enum class TrajectoryExportOrientationConvention {
    AxisAngle,
    EulerXYZ,
    EulerZYX
};

/*!
 * \brief exportTrajectory export the trajectory, with possible a sensor2body transform, selecting the options with a gui
 * \param traj the trajectory
 * \param sensor2body the pose of the sensor which trajectory needs to be exported, with respect to body.
 */
void exportTrajectory(Trajectory* traj,
                      StereoVision::Geometry::RigidBodyTransform<double> const& sensor2body = StereoVision::Geometry::RigidBodyTransform<double>());

/*!
 * \brief exportTrajectory export the trajectory in ecef, with possible a sensor2body transform
 * \param traj the trajectory
 * \param filePath the file to save to
 * \param exportOptimized if the optimized trajectory needs to be used
 * \param sensor2body the pose of the sensor which trajectory needs to be exported, with respect to body.
 */
void exportTrajectory(Trajectory* traj,
                      QString filePath,
                      bool exportOptimized = false,
                      TrajectoryExportOrientationConvention const& orientationConvention = TrajectoryExportOrientationConvention::AxisAngle,
                      StereoVision::Geometry::RigidBodyTransform<double> const& sensor2body = StereoVision::Geometry::RigidBodyTransform<double>());
/*!
 * \brief exportTrajectoryGeographic export the trajectory in SBET compatible representation, with possible a sensor2body transform
 * \param traj the trajectory
 * \param filePath the file to save to
 * \param exportOptimized if the optimized trajectory needs to be used
 * \param sensor2body the pose of the sensor which trajectory needs to be exported, with respect to body.
 */
void exportTrajectoryGeographic(Trajectory* traj,
                                QString filePath = "",
                                bool exportOptimized = false,
                                TrajectoryExportOrientationConvention const& orientationConvention = TrajectoryExportOrientationConvention::EulerZYX,
                                StereoVision::Geometry::RigidBodyTransform<double> const& sensor2body = StereoVision::Geometry::RigidBodyTransform<double>());

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYACTIONS_H
