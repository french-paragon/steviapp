#ifndef STEREOVISIONAPP_TRAJECTORYACTIONS_H
#define STEREOVISIONAPP_TRAJECTORYACTIONS_H

#include <QString>

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

void exportTrajectory(Trajectory* traj, QString filePath = "", bool exportOptimized = false);

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYACTIONS_H
