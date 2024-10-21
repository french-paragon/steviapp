#ifndef STEREOVISIONAPP_TRAJECTORYACTIONS_H
#define STEREOVISIONAPP_TRAJECTORYACTIONS_H


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

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYACTIONS_H
