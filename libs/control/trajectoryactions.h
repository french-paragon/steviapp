#ifndef STEREOVISIONAPP_TRAJECTORYACTIONS_H
#define STEREOVISIONAPP_TRAJECTORYACTIONS_H


namespace StereoVisionApp {

class Trajectory;

void viewTrajectory(Trajectory* traj, bool optimized);

void setLeverArm(Trajectory* traj);
void setAccelerometerMounting(Trajectory* traj);
void setGyroMounting(Trajectory* traj);

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYACTIONS_H
