#ifndef STEREOVISIONAPP_LENSDISTORTION_H
#define STEREOVISIONAPP_LENSDISTORTION_H

#include "geometry/core.h"

namespace StereoVisionApp {

Eigen::Vector2f radialDistortion(Eigen::Vector2f pos, Eigen::Vector3f k123);
Eigen::Vector2d radialDistortionD(Eigen::Vector2d pos, Eigen::Vector3d k123);

Eigen::Vector2f tangentialDistortion(Eigen::Vector2f pos, Eigen::Vector2f t12);
Eigen::Vector2d tangentialDistortionD(Eigen::Vector2d pos, Eigen::Vector2d t12);

Eigen::Vector2f skewDistortion(Eigen::Vector2f pos, Eigen::Vector2f B12, float f, Eigen::Vector2f pp);
Eigen::Vector2d skewDistortionD(Eigen::Vector2d pos, Eigen::Vector2d B12, double f, Eigen::Vector2d pp);

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LENSDISTORTION_H
