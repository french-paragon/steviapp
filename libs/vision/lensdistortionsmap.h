#ifndef STEREOVISIONAPP_LENSDISTORTIONSMAP_H
#define STEREOVISIONAPP_LENSDISTORTIONSMAP_H

#include "./vision_global.h"

#include <eigen3/Eigen/Core>

namespace StereoVisionApp {

ImageArray computeLensDistortionMap(int height,
									int width,
									float f,
									Eigen::Vector2f pp,
									Eigen::Vector3f k123,
									Eigen::Vector2f t12,
									Eigen::Vector2f B12);

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LENSDISTORTIONSMAP_H
