#ifndef STEREOVISIONAPP_UTILS_FUNCTIONS_H
#define STEREOVISIONAPP_UTILS_FUNCTIONS_H

#include <memory>

namespace StereoVision::Geometry {

	template<typename T>
	class ImageRectifier;

	class StereoRigRectifier;
}

namespace StereoVisionApp {

class ImagePair;
class Camera;

std::unique_ptr<StereoVision::Geometry::ImageRectifier<float>> configureRectifierForSingleCamera(Camera* cam, bool useOptimizedParametersSet = true);
std::unique_ptr<StereoVision::Geometry::StereoRigRectifier> configureRectifierForStereoPair(ImagePair* pair, bool useOptimizedParametersSet = true);

}


#endif // STEREOVISIONAPP_UTILS_FUNCTIONS_H
