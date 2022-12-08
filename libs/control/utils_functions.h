#ifndef STEREOVISIONAPP_UTILS_FUNCTIONS_H
#define STEREOVISIONAPP_UTILS_FUNCTIONS_H

#include <memory>

namespace StereoVision::Geometry {
	class StereoRigRectifier;
}

namespace StereoVisionApp {

class ImagePair;

std::unique_ptr<StereoVision::Geometry::StereoRigRectifier> configureRectifierForStereoPair(ImagePair* pair, bool useOptimizedParametersSet = true);

}


#endif // STEREOVISIONAPP_UTILS_FUNCTIONS_H
