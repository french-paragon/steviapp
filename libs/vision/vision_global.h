#ifndef VISION_GLOBAL_H
#define VISION_GLOBAL_H

#include <MultidimArrays/MultidimArrays.h>

namespace StereoVisionApp {

typedef Multidim::Array<float, 3, Multidim::NonConstView> ImageArray;
typedef Multidim::Array<float, 2, Multidim::NonConstView> GrayImageArray;

}

#endif // VISION_GLOBAL_H
