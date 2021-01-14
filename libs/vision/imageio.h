#ifndef STEREOVISIONAPP_IMAGEIO_H
#define STEREOVISIONAPP_IMAGEIO_H

#include "./vision_global.h"

#include <QString>

namespace StereoVisionApp {

ImageArray getImageData(QString const& imFile, float gamma = 2.2);

bool saveImageData(QString const& imFile, ImageArray const& imageData, float gamma = 2.2);

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMAGEIO_H
