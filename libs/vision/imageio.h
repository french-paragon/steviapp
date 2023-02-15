#ifndef STEREOVISIONAPP_IMAGEIO_H
#define STEREOVISIONAPP_IMAGEIO_H

#include "./vision_global.h"

#include <QString>
#include <optional>

namespace StereoVisionApp {

ImageArray getImageData(QString const& imFile, float gamma = 2.2, int* originalFormat = nullptr);

bool saveImageData(QString const& imFile, ImageArray const& imageData, float gamma = 2.2, std::optional<int> imgFormat = std::nullopt);
bool saveImageData(QString const& imFile, GrayImageArray const& imageData, float gamma = 2.2, std::optional<int> imgFormat = std::nullopt);

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMAGEIO_H
