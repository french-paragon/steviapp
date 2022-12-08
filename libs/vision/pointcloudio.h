#ifndef STEREOVISIONAPP_POINTCLOUDIO_H
#define STEREOVISIONAPP_POINTCLOUDIO_H

#include "./vision_global.h"

#include <MultidimArrays/MultidimArrays.h>

#include <QString>

namespace StereoVisionApp {

int saveXYZRGBpointcloud(QString file, Multidim::Array<float, 3> const& geom, Multidim::Array<float, 3> const& color, float colorScale = 255, bool correctGamma = true);

} // namespace StereoVisionApp

#endif // POINTCLOUDIO_H
