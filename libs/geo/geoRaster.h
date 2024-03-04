#ifndef GEORASTER_H
#define GEORASTER_H

#include <optional>

#include <MultidimArrays/MultidimArrays.h>
#include <Eigen/Core>

namespace StereoVisionApp {
namespace Geo {

template <typename T, int nDim>
struct GeoRasterData {

    static_assert (nDim == 2 or nDim == 3, "a georaster can only have 2 or three dimensions");

    Multidim::Array<T, nDim> raster;
    Eigen::Matrix<double, 2,3> geoTransform;
    std::string crsInfos;

};

} // namespace Geo
} // namespace StereoVisionApp

#endif // GEORASTER_H
