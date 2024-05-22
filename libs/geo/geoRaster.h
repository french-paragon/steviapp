#ifndef GEORASTER_H
#define GEORASTER_H

#include <optional>

#include <MultidimArrays/MultidimArrays.h>
#include <Eigen/Core>

namespace StereoVisionApp {
namespace Geo {

/*!
 * \brief The WorldInfos struct contain the info of a geotransform following the esri convention for .wld files.
 */
struct WorldInfos {
    double leftPixXSize;
    double yShift;
    double xShift;
    double downPixYSize;
    double posXUpperLeft;
    double posYUpperLeft;
};

template <typename T, int nDim>
struct GeoRasterData {

    static_assert (nDim == 2 or nDim == 3, "a georaster can only have 2 or three dimensions");

    Multidim::Array<T, nDim> raster;
    Eigen::Matrix<double, 2,3> geoTransform;
    std::string crsInfos;

    WorldInfos getWorldInfos() const {
        return WorldInfos {geoTransform(0,0), geoTransform(0,1), geoTransform(1,0), geoTransform(1,1), geoTransform(0,2), geoTransform(1,2)};
    }

};

} // namespace Geo
} // namespace StereoVisionApp

#endif // GEORASTER_H
