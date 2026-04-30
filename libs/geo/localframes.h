#ifndef LOCALFRAMES_H
#define LOCALFRAMES_H

#include <optional>

#include <Eigen/Core>
#include <Eigen/QR>

#include <StereoVision/geometry/rotations.h>

#include <proj.h>

#include "../utils/types_infos.h"
#include "./wgs84.h"

#include <iostream>

namespace StereoVisionApp {
namespace Geo {

enum TopocentricConvention {
    NED = 0, //North East Down
    ENU = 1, //East North Up
    NWU = 2, //North West Up
};

template<typename T>
StereoVision::Geometry::AffineTransform<T> getLocalCartesianFrameOnSphere(Eigen::Matrix<T,3,1> const& mean, T radius) {

    using V3T = Eigen::Matrix<T,3,1>;
    using M3T = Eigen::Matrix<T,3,3>;

    V3T direction = mean.normalized();
    V3T origin = direction*radius;

    V3T vertical = V3T(0,0,1);

    if (direction.z() < 0) {
        vertical = -vertical;
    }

    T scaling = vertical.dot(direction);

    if (scaling < 1e-3) {
        V3T toNorth = V3T(0,0,1);
        direction.z() = 0;
        direction.normalize();
        V3T toEast = toNorth.cross(direction);

        M3T Rlocal2ecef;
        Rlocal2ecef.template block<3,1>(0,0) = toEast;
        Rlocal2ecef.template block<3,1>(0,1) = toNorth;
        Rlocal2ecef.template block<3,1>(0,2) = direction;

        M3T Recef2local = Rlocal2ecef.transpose();

        return StereoVision::Geometry::AffineTransform<T>(Recef2local, -Recef2local*origin);
    }

    vertical /= scaling; //scale to form a right triangle perpendicular to the sphere
    V3T toNorth = vertical - direction;

    if (direction.z() < 0) {
        toNorth = -toNorth;
    }

    if (toNorth.norm()*radius < 1) {
        //put a local system at the poles
        return StereoVision::Geometry::AffineTransform<T>(M3T::Identity(), V3T(0,0,-radius));
    }

    toNorth.normalize();
    V3T toEast = toNorth.cross(direction);

    M3T Rlocal2ecef;

    Rlocal2ecef.template block<3,1>(0,0) = toEast;
    Rlocal2ecef.template block<3,1>(0,1) = toNorth;
    Rlocal2ecef.template block<3,1>(0,2) = direction;

    M3T Recef2local = Rlocal2ecef.transpose();

    return StereoVision::Geometry::AffineTransform<T>(Recef2local, -Recef2local*origin);

}

template <typename T>
static inline constexpr Eigen::Matrix<T,3,3> localFrame2ECEFFromGeo(std::array<T,3> const& latLonHeight,
                                                                      TopocentricConvention topocentricConventions) {
    double scale = M_PI/180.0;
    T lat = latLonHeight[0];
    T lon = latLonHeight[1];
    T h = latLonHeight[2];
    T sLat = sin(lat*T(scale));
    T cLat = cos(lat*T(scale));
    T sLon = sin(lon*T(scale));
    T cLon = cos(lon*T(scale));

    using V3T = Eigen::Matrix<T,3,1>;

    V3T North(-sLat*cLon, -sLat*sLon, cLat);
    V3T East(-sLon, cLon, 0);
    V3T Down(-cLat*cLon, -cLat*sLon, -sLat);

    Eigen::Matrix<T,3,3> ret;

    switch (topocentricConventions) {
    case NED:
        ret.col(0) = North;
        ret.col(1) = East;
        ret.col(2) = Down;
        break;
    case ENU:
        ret.col(0) = East;
        ret.col(1) = North;
        ret.col(2) = -Down;
        break;
    case NWU:
        ret.col(0) = North;
        ret.col(1) = -East;
        ret.col(2) = -Down;
        break;
    }

    return ret;
}

template <typename T>
static inline constexpr Eigen::Matrix<T,3,3> localFrame2ECEFFromECEF(std::array<T,3> const& ECEF,
                                                                       TopocentricConvention topocentricConventions) {

    std::array<T,3> latLonHeight = WGS84Ellipsoid::Ecef2LatLonHeight(ECEF);
    return localFrame2ECEFFromGeo(latLonHeight, topocentricConventions);
}

template <typename C_T>
static inline constexpr Eigen::Matrix<typename IndexableInfos<C_T>::ScalarT,3,3> localFrame2ECEFFromGeo(C_T const& latLonHeight,
                                                                                                          TopocentricConvention topocentricConventions) {
    using T = typename IndexableInfos<C_T>::ScalarT;
    return localFrame2ECEFFromGeo(std::array<T,3>{latLonHeight[0],latLonHeight[1],latLonHeight[2]}, topocentricConventions);
}
template <typename C_T>
static inline constexpr Eigen::Matrix<typename IndexableInfos<C_T>::ScalarT,3,3> localFrame2ECEFFromECEF(C_T const& ECEF,
                                                                                                           TopocentricConvention topocentricConventions) {
    using T = typename IndexableInfos<C_T>::ScalarT;
    return localFrame2ECEFFromECEF(std::array<T,3>{ECEF[0],ECEF[1],ECEF[2]}, topocentricConventions);
}


/*!
 * \brief getLTPC2ECEF gets a local frame of reference in a certain geographic coordinate system.
 * \return an affine transform, representing the transformation from the Local Tangent Plane Coordinates (LTPC) system to the ECEF frame
 */
template<typename T>
std::optional<StereoVision::Geometry::AffineTransform<T>> getLTPC2ECEF(Eigen::Matrix<T,3,1> const& inputCoord,
                                                                       std::string const& crs,
                                                                       TopocentricConvention topocentricConventions) {

    const char* wgs84_ecef = "EPSG:4978";

    PJ_CONTEXT* ctx = proj_context_create();

    PJ* toEcef = proj_create_crs_to_crs(ctx, crs.c_str(), wgs84_ecef, nullptr);

    if (toEcef == 0) { //in case of error
        proj_context_destroy(ctx);
        return std::nullopt;
    }

    std::optional<StereoVision::Geometry::AffineTransform<T>> ret = std::nullopt;

    PJ_COORD coordBase;
    coordBase.xyz.x = inputCoord[0];
    coordBase.xyz.y = inputCoord[1];
    coordBase.xyz.z = inputCoord[2];

    PJ_COORD ecefPos = coordBase;
    if (wgs84_ecef != crs) {
        ecefPos = proj_trans(toEcef, PJ_FWD, coordBase);
    }

    StereoVision::Geometry::AffineTransform<T> transform;

    transform.t[0] = ecefPos.xyz.x;
    transform.t[1] = ecefPos.xyz.y;
    transform.t[2] = ecefPos.xyz.z;

    transform.R = localFrame2ECEFFromECEF(transform.t, topocentricConventions);

    ret = transform;

    proj_destroy(toEcef);
    proj_context_destroy(ctx);

    return ret;

}

template<typename T>
std::optional<std::vector<StereoVision::Geometry::AffineTransform<T>>> getLTPC2ECEF(Eigen::Array<T,3,Eigen::Dynamic> const& inputCoords,
                                                                                    std::string const& crs,
                                                                                    TopocentricConvention topocentricConventions) {

    const char* wgs84_ecef = "EPSG:4978";

    PJ_CONTEXT* ctx = proj_context_create();

    PJ* toEcef = proj_create_crs_to_crs(ctx, crs.c_str(), wgs84_ecef, nullptr);

    if (toEcef == nullptr) { //in case of error
        proj_context_destroy(ctx);
        return std::nullopt;
    }

    std::vector<StereoVision::Geometry::AffineTransform<T>> ret(inputCoords.cols());

    for (int i = 0; i < inputCoords.cols(); i++) {

        PJ_COORD coordBase;
        coordBase.xyz.x = inputCoords(0,i);
        coordBase.xyz.y = inputCoords(1,i);
        coordBase.xyz.z = inputCoords(2,i);

        PJ_COORD ecefPos = coordBase;
        if (wgs84_ecef != crs) {
            ecefPos = proj_trans(toEcef, PJ_FWD, coordBase);
        }

        StereoVision::Geometry::AffineTransform<T> transform;

        transform.t[0] = ecefPos.xyz.x;
        transform.t[1] = ecefPos.xyz.y;
        transform.t[2] = ecefPos.xyz.z;

        transform.R = localFrame2ECEFFromECEF(transform.t, topocentricConventions);

        ret[i] = transform;
    }

    proj_destroy(toEcef);
    proj_context_destroy(ctx);

    return ret;

}

} // namespace Geo
} // namespace StereoVisionApp

#endif // LOCALFRAMES_H
