#ifndef LOCALFRAMES_H
#define LOCALFRAMES_H

#include <optional>

#include <Eigen/Core>
#include <Eigen/QR>

#include <StereoVision/geometry/rotations.h>

#include <proj.h>
#include <iostream>

namespace StereoVisionApp {
namespace Geo {

enum TopocentricConvention {
    NED = 0, //North East Down
    ENU = 1, //East North Up
    NWU = 2, //North West Up
};

/*!
 * \brief getLTPC2ECEF gets a local frame of reference in a certain geographic coordinate system.
 * \return an affine transform, representing the transformation from the Local Tangent Plane Coordinates (LTPC) system to the ECEF frame
 */
template<typename T>
std::optional<StereoVision::Geometry::AffineTransform<T>> getLTPC2ECEF(Eigen::Matrix<T,3,1> const& inputCoord,
                                                                       std::string const& crs,
                                                                       TopocentricConvention topocentricConventions) {

    const char* wgs84_latlon = "EPSG:4979"; //lat lon, height above the ellipsoid
    const char* wgs84_ecef = "EPSG:4978";

    PJ_CONTEXT* ctx = proj_context_create();

    PJ* toEcef = proj_create_crs_to_crs(ctx, crs.c_str(), wgs84_ecef, nullptr);

    if (toEcef == 0) { //in case of error
        proj_context_destroy(ctx);
        return std::nullopt;
    }

    PJ* latLon2ecef = proj_create_crs_to_crs(ctx, wgs84_latlon, wgs84_ecef, nullptr);
    PJ* ecef2latLon = proj_create_crs_to_crs(ctx, wgs84_ecef, wgs84_latlon, nullptr);

    std::optional<StereoVision::Geometry::AffineTransform<T>> ret = std::nullopt;

    PJ_COORD coordBase;
    coordBase.xyz.x = inputCoord[0];
    coordBase.xyz.y = inputCoord[1];
    coordBase.xyz.z = inputCoord[2];

    PJ_COORD ecefPos = proj_trans(toEcef, PJ_FWD, coordBase);
    PJ_COORD geoPos = proj_trans(ecef2latLon, PJ_FWD, ecefPos);

    PJ_COORD posUpN = geoPos;
    posUpN.lpz.phi += 1e-5; //approximately 1m at sea level
    PJ_COORD posUpNEcef = proj_trans(latLon2ecef, PJ_FWD, posUpN);

    PJ_COORD posDownS = geoPos;
    posDownS.lpz.phi -= 1e-5; //approximately 1m at sea level
    PJ_COORD posDownSEcef = proj_trans(latLon2ecef, PJ_FWD, posDownS);

    PJ_COORD posUp = geoPos;
    posUp.lpz.z += 1;
    PJ_COORD posUpEcef = proj_trans(latLon2ecef, PJ_FWD, posUp);

    Eigen::Vector3d upVec(posUpEcef.xyz.x - ecefPos.xyz.x,
                          posUpEcef.xyz.y - ecefPos.xyz.y,
                          posUpEcef.xyz.z - ecefPos.xyz.z);
    upVec.normalize();

    Eigen::Vector3d northVec(posUpNEcef.xyz.x - posDownS.xyz.x,
                             posUpNEcef.xyz.y - posDownS.xyz.y,
                             posUpNEcef.xyz.z - posDownS.xyz.z);
    double res = northVec.dot(upVec);
    northVec -= res*upVec;

    northVec.normalize();

    Eigen::Vector3d eastVec = northVec.cross(upVec);

    StereoVision::Geometry::AffineTransform<T> transform;

    transform.t[0] = ecefPos.xyz.x;
    transform.t[1] = ecefPos.xyz.y;
    transform.t[2] = ecefPos.xyz.z;

    Eigen::Matrix3d R;

    switch (topocentricConventions) {
    case TopocentricConvention::NED:
        R.col(0) = northVec;
        R.col(1) = eastVec;
        R.col(2) = -upVec;
        break;
    case TopocentricConvention::ENU:
        R.col(0) = eastVec;
        R.col(1) = northVec;
        R.col(2) = upVec;
        break;
    case TopocentricConvention::NWU:
        R.col(0) = northVec;
        R.col(1) = -eastVec;
        R.col(2) = upVec;
        break;
    }

    transform.R = R.cast<T>();

    ret = transform;

    cleanup :

    proj_destroy(latLon2ecef);
    proj_destroy(ecef2latLon);
    proj_destroy(toEcef);
    proj_context_destroy(ctx);

    return ret;

}

template<typename T>
std::optional<std::vector<StereoVision::Geometry::AffineTransform<T>>> getLTPC2ECEF(Eigen::Array<T,3,Eigen::Dynamic> const& inputCoords,
                                                                                    std::string const& crs,
                                                                                    TopocentricConvention topocentricConventions) {

    const char* wgs84_latlon = "EPSG:4979"; //lat lon, height above the ellipsoid
    const char* wgs84_ecef = "EPSG:4978";

    PJ_CONTEXT* ctx = proj_context_create();

    PJ* toEcef = proj_create_crs_to_crs(ctx, crs.c_str(), wgs84_ecef, nullptr);

    if (toEcef == 0) { //in case of error
        proj_context_destroy(ctx);
        return std::nullopt;
    }

    PJ* latLon2ecef = proj_create_crs_to_crs(ctx, wgs84_latlon, wgs84_ecef, nullptr);
    PJ* ecef2latLon = proj_create_crs_to_crs(ctx, wgs84_ecef, wgs84_latlon, nullptr);

    if (latLon2ecef == nullptr or ecef2latLon == nullptr) {
        if (latLon2ecef != nullptr) proj_destroy(latLon2ecef);
        if (ecef2latLon != nullptr) proj_destroy(ecef2latLon);
        proj_destroy(toEcef);
        proj_context_destroy(ctx);
        return std::nullopt;
    }

    std::vector<StereoVision::Geometry::AffineTransform<T>> ret(inputCoords.cols());

    for (int i = 0; i < inputCoords.cols(); i++) {

        PJ_COORD coordBase;
        coordBase.xyz.x = inputCoords(0,i);
        coordBase.xyz.y = inputCoords(1,i);
        coordBase.xyz.z = inputCoords(2,i);

        std::array<double, 3> ecefPosArray = {inputCoords(0,i), inputCoords(1,i), inputCoords(2,i)};

        constexpr int stride = sizeof(double);

        //proj can react strangely in release mode if we have the same input and output CRS
        if (std::string(wgs84_ecef) != crs) {
            proj_trans_generic(toEcef, PJ_FWD, &(ecefPosArray[0]), stride, 1, &(ecefPosArray[1]), stride, 1, &(ecefPosArray[2]), stride, 1, nullptr, 0, 0);
        }

        std::array<double, 3> geoPos = ecefPosArray;
        proj_trans_generic(ecef2latLon, PJ_FWD, &(geoPos[0]), stride, 1, &(geoPos[1]), stride, 1, &(geoPos[2]), stride, 1, nullptr, 0, 0);

        std::array<double, 3> posUpN = geoPos;
        posUpN[0] += 1e-5; //approximately 1m at sea level
        proj_trans_generic(latLon2ecef, PJ_FWD, &(posUpN[0]), stride, 1, &(posUpN[1]), stride, 1, &(posUpN[2]), stride,1, nullptr, 0, 0);

        std::array<double, 3> posDownS = geoPos;
        posDownS[0] -= 1e-5; //approximately 1m at sea level
        proj_trans_generic(latLon2ecef, PJ_FWD, &(posDownS[0]), stride, 1, &(posDownS[1]), stride, 1, &(posDownS[2]), stride, 1, nullptr, 0, 0);

        std::array<double, 3> posUp = geoPos;
        posUp[2] += 1;
        proj_trans_generic(latLon2ecef, PJ_FWD, &(posUp[0]), stride, 1, &(posUp[1]), stride, 1, &(posUp[2]), stride, 1, nullptr, 0, 0);

        Eigen::Vector3d upVec(posUp[0] - ecefPosArray[0],
                              posUp[1] - ecefPosArray[1],
                              posUp[2] - ecefPosArray[2]);
        upVec.normalize();

        Eigen::Vector3d northVec(posUpN[0] - posDownS[0],
                                 posUpN[1] - posDownS[1],
                                 posUpN[2] - posDownS[2]);

        double res = northVec.dot(upVec);
        northVec -= res*upVec;

        northVec.normalize();

        Eigen::Vector3d eastVec = northVec.cross(upVec);

        StereoVision::Geometry::AffineTransform<T> transform;

        transform.t[0] = ecefPosArray[0];
        transform.t[1] = ecefPosArray[1];
        transform.t[2] = ecefPosArray[2];

        Eigen::Matrix3d R;

        switch (topocentricConventions) {
        case TopocentricConvention::NED:
            R.col(0) = northVec;
            R.col(1) = eastVec;
            R.col(2) = -upVec;
            break;
        case TopocentricConvention::ENU:
            R.col(0) = eastVec;
            R.col(1) = northVec;
            R.col(2) = upVec;
            break;
        case TopocentricConvention::NWU:
            R.col(0) = northVec;
            R.col(1) = -eastVec;
            R.col(2) = upVec;
            break;
        default:
            //without valid topocentric convention, we have to return nullopt
            return std::nullopt;
        }

        transform.R = R.cast<T>();

        ret[i] = transform;
    }

    proj_destroy(latLon2ecef);
    proj_destroy(ecef2latLon);
    proj_destroy(toEcef);
    proj_context_destroy(ctx);

    return ret;

}

} // namespace Geo
} // namespace StereoVisionApp

#endif // LOCALFRAMES_H
