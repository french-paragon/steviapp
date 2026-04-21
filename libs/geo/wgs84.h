#ifndef WGS84_H
#define WGS84_H

#include <array>
#include <math.h>
#include <type_traits>
#include "../utils/types_infos.h"

namespace StereoVisionApp {
namespace Geo {

struct WGS84Ellipsoid {
    static constexpr double a = 6378137.0;
    static constexpr double b = 6356752.314245;
    static constexpr double SemiMajorAxis = a;
    static constexpr double SemiMinorAxis = b;

    static constexpr double EquatorNormalGravity = 9.7803253359;
    static constexpr double PoleNormalGravity = 9.8321849378;

    template <typename T>
    static inline constexpr std::array<T,3> Ecef2LatLonHeight(std::array<T,3> const& ECEF) {

        //lon
        T lon = atan2(ECEF[1],ECEF[0]);

        //lat
        T p = sqrt(ECEF[1]*ECEF[1] + ECEF[0]*ECEF[0]);
        T psi = atan2(a*ECEF[2],p*b);

        constexpr double e2 = (a*a - b*b)/(a*a);
        constexpr double ep2 = (a*a - b*b)/(b*b);

        T spsi = sin(psi);
        T s3psi = spsi*spsi*spsi;

        T cpsi = cos(psi);
        T c3psi = cpsi*cpsi*cpsi;

        T lat = atan2(ECEF[2]+T(ep2)*b*s3psi, p-T(e2)*a*c3psi);

        //height
        T slat = sin(lat);
        T clat = cos(lat);
        T N = a*a/sqrt(a*a*clat*clat + b*b*slat*slat);

        T h = p/clat - N;

        double scale = 180.0/M_PI;

        return std::array<T,3>{lat*T(scale), lon*T(scale), h};
    }

    template <typename C_T>
    static inline constexpr std::array<typename IndexableInfos<C_T>::ScalarT,3> Ecef2LatLonHeight(C_T const& ECEF) {
        using T = typename IndexableInfos<C_T>::ScalarT;
        return Ecef2LatLonHeight(std::array<T,3>{ECEF[0],ECEF[1],ECEF[2]});
    }

    template <typename T>
    static inline constexpr std::array<T,3> LatLonHeight2ECEF(std::array<T,3> const& latLonHeight) {


        double scale = M_PI/180.0;

        T slat = sin(latLonHeight[0]*T(scale));
        T clat = cos(latLonHeight[0]*T(scale));

        T slon = sin(latLonHeight[1]*T(scale));
        T clon = cos(latLonHeight[1]*T(scale));

        T N = a*a/sqrt(a*a*clat*clat + b*b*slat*slat);

        T x = (N+latLonHeight[2])*clat*clon;
        T y = (N+latLonHeight[2])*clat*slon;
        T z = ((b*b/(a*a))*N+latLonHeight[2])*slat;

        return std::array<T,3>{x,y,z};
    }
    template <typename C_T>
    static inline constexpr std::array<typename IndexableInfos<C_T>::ScalarT,3> LatLonHeight2ECEF(C_T const& latLonHeight) {
        using T = typename IndexableInfos<C_T>::ScalarT;
        return LatLonHeight2ECEF(std::array<T,3>{latLonHeight[0],latLonHeight[1],latLonHeight[2]});
    }

    template <typename T>
    static inline constexpr std::array<T,3> gravityEcefModel(std::array<T,3> const& ECEF) {
        std::array<T,3> latLonHeight = Ecef2LatLonHeight(ECEF);
        T lat = latLonHeight[0];
        T h = latLonHeight[2];
        T sLat = sin(lat);
        T sLat2 = sLat*sLat;
        T cLat2 = T(1) - sLat2;

        T gScale = (T(a*EquatorNormalGravity)*cLat2 + T(b*PoleNormalGravity)*sLat2)/sqrt(T(a*a)*cLat2 + T(b*b)*sLat2);

        constexpr double f = (a-b)/a;
        constexpr double G = 6.674e-11;
        constexpr double Me = 5.9722e24;
        constexpr double we = 2*M_PI/(24*60*60); //rotation rate of the earth
        constexpr double m = we*we*a*a*b/(G*Me);

        T hCorr = T(1) - T(2/a)*(T(1 + f + m) - T(2*f)*sLat2)*h + T(3/(a*a))*h*h;
        gScale *= hCorr;

        //shift to compute normal
        latLonHeight[2] += T(10); //10 meters highers
        std::array<T,3> ecefShifted = LatLonHeight2ECEF(latLonHeight);

        std::array<T,3> g;
        T norm = T(0);
        for (int i = 0; i < 3; i++) {
            g[i] = ecefShifted[i] - ECEF[i];
            norm += g[i]*g[i];
        }
        norm = sqrt(norm);
        T scale = gScale/norm;
        for (int i = 0; i < 3; i++) {
            g[i] *= scale;
        }

        return g;
    }
    template <typename C_T>
    static inline constexpr std::array<typename IndexableInfos<C_T>::ScalarT,3> gravityEcefModel(C_T const& ECEF) {
        using T = typename IndexableInfos<C_T>::ScalarT;
        return gravityEcefModel(std::array<T,3>{ECEF[0],ECEF[1],ECEF[2]});
    }

};

} // namespace Geo
} // namespace StereoVisionApp

#endif // WGS84_H
