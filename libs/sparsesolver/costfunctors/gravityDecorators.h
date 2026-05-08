#ifndef GRAVITYDECORATORS_H
#define GRAVITYDECORATORS_H

#include <ceres/jet.h>
#include <Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>
#include <StereoVision/geometry/alignement.h>

#include <type_traits>

#include "../../utils/functional.h"
#include "../../geo/wgs84.h"

namespace StereoVisionApp {



template<typename FunctorT, int localPosParamPos, int gravityParamPos>
class GravityReoriented : private FunctorT {

    static_assert(localPosParamPos != gravityParamPos, "Local position and gravity parameters cannot be at the same index");
    static_assert(localPosParamPos >= 0, "local position parameter index cannot be negative");
    static_assert(gravityParamPos >= 0, "gravity parameter index cannot be negative");

public:

    /*!
 * \brief reorientGravity reorient the gravity towards the center of the earth in a local frame
 * \param referenceGravity the reference gravity at position 0,0,0
 * \param localEarthCenter the position of the earth center in the local frame
 * \param pos the position at which gravity should be estimated, in local frame
 * \return an estimation of the gravity
 */
    template<typename T>
    static Eigen::Matrix<T,3,1> reorientGravity(Eigen::Matrix<T,3,1> const& referenceGravity,
                                                  Eigen::Matrix<double,3,1> const& localEarthCenter,
                                                  Eigen::Matrix<T,3,1> const& pos) {


        using V3T = Eigen::Matrix<T,3,1>;

        V3T deltaPos = localEarthCenter.cast<T>() - pos;
        deltaPos.normalize();

        V3T cross = localEarthCenter.normalized().cast<T>().cross(deltaPos);
        T alpha = ceres::asin(cross.norm());

        if (alpha > T(1e-3)) {
            cross.normalize();
            cross *= alpha;
        }

        return StereoVision::Geometry::angleAxisRotate(cross, referenceGravity);
    }

    template <typename ... P>
    GravityReoriented(StereoVision::Geometry::AffineTransform<double> const& local2ecef, P... args) : //take the full local2ecef transform to be compatible with other decorator which might need it.
        FunctorT(args...),
        _localEarthCenter(-local2ecef.R.transpose()*local2ecef.t)
    {
    }

    template <typename ... P>
    GravityReoriented(Eigen::Vector3d const& localEarthCenter, P... args) : //take the full local2ecef transform to be compatible with other decorator which might need it.
        FunctorT(args...),
        _localEarthCenter(localEarthCenter)
    {
    }

    template <typename ... P>
    bool operator()(P ... params) const {

        if (!_localEarthCenter.allFinite()) {
            //if earth center is at infinity
            return FunctorT::operator()(params...);
        }

        std::tuple<P...> args(params...);

        using T = std::remove_const_t<
            std::remove_pointer_t<
                std::remove_reference_t<decltype (std::get<0>(args))>
                >>;

        constexpr int maxParamPos = std::max(localPosParamPos, gravityParamPos);

        static_assert (sizeof... (params) > maxParamPos, "Not enough arguments provided");

        using V3T = Eigen::Vector<T,3>;

        const T* const pos = std::get<localPosParamPos>(args);
        const T* const g = std::get<gravityParamPos>(args);

        T processed_g[3];

        //remove the old arguments and insert new arguments such that processed_g is
        // at pos gravityParamPos.
        auto processedArgs =
            CallFromTuple::insertArgInTuple<gravityParamPos, const T*>(
                    CallFromTuple::removeArgFromTuple<gravityParamPos>(args), processed_g
                );

        V3T posVec;
        posVec << pos[0], pos[1], pos[2];

        V3T gVec;
        gVec << g[0], g[1], g[2];

        V3T alignGVec = reorientGravity(gVec, _localEarthCenter, posVec);

        processed_g[0] = alignGVec[0];
        processed_g[1] = alignGVec[1];
        processed_g[2] = alignGVec[2];

        auto variadic_lambda = [this] (auto... params) {
            (void) this;
            return FunctorT::operator()(params...);
        };

        return CallFromTuple::call(variadic_lambda, processedArgs);
    }

protected:

    Eigen::Vector3d _localEarthCenter;
    Eigen::Vector3d _localEarthCenterDir;

};

template<typename FunctorT, int localPosParamPos, int gravityParamPos>
class Wgs84GravityCorrection : private FunctorT {

public:

    template<typename T>
    static Eigen::Matrix<T,3,1> reorientGravity(Eigen::Matrix<T,3,1> const& originGravity,
                                                  Eigen::Matrix<T,3,1> const& referenceGravity,
                                                  StereoVision::Geometry::AffineTransform<double> const& local2ecef,
                                                  Eigen::Matrix<T,3,1> const& pos) {


        using V3T = Eigen::Matrix<T,3,1>;

        V3T anomaly = originGravity - referenceGravity;
        StereoVision::Geometry::AffineTransform<T> transform = local2ecef.cast<T>();

        V3T posEcef = transform*pos;
        std::array<T,3> localG = Geo::WGS84Ellipsoid::gravityEcefModel(posEcef);
        V3T gVec;
        gVec << localG[0],localG[1],localG[2];

        gVec = -transform.R.transpose()*gVec;

        return gVec + anomaly;
    }


    template <typename ... P>
    Wgs84GravityCorrection(StereoVision::Geometry::AffineTransform<double> const& local2ecef, P... args) :
        FunctorT(args...),
        _local2ecef(local2ecef)
    {
        std::array<double,3> originEcefPos{local2ecef.t.x(), local2ecef.t.y(), local2ecef.t.z()};
        std::array<double,3> origin_reference_gravity = Geo::WGS84Ellipsoid::gravityEcefModel(originEcefPos);
        _origin_reference_gravity = -_local2ecef.R.transpose()*Eigen::Vector3d(origin_reference_gravity[0],origin_reference_gravity[1],origin_reference_gravity[2]);
    }

    template <typename ... P>
    bool operator()(P ... params) const {

        if (!_local2ecef.R.allFinite() or !_local2ecef.t.allFinite()) {
            //if earth center is at infinity
            return FunctorT::operator()(params...);
        }

        std::tuple<P...> args(params...);

        using T = std::remove_const_t<
            std::remove_pointer_t<
                std::remove_reference_t<decltype (std::get<0>(args))>
                >>;

        constexpr int maxParamPos = std::max(localPosParamPos, gravityParamPos);

        static_assert (sizeof... (params) > maxParamPos, "Not enough arguments provided");

        using V3T = Eigen::Vector<T,3>;

        const T* const pos = std::get<localPosParamPos>(args);
        const T* const g = std::get<gravityParamPos>(args);

        T processed_g[3];

        //remove the old arguments and insert new arguments such that processed_g is
        // at pos gravityParamPos.
        auto processedArgs =
            CallFromTuple::insertArgInTuple<gravityParamPos, const T*>(
                CallFromTuple::removeArgFromTuple<gravityParamPos>(args), processed_g
                );

        V3T posVec;
        posVec << pos[0], pos[1], pos[2];

        V3T gVec;
        gVec << g[0], g[1], g[2];

        V3T alignGVec = reorientGravity<T>(gVec, _origin_reference_gravity.cast<T>(), _local2ecef, posVec);

        processed_g[0] = alignGVec[0];
        processed_g[1] = alignGVec[1];
        processed_g[2] = alignGVec[2];

        auto variadic_lambda = [this] (auto... params) {
            (void) this;
            return FunctorT::operator()(params...);
        };

        return CallFromTuple::call(variadic_lambda, processedArgs);
    }

protected:

    Eigen::Vector3d _origin_reference_gravity;
    StereoVision::Geometry::AffineTransform<double> _local2ecef;

};

template<typename FunctorT, int localPosParamPos0, int localPosParamPos1, int gravityParamPos>
class CoriolisAccCorrection : private FunctorT {
public:

    template<typename T>
    static Eigen::Matrix<T,3,1> coriolisEffect(Eigen::Matrix3d const& Recef2local,
                                               Eigen::Matrix<T,3,1> const& speed) {


        using V3T = Eigen::Matrix<T,3,1>;

        constexpr double we = 2*M_PI/(24*60*60); //rotation rate of the earth;
        V3T rotationRate;
        rotationRate << T(0), T(0), T(we);

        rotationRate = Recef2local.cast<T>()*rotationRate;

        return -T(2)*rotationRate.cross(speed);
    }

    template <typename ... P>
    CoriolisAccCorrection(Eigen::Matrix3d const& Recef2local, double dt, P... args) :
        FunctorT(args...),
        _dt(dt),
        _Recef2local(Recef2local)
    {

    }

    template <typename ... P>
    CoriolisAccCorrection(StereoVision::Geometry::AffineTransform<double> const& local2ecef, double dt, P... args) :
        FunctorT(args...),
        _dt(dt),
        _Recef2local(local2ecef.R.transpose())
    {

    }

    template <typename ... P>
    bool operator()(P ... params) const {

        if (!_Recef2local.allFinite()) {
            //if earth center is at infinity
            return FunctorT::operator()(params...);
        }

        std::tuple<P...> args(params...);

        using T = std::remove_const_t<
            std::remove_pointer_t<
                std::remove_reference_t<decltype (std::get<0>(args))>
                >>;

        constexpr int maxParamPos = std::max(std::max(localPosParamPos0,localPosParamPos1), gravityParamPos);

        static_assert (sizeof... (params) > maxParamPos, "Not enough arguments provided");

        using V3T = Eigen::Vector<T,3>;

        const T* const pos0 = std::get<localPosParamPos0>(args);
        const T* const pos1 = std::get<localPosParamPos1>(args);
        const T* const g = std::get<gravityParamPos>(args);

        T processed_g[3];

        //remove the old arguments and insert new arguments such that processed_g is
        // at pos gravityParamPos.
        auto processedArgs =
            CallFromTuple::insertArgInTuple<gravityParamPos, const T*>(
                CallFromTuple::removeArgFromTuple<gravityParamPos>(args), processed_g
                );

        V3T posVec0;
        posVec0 << pos0[0], pos0[1], pos0[2];
        V3T posVec1;
        posVec1 << pos1[0], pos1[1], pos1[2];

        V3T speedVec = (posVec1 - posVec0)/T(_dt);

        V3T gVec;
        gVec << g[0], g[1], g[2];

        V3T delta = coriolisEffect<T>(_Recef2local, speedVec);
        V3T alignGVec = gVec + delta;

        processed_g[0] = alignGVec[0];
        processed_g[1] = alignGVec[1];
        processed_g[2] = alignGVec[2];

        auto variadic_lambda = [this] (auto... params) {
            (void) this;
            return FunctorT::operator()(params...);
        };

        return CallFromTuple::call(variadic_lambda, processedArgs);
    }

protected:

    double _dt;
    Eigen::Matrix3d _Recef2local;
};

} // namespace StereoVisionApp

#endif // GRAVITYDECORATORS_H
