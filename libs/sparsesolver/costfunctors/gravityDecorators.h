#ifndef GRAVITYDECORATORS_H
#define GRAVITYDECORATORS_H

#include <ceres/jet.h>
#include <Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>
#include <StereoVision/geometry/alignement.h>

#include <type_traits>

#include "../../utils/functional.h"

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
    GravityReoriented(Eigen::Vector3d const& localEarthCenter, P... args) :
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

}

#endif // GRAVITYDECORATORS_H
