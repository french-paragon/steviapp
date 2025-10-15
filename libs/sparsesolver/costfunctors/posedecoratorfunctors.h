#ifndef POSEDECORATORFUNCTORS_H
#define POSEDECORATORFUNCTORS_H

#include <ceres/jet.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/dynamic_autodiff_cost_function.h>
#include <Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>
#include <StereoVision/geometry/alignement.h>

#include <type_traits>

#include <vector>

#include "../../utils/functional.h"

namespace ceres {
    class CostFunction;
}

namespace StereoVisionApp {

/*!
 * \brief The PoseConfiguration enum encode flags to configure the poses that will be processed by the decorators
 */
enum DecoratorPoseConfiguration {
    Sensor2Body = 1, //the boresight and lever arm represent a Sensor2Body transformation
    Body2Sensor = 0, //the boresight and lever arm represent a Body2Sensor transformation
    Body2World = 2, //the input pose represent a Body2World transformation
    World2Body = 0, //the input pose represent a World2Body transformation
    BoresightInfoBit = 1, //the bit containing the Boresight Info
    PoseInfoBit = 2, //the bit containing the Pose Info
};

/*!
 * \brief The PoseTransformDirection enum represent how a transform of a pose should be applied
 *
 * Assuming a pose is given from an intial space Initial to a final space Final, along
 * with a transform, this enum indicate if the transform should be applied before (from some
 * source space to the initial space), or after (from the final space to some target space).
 */
enum PoseTransformDirection {
    SourceToInitial = 0, // the transformation should be applied before the pose (map from prior space to initial space)
    FinalToTarget = 1 // the transformation should be applied after the pose (map from final space to target space).
};

/*!
 * \brief The AddParam class is a decorator which add a unused param to a functor which can be used by other decorators further down the line.
 */
template<typename FunctorT, int poseParamPos = 0>
class AddParam : private FunctorT {

public:

    template <typename ... P>
    AddParam(P... args) :
        FunctorT(args...)
    {

    }

    template <typename ... P>
    bool operator()(P ... params) const {

        std::tuple<P...> args(params...);

        using T = std::remove_const_t<
            std::remove_pointer_t<
                std::remove_reference_t<decltype (std::get<0>(args))>
                >>;

        static_assert (sizeof... (params) > poseParamPos+1, "Wrong number of arguments provided");

        auto processedArgs = CallFromTuple::removeArgFromTuple<poseParamPos>(args);

        auto variadic_lambda = [this] (auto... params) {
            return FunctorT::operator()(params...);
        };

        return CallFromTuple::call(variadic_lambda, processedArgs);

    }
};

/*!
 * \brief The AddOrientation class represent a decorator for the functor type FunctorT, adding an orientation parameter on top of a position
 *
 * The FunctorT type is assumed to have an operator() const taking as argument
 * "const T* const t" the position parameters of a pose,
 * and then additional arguments.
 * AddOrientation<FunctorT> then has an operator() const taking as argument
 * "const T* const r, const T* const t"
 * at the same position the position parameter is called in the original functor.
 *
 * This functor does nothing with the added orientation, it is meant to add the orientation
 * parameter so that the functor can be further processed by other decorators.
 */
template<typename FunctorT, int poseParamGroupPos = 0>
using AddOrientation = AddParam<FunctorT, poseParamGroupPos>;

/*!
 * \brief The AddPose class represent a decorator for the functor type FunctorT, adding a pose parameter to the parameters
 *
 * The FunctorT type is assumed to have an operator() const.
 * AddPose<FunctorT> then has an operator() const taking as additional argument
 * "const T* const r, const T* const t" at position poseParamGroupPos.
 *
 * This functor does nothing with the added pose, it is meant to add the pose
 * parameter so that the functor can be further processed by other decorators.
 */
template<typename FunctorT, int poseParamGroupPos = 0>
class AddPose : private FunctorT {

public:

    template <typename ... P>
    AddPose(P... args) :
        FunctorT(args...)
    {

    }

    template <typename ... P>
    bool operator()(P ... params) const {

        std::tuple<P...> args(params...);

        using T = std::remove_const_t<
            std::remove_pointer_t<
                std::remove_reference_t<decltype (std::get<0>(args))>
                >>;

        static_assert (sizeof... (params) > poseParamGroupPos+2, "Wrong number of arguments provided");

        auto processedArgs = CallFromTuple::removeArgFromTuple<poseParamGroupPos>(
            CallFromTuple::removeArgFromTuple<poseParamGroupPos>(args)
            );

        auto variadic_lambda = [this] (auto... params) {
            return FunctorT::operator()(params...);
        };

        return CallFromTuple::call(variadic_lambda, processedArgs);

    }
};

/*!
 * \brief The ApplyLeverArm class is a functor decorator which apply a lever arm to a pose.
 *
 * Both the lever arm and the pose have to be already present as parameters in the functor.
 * Unlike the basic lever arm decorator, this decorator can be chained to apply the lever arm to multiple
 * pose parameters in the functor
 */
template<typename FunctorT, int leverArmParamGroupPos, int poseParamGroupPos, int poseConfig = Sensor2Body | Body2World>
class ApplyLeverArm : private FunctorT {
public:
    template <typename ... P>
    ApplyLeverArm(P... args) :
        FunctorT(args...)
    {

    }

    template <typename ... P>
    bool operator()(P ... params) const {

        std::tuple<P...> args(params...);

        using T = std::remove_const_t<
            std::remove_pointer_t<
                std::remove_reference_t<decltype (std::get<0>(args))>
                >>;

        static_assert (leverArmParamGroupPos - poseParamGroupPos >= 2 or poseParamGroupPos - leverArmParamGroupPos >= 2, "leverArmParamGroup and poseParamGroup overlap");
        static_assert (sizeof... (params) > std::max(leverArmParamGroupPos, poseParamGroupPos)+1, "Wrong number of arguments provided");
        static_assert (((poseConfig & PoseInfoBit) == Body2World) or ((poseConfig & PoseInfoBit) == World2Body), "misconfigured DecoratorPoseConfiguration");
        static_assert (((poseConfig & BoresightInfoBit) == Sensor2Body) or ((poseConfig & BoresightInfoBit) == Body2Sensor), "misconfigured DecoratorPoseConfiguration");

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        const T* const r = std::get<poseParamGroupPos>(args);
        const T* const t = std::get<poseParamGroupPos+1>(args);

        const T* const boresight = std::get<leverArmParamGroupPos>(args);
        const T* const leverArm = std::get<leverArmParamGroupPos+1>(args);

        T processed_r[3];
        T processed_t[3];

        //remove the old arguments and insert new arguments such that processed_r is
        // at pos poseParamGroupPos and processed_t at pos poseParamGroupPos+1.
        auto processedArgs =
            CallFromTuple::insertArgInTuple<poseParamGroupPos, const T*>(
                CallFromTuple::insertArgInTuple<poseParamGroupPos, const T*>(
                    CallFromTuple::removeArgFromTuple<poseParamGroupPos>(
                        CallFromTuple::removeArgFromTuple<poseParamGroupPos>(args)
                    ), processed_t
                ), processed_r
            );

        if ((poseConfig & PoseInfoBit) == Body2World) {

            StereoVision::Geometry::RigidBodyTransform<T> body2world;

            body2world.r << r[0], r[1], r[2];
            body2world.t << t[0], t[1], t[2];

            StereoVision::Geometry::RigidBodyTransform<T> sensor2world;

            if ((poseConfig & BoresightInfoBit) == Sensor2Body) {

                StereoVision::Geometry::RigidBodyTransform<T> sensor2body;
                sensor2body.r << boresight[0], boresight[1], boresight[2];
                sensor2body.t << leverArm[0], leverArm[1], leverArm[2];

                sensor2world = body2world*sensor2body;

            } else if ((poseConfig & BoresightInfoBit) == Body2Sensor) {

                StereoVision::Geometry::RigidBodyTransform<T> body2sensor;
                body2sensor.r << boresight[0], boresight[1], boresight[2];
                body2sensor.t << leverArm[0], leverArm[1], leverArm[2];

                sensor2world = body2world*body2sensor.inverse();
            }

            for (int i = 0; i < 3; i++) {
                processed_r[i] = sensor2world.r[i];
                processed_t[i] = sensor2world.t[i];
            }

        } else if ((poseConfig & PoseInfoBit) == World2Body) {

            StereoVision::Geometry::RigidBodyTransform<T> world2body;

            world2body.r << r[0], r[1], r[2];
            world2body.t << t[0], t[1], t[2];

            StereoVision::Geometry::RigidBodyTransform<T> world2sensor;

            if ((poseConfig & BoresightInfoBit) == Sensor2Body) {

                StereoVision::Geometry::RigidBodyTransform<T> sensor2body;
                sensor2body.r << boresight[0], boresight[1], boresight[2];
                sensor2body.t << leverArm[0], leverArm[1], leverArm[2];

                world2sensor = sensor2body.inverse()*world2body;

            } else if ((poseConfig & BoresightInfoBit) == Body2Sensor) {

                StereoVision::Geometry::RigidBodyTransform<T> body2sensor;
                body2sensor.r << boresight[0], boresight[1], boresight[2];
                body2sensor.t << leverArm[0], leverArm[1], leverArm[2];

                world2sensor = body2sensor*world2body;
            }

            for (int i = 0; i < 3; i++) {
                processed_r[i] = world2sensor.r[i];
                processed_t[i] = world2sensor.t[i];
            }

        }

        auto variadic_lambda = [this] (auto... params) {
            return FunctorT::operator()(params...);
        };

        return CallFromTuple::call(variadic_lambda, processedArgs);
    }
};

template<typename FunctorT, int boresightParamPos, int positionParamPos, int poseConfig = Sensor2Body | Body2World>
class ApplyBoresightRotation : private FunctorT {
public:
    template <typename ... P>
    ApplyBoresightRotation(P... args) :
        FunctorT(args...)
    {

    }

    template <typename ... P>
    bool operator()(P ... params) const {

        std::tuple<P...> args(params...);

        using T = std::remove_const_t<
            std::remove_pointer_t<
                std::remove_reference_t<decltype (std::get<0>(args))>
                >>;

        static_assert (boresightParamPos == positionParamPos, "boresightParamPos and positionParamPos overlap");
        static_assert (sizeof... (params) > std::max(boresightParamPos, positionParamPos)+1, "Wrong number of arguments provided");
        static_assert (((poseConfig & PoseInfoBit) == World2Body), "misconfigured DecoratorPoseConfiguration, Body2World will have no effect!");
        static_assert (((poseConfig & BoresightInfoBit) == Sensor2Body) or ((poseConfig & BoresightInfoBit) == Body2Sensor), "misconfigured DecoratorPoseConfiguration");

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        const T* const t = std::get<positionParamPos>(args);

        const T* const boresight = std::get<ApplyBoresightRotation>(args);

        T processed_t[3];

        //remove the old arguments and insert new arguments such that processed_r is
        // at pos poseParamGroupPos and processed_t at pos poseParamGroupPos+1.
        auto processedArgs =
            CallFromTuple::insertArgInTuple<positionParamPos, const T*>(
                CallFromTuple::removeArgFromTuple<positionParamPos>(args), processed_t
            );

        if ((poseConfig & PoseInfoBit) == Body2World) {

            StereoVision::Geometry::RigidBodyTransform<T> body2world;

            V3T body2worldT;
            body2worldT << t[0], t[1], t[2];

            //no effect
            for (int i = 0; i < 3; i++) {
                processed_t[i] = body2worldT[i];
            }

        } else if ((poseConfig & PoseInfoBit) == World2Body) {

            V3T world2bodyT;

            world2bodyT << t[0], t[1], t[2];

            V3T world2sensorT;

            if ((poseConfig & BoresightInfoBit) == Sensor2Body) {

                V3T sensor2bodyR;
                sensor2bodyR << boresight[0], boresight[1], boresight[2];

                world2sensorT = StereoVision::Geometry::angleAxisRotate(-sensor2bodyR, world2bodyT);

            } else if ((poseConfig & BoresightInfoBit) == Body2Sensor) {

                V3T body2sensorR;
                body2sensorR << boresight[0], boresight[1], boresight[2];

                world2sensorT = StereoVision::Geometry::angleAxisRotate(body2sensorR, world2bodyT);
            }

            for (int i = 0; i < 3; i++) {
                processed_t[i] = world2sensorT[i];
            }

        }

        auto variadic_lambda = [this] (auto... params) {
            return FunctorT::operator()(params...);
        };

        return CallFromTuple::call(variadic_lambda, processedArgs);
    }
};

template<typename FunctorT, int boresightParamPos, int rotationParamPos, int poseConfig = Sensor2Body | Body2World>
class ComposeBoresightRotation : private FunctorT {
public:
    template <typename ... P>
    ComposeBoresightRotation(P... args) :
        FunctorT(args...)
    {

    }

    template <typename ... P>
    bool operator()(P ... params) const {

        std::tuple<P...> args(params...);

        using T = std::remove_const_t<
            std::remove_pointer_t<
                std::remove_reference_t<decltype (std::get<0>(args))>
                >>;

        static_assert (rotationParamPos != boresightParamPos, "boresightParamPos and rotationParamPos overlap");
        static_assert (sizeof... (params) > std::max(rotationParamPos, boresightParamPos)+1, "Wrong number of arguments provided");
        static_assert (((poseConfig & PoseInfoBit) == Body2World) or ((poseConfig & PoseInfoBit) == World2Body), "misconfigured DecoratorPoseConfiguration");
        static_assert (((poseConfig & BoresightInfoBit) == Sensor2Body) or ((poseConfig & BoresightInfoBit) == Body2Sensor), "misconfigured DecoratorPoseConfiguration");

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        const T* const r = std::get<rotationParamPos>(args);

        const T* const boresight = std::get<boresightParamPos>(args);

        T processed_r[3];

        //remove the old arguments and insert new arguments such that processed_r is
        // at pos poseParamGroupPos and processed_t at pos poseParamGroupPos+1.
        auto processedArgs =
            CallFromTuple::insertArgInTuple<rotationParamPos, const T*>(
                CallFromTuple::removeArgFromTuple<rotationParamPos>(args), processed_r
            );

        if ((poseConfig & PoseInfoBit) == Body2World) {

            StereoVision::Geometry::RigidBodyTransform<T> body2world;

            body2world.r << r[0], r[1], r[2];
            body2world.t << T(0), T(0), T(0);

            StereoVision::Geometry::RigidBodyTransform<T> sensor2world;

            if ((poseConfig & BoresightInfoBit) == Sensor2Body) {

                StereoVision::Geometry::RigidBodyTransform<T> sensor2body;
                sensor2body.r << boresight[0], boresight[1], boresight[2];
                sensor2body.t << T(0), T(0), T(0);

                sensor2world = body2world*sensor2body;

            } else if ((poseConfig & BoresightInfoBit) == Body2Sensor) {

                StereoVision::Geometry::RigidBodyTransform<T> body2sensor;
                body2sensor.r << boresight[0], boresight[1], boresight[2];
                body2sensor.t << T(0), T(0), T(0);

                sensor2world = body2world*body2sensor.inverse();
            }

            for (int i = 0; i < 3; i++) {
                processed_r[i] = sensor2world.r[i];
            }

        } else if ((poseConfig & PoseInfoBit) == World2Body) {

            StereoVision::Geometry::RigidBodyTransform<T> world2body;

            world2body.r << r[0], r[1], r[2];
            world2body.t << T(0), T(0), T(0);

            StereoVision::Geometry::RigidBodyTransform<T> world2sensor;

            if ((poseConfig & BoresightInfoBit) == Sensor2Body) {

                StereoVision::Geometry::RigidBodyTransform<T> sensor2body;
                sensor2body.r << boresight[0], boresight[1], boresight[2];
                sensor2body.t << T(0), T(0), T(0);

                world2sensor = sensor2body.inverse()*world2body;

            } else if ((poseConfig & BoresightInfoBit) == Body2Sensor) {

                StereoVision::Geometry::RigidBodyTransform<T> body2sensor;
                body2sensor.r << boresight[0], boresight[1], boresight[2];
                body2sensor.t << T(0), T(0), T(0);

                world2sensor = body2sensor*world2body;
            }

            for (int i = 0; i < 3; i++) {
                processed_r[i] = world2sensor.r[i];
            }

        }

        auto variadic_lambda = [this] (auto... params) {
            return FunctorT::operator()(params...);
        };

        return CallFromTuple::call(variadic_lambda, processedArgs);
    }
};

/*!
 * \brief The IdentityDynamic class is a template class to ensure a dynamic functor inherit virtually BaseDynamicDecorator.
 */
template<typename DynamicFunctorT>
class IdentityDynamic : public DynamicFunctorT, public virtual BaseDynamicDecorator {

public:
    template <typename ... P>
    IdentityDynamic(P... args) :
        DynamicFunctorT(args...)
    {

    }
};

/*!
 * \brief The AddLeverArm class represent a decorator for the functor type FunctorT, adding a lever arm parameter
 *
 * The FunctorT type is assumed to have an operator() const taking as argument
 * "const T* const r, const T* const t" the orientation and position parameters of a pose,
 * and then additional arguments.
 * AddLeverArm<FunctorT> then has an operator() const taking as argument
 * "const T* const r, const T* const t, const T* const boresight, const T* const leverArm"
 * at the same position the pose is call in the original functor.
 * The lever arm is applied to the pose on the fly and then the underlying functor is called.
 *
 * The poseParamGroupPos is the position of the pose arguments in the argument pack.
 *
 * The poseConfig indicate how the boresight has to be applied.
 *
 * if poseConfig & PoseInfoBit == Body2World, then the boresight is used to compute Sensor2World, which is passed to the underlying functor.
 * if poseConfig & PoseInfoBit == World2Body, then the boresight is used to compute World2Sensor, which is passed to the underlying functor.
 *
 * if poseConfig & BoresightInfoBit == Sensor2Body, then the boresight is assumed to represent the Sensor2Body transform.
 * if poseConfig & BoresightInfoBit == Body2Sensor, then the boresight is assumed to represent the Body2Sensor transform.
 */
template<typename FunctorT, int poseConfig = Sensor2Body | Body2World, int poseParamGroupPos = 0>
using LeverArm = ApplyLeverArm<AddPose<FunctorT, poseParamGroupPos+2>, poseParamGroupPos+2, poseParamGroupPos, poseConfig>;

template<typename DynamicFunctorT, int poseConfig = Sensor2Body | Body2World, int poseParamGroupPos = 0>
class LeverArmDynamic : private DynamicFunctorT, public virtual BaseDynamicDecorator {

public:
    template <typename ... P>
    LeverArmDynamic(P... args) :
        DynamicFunctorT(args...)
    {

    }

    template <typename T, typename ... P>
    bool operator()(T const* const* parameters, T* residuals) const {

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        const T* const r = parameters[poseParamGroupPos];
        const T* const t = parameters[poseParamGroupPos+1];

        const T* const boresight = parameters[poseParamGroupPos+2];
        const T* const leverArm = parameters[poseParamGroupPos+3];

        T processed_r[3];
        T processed_t[3];

        if ((poseConfig & PoseInfoBit) == Body2World) {

            StereoVision::Geometry::RigidBodyTransform<T> body2world;

            body2world.r << r[0], r[1], r[2];
            body2world.t << t[0], t[1], t[2];

            StereoVision::Geometry::RigidBodyTransform<T> sensor2world;

            if ((poseConfig & BoresightInfoBit) == Sensor2Body) {

                StereoVision::Geometry::RigidBodyTransform<T> sensor2body;
                sensor2body.r << boresight[0], boresight[1], boresight[2];
                sensor2body.t << leverArm[0], leverArm[1], leverArm[2];

                sensor2world = body2world*sensor2body;

            } else if ((poseConfig & BoresightInfoBit) == Body2Sensor) {

                StereoVision::Geometry::RigidBodyTransform<T> body2sensor;
                body2sensor.r << boresight[0], boresight[1], boresight[2];
                body2sensor.t << leverArm[0], leverArm[1], leverArm[2];

                sensor2world = body2world*body2sensor.inverse();
            }

            for (int i = 0; i < 3; i++) {
                processed_r[i] = sensor2world.r[i];
                processed_t[i] = sensor2world.t[i];
            }

        } else if ((poseConfig & PoseInfoBit) == World2Body) {

            StereoVision::Geometry::RigidBodyTransform<T> world2body;

            world2body.r << r[0], r[1], r[2];
            world2body.t << t[0], t[1], t[2];

            StereoVision::Geometry::RigidBodyTransform<T> world2sensor;

            if ((poseConfig & BoresightInfoBit) == Sensor2Body) {

                StereoVision::Geometry::RigidBodyTransform<T> sensor2body;
                sensor2body.r << boresight[0], boresight[1], boresight[2];
                sensor2body.t << leverArm[0], leverArm[1], leverArm[2];

                world2sensor = sensor2body.inverse()*world2body;

            } else if ((poseConfig & BoresightInfoBit) == Body2Sensor) {

                StereoVision::Geometry::RigidBodyTransform<T> body2sensor;
                body2sensor.r << boresight[0], boresight[1], boresight[2];
                body2sensor.t << leverArm[0], leverArm[1], leverArm[2];

                world2sensor = body2sensor*world2body;
            }

            for (int i = 0; i < 3; i++) {
                processed_r[i] = world2sensor.r[i];
                processed_t[i] = world2sensor.t[i];
            }

        }

        std::vector<T const*> params(nParams()-2);

        for (int i = 0; i < nParams(); i++) {

            if (i == poseParamGroupPos) {

                params[i] = processed_r;
                continue;

            } else if (i == poseParamGroupPos+1) {

                params[i] = processed_t;
                continue;
            } else if (i == poseParamGroupPos+2 or i == poseParamGroupPos+3) {
                continue;
            }

            int pos = i;

            if (i > poseParamGroupPos) {
                pos -= 2;
            }

            params[pos] = parameters[i];
        }

        //indicate to subsequent functor that the number of parameters has been reduced by two.
        int oldNParams = nParams();
        setNParams(oldNParams-2);

        bool ok = DynamicFunctorT::operator()(params.data(), residuals);

        //reset the correct number of parameters
        setNParams(oldNParams);

        return ok;
    }
};

template<typename FunctorT, PoseTransformDirection poseTransformDirection = PoseTransformDirection::FinalToTarget, int poseParamGroupPos = 0>
class PoseTransform : private FunctorT {

public:
    template <typename ... P>
    PoseTransform(StereoVision::Geometry::RigidBodyTransform<double> const& transform, P... args) :
        FunctorT(args...),
        _transform(transform)
    {

    }

    template <typename ... P>
    bool operator()(P ... params) const {

        std::tuple<P...> args(params...);

        using T = std::remove_const_t<
        std::remove_pointer_t<
        std::remove_reference_t<decltype (std::get<0>(args))>
        >>;

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        const T* const r = std::get<poseParamGroupPos>(args);
        const T* const t = std::get<poseParamGroupPos+1>(args);

        T processed_r[3];
        T processed_t[3];

        //remove the old arguments and insert new arguments such that processed_r is
        // at pos poseParamGroupPos and processed_t at pos poseParamGroupPos+1.
        auto processedArgs =
                CallFromTuple::insertArgInTuple<poseParamGroupPos, const T*>(
                CallFromTuple::insertArgInTuple<poseParamGroupPos, const T*>(
                CallFromTuple::removeArgFromTuple<poseParamGroupPos>(
                    CallFromTuple::removeArgFromTuple<poseParamGroupPos>(
                        args)
                    ), processed_t
                    ), processed_r
                );

        StereoVision::Geometry::RigidBodyTransform<T> pose;

        pose.r << r[0], r[1], r[2];
        pose.t << t[0], t[1], t[2];

        StereoVision::Geometry::RigidBodyTransform<T> transformed;

        if (poseTransformDirection == PoseTransformDirection::FinalToTarget) {
            transformed = _transform.cast<T>() * pose;
        }

        if (poseTransformDirection == PoseTransformDirection::SourceToInitial) {
            transformed = pose * _transform.cast<T>();
        }

        for (int i = 0; i < 3; i++) {
            processed_r[i] = transformed.r[i];
            processed_t[i] = transformed.t[i];
        }

        auto variadic_lambda = [this] (auto... params) {
            return FunctorT::operator()(params...);
        };

        return CallFromTuple::call(variadic_lambda, processedArgs);

    }

protected:

    StereoVision::Geometry::RigidBodyTransform<double> _transform;

};

template<typename DynamicFunctorT, PoseTransformDirection poseTransformDirection = PoseTransformDirection::FinalToTarget, int poseParamGroupPos = 0>
class PoseTransformDynamic : private DynamicFunctorT, public virtual BaseDynamicDecorator {

public:
    template <typename ... P>
    PoseTransformDynamic(StereoVision::Geometry::RigidBodyTransform<double> const& transform, P... args) :
        DynamicFunctorT(args...),
        _transform(transform)
    {

    }

    template <typename T, typename ... P>
    bool operator()(T const* const* parameters, T* residuals) const {

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        const T* const r = parameters[poseParamGroupPos];
        const T* const t = parameters[poseParamGroupPos+1];

        StereoVision::Geometry::RigidBodyTransform<T> pose;

        pose.r << r[0], r[1], r[2];
        pose.t << t[0], t[1], t[2];

        StereoVision::Geometry::RigidBodyTransform<T> transformed;

        if (poseTransformDirection == PoseTransformDirection::FinalToTarget) {
            transformed = _transform.cast<T>() * pose;
        }

        if (poseTransformDirection == PoseTransformDirection::SourceToInitial) {
            transformed = pose * _transform.cast<T>();
        }

        std::vector<T const*> params(nParams());

        for (int i = 0; i < nParams(); i++) {

            if (i == poseParamGroupPos) {

                params[i] = transformed.r.data();
                continue;

            } else if (i == poseParamGroupPos+1) {

                params[i] = transformed.t.data();
                continue;
            }

            params[i] = parameters[i];
        }

        return DynamicFunctorT::operator()(params.data(), residuals);
    }

protected:

    StereoVision::Geometry::RigidBodyTransform<double> _transform;
};

/*!
 * \brief The LocalFrame2MappedPoint class is a decorator using the pose of a local system to map a point to mapping frame
 *
 * It can occur that we know the position of a point relative to a local coordinate system,
 * and we want to project that point to a UV coordinate. This decorator replace an optimizable 3D point parameter by a pose,
 * use the pose to compute the position of a point in mapping frame and pass down this pose to the underlying functor.
 */
template<typename FunctorT, DecoratorPoseConfiguration poseOrder = DecoratorPoseConfiguration::Body2World, int pointParamPos = 0>
class LocalFrame2MappedPoint : private FunctorT {

public:
    template <typename ... P>
    LocalFrame2MappedPoint(Eigen::Vector3d const& localPointPos, P... args) :
        FunctorT(args...),
        _localPoint(localPointPos)
    {

    }

    template <typename ... P>
    bool operator()(P ... params) const {

        std::tuple<P...> args(params...);

        using T = std::remove_const_t<
            std::remove_pointer_t<
                std::remove_reference_t<decltype (std::get<0>(args))>
                >>;

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        const T* const r = std::get<pointParamPos>(args);
        const T* const t = std::get<pointParamPos+1>(args);

        T processed_t[3];

        //remove the old arguments and insert new arguments such that processed_r is
        // at pos poseParamGroupPos and processed_t at pos poseParamGroupPos+1.
        auto processedArgs =
            CallFromTuple::insertArgInTuple<pointParamPos, const T*>(
                CallFromTuple::removeArgFromTuple<pointParamPos>(
                    CallFromTuple::removeArgFromTuple<pointParamPos>(
                            args)
                    ), processed_t
                );

        StereoVision::Geometry::RigidBodyTransform<T> pose;

        pose.r << r[0], r[1], r[2];
        pose.t << t[0], t[1], t[2];

        if ((poseOrder | PoseInfoBit) == World2Body) {
            pose = pose.inverse(); //ensure we have the pose as body2world
        }

        V3T transformed = pose*_localPoint.cast<T>();

        for (int i = 0; i < 3; i++) {
            processed_t[i] = transformed[i];
        }

        auto variadic_lambda = [this] (auto... params) {
            return FunctorT::operator()(params...);
        };

        return CallFromTuple::call(variadic_lambda, processedArgs);

    }

protected:

    Eigen::Vector3d _localPoint;
};

template<typename DynamicFunctorT, DecoratorPoseConfiguration poseOrder = DecoratorPoseConfiguration::Body2World, int pointParamPos = 0>
class LocalFrame2MappedPointDynamic : private DynamicFunctorT, public virtual BaseDynamicDecorator {

public:
    template <typename ... P>
    LocalFrame2MappedPointDynamic(Eigen::Vector3d const& localPointPos, P... args) :
        DynamicFunctorT(args...),
        _localPoint(localPointPos)
    {

    }

    template <typename T>
    bool operator()(T const* const* parameters, T* residuals) const {

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        const T* const r = parameters[pointParamPos];
        const T* const t = parameters[pointParamPos+1];

        T processed_t[3];

        StereoVision::Geometry::RigidBodyTransform<T> pose;

        pose.r << r[0], r[1], r[2];
        pose.t << t[0], t[1], t[2];

        if ((poseOrder | PoseInfoBit) == World2Body) {
            pose = pose.inverse(); //ensure we have the pose as body2world
        }

        V3T transformed = pose*_localPoint.cast<T>();

        for (int i = 0; i < 3; i++) {
            processed_t[i] = transformed[i];
        }

        std::vector<T const*> params(nParams()-1);

        for (int i = 0; i < nParams(); i++) {

            if (i == pointParamPos) {

                params[i] = processed_t;
                continue;

            } else if (i == pointParamPos+1) {
                continue;
            }

            int pos = i;

            if (i > pointParamPos) {
                pos -= 1;
            }

            params[pos] = parameters[i];
        }

        return DynamicFunctorT::operator()(params.data(), residuals);

    }

protected:

    Eigen::Vector3d _localPoint;
};

/*!
 * \brief The InterpolatedPose class represent a decorator for the functor type FunctorT, interpolating between two poses.
 *
 * The FunctorT type is assumed to have an operator() const taking as argument
 * "const T* const r, const T* const t" the orientation and position parameters of a pose,
 * and then additional arguments.
 * InterpolatedPose<FunctorT> then has an operator() const taking as argument
 * "const T* const r1, const T* const t1, const T* const r2, const T* const t2",
 * and then the same additional arguments. The pose passed to the underlying functor is
 * interpolated from r1,t1 and r2,t2 using wheights w1 and w2 passed to the constructor.
 * Interpolation is done on the SO(3) manifold.
 *
 * The poseParamGroupPos is the position of the pose arguments in the argument pack.
 */
template<typename FunctorT, int poseParamGroupPos = 0>
class InterpolatedPose : private FunctorT
{
public:

    template <typename ... P>
    InterpolatedPose(double w1, double w2, P... args) :
        FunctorT(args...),
        _w1(w1),
        _w2(w2)
    {

    }

    template <typename ... P>
    bool operator()(P ... params) const {

        std::tuple<P...> args(params...);

        using T = std::remove_const_t<
        std::remove_pointer_t<
        std::remove_reference_t<decltype (std::get<0>(args))>
        >>;

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        const T* const r1 = std::get<poseParamGroupPos>(args);
        const T* const t1 = std::get<poseParamGroupPos+1>(args);

        const T* const r2 = std::get<poseParamGroupPos+2>(args);
        const T* const t2 = std::get<poseParamGroupPos+3>(args);

        V3T rot1;
        rot1 << r1[0], r1[1], r1[2];
        V3T tran1;
        tran1 << t1[0], t1[1], t1[2];
        StereoVision::Geometry::RigidBodyTransform<T> transform1(rot1,tran1);

        V3T rot2;
        rot2 << r2[0], r2[1], r2[2];
        V3T tran2;
        tran2 << t2[0], t2[1], t2[2];
        StereoVision::Geometry::RigidBodyTransform<T> transform2(rot2,tran2);

        StereoVision::Geometry::RigidBodyTransform<T> interpolated =
                StereoVision::Geometry::interpolateRigidBodyTransformOnManifold(T(_w1), transform1, T(_w2), transform2);

        T processed_r[3];
        T processed_t[3];

        //remove the old arguments and insert new arguments such that processed_r is
        // at pos leverArmParamGroupPos and processed_t at pos leverArmParamGroupPos+1.
        auto processedArgs =
                CallFromTuple::insertArgInTuple<poseParamGroupPos, const T*>(
                CallFromTuple::insertArgInTuple<poseParamGroupPos, const T*>(
                CallFromTuple::removeArgFromTuple<poseParamGroupPos>(
                    CallFromTuple::removeArgFromTuple<poseParamGroupPos>(
                        CallFromTuple::removeArgFromTuple<poseParamGroupPos>(
                            CallFromTuple::removeArgFromTuple<poseParamGroupPos>(args)
                            )
                        )
                    ), processed_t
                    ), processed_r
                );

        for (int i = 0; i < 3; i++) {
            processed_r[i] = interpolated.r[i];
            processed_t[i] = interpolated.t[i];
        }

        auto variadic_lambda = [this] (auto... params) {
            return FunctorT::operator()(params...);
        };

        return CallFromTuple::call(variadic_lambda, processedArgs);

    }

protected:

    double _w1;
    double _w2;

};

template<typename DynamicFunctorT, int poseParamGroupPos = 0>
class InterpolatedPoseDynamic : private DynamicFunctorT, public virtual BaseDynamicDecorator
{
public:

    template <typename ... P>
    InterpolatedPoseDynamic(double w1, double w2, P... args) :
        DynamicFunctorT(args...),
        _w1(w1),
        _w2(w2)
    {

    }

    template <typename T, typename ... P>
    bool operator()(T const* const* parameters, T* residuals) const {

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        const T* const r1 = parameters[poseParamGroupPos];
        const T* const t1 = parameters[poseParamGroupPos+1];

        const T* const r2 = parameters[poseParamGroupPos+2];
        const T* const t2 = parameters[poseParamGroupPos+3];

        V3T rot1;
        rot1 << r1[0], r1[1], r1[2];
        V3T tran1;
        tran1 << t1[0], t1[1], t1[2];
        StereoVision::Geometry::RigidBodyTransform<T> transform1(rot1,tran1);

        V3T rot2;
        rot2 << r2[0], r2[1], r2[2];
        V3T tran2;
        tran2 << t2[0], t2[1], t2[2];
        StereoVision::Geometry::RigidBodyTransform<T> transform2(rot2,tran2);

        StereoVision::Geometry::RigidBodyTransform<T> interpolated =
            StereoVision::Geometry::interpolateRigidBodyTransformOnManifold(T(_w1), transform1, T(_w2), transform2);


        std::vector<T const*> params(nParams()-2);

        for (int i = 0; i < nParams(); i++) {

            if (i == poseParamGroupPos) {

                params[i] = interpolated.r.data();
                continue;

            } else if (i == poseParamGroupPos+1) {

                params[i] = interpolated.t.data();
                continue;
            } else if (i == poseParamGroupPos+2 or i == poseParamGroupPos+3) {
                continue;
            }

            int pos = i;

            if (i > poseParamGroupPos) {
                pos -= 2;
            }

            params[pos] = parameters[i];
        }

        return DynamicFunctorT::operator()(params.data(), residuals);
    }

protected:

    double _w1;
    double _w2;
};


/*!
 * \brief The InvertPose class represent a decorator for the functor type FunctorT, inverting the input pose.
 *
 * The FunctorT type is assumed to have an operator() const taking as argument
 * "const T* const r, const T* const t" the orientation and position parameters of a pose,
 * and then additional arguments.
 * InterpolatedPose<FunctorT> then has an operator() const taking the same arguments
 * as FunctorT. The pose passed to the underlying functor is
 * the inverted posed passed as input to InterpolatedPose<FunctorT>.
 * So if you have a functor taking as input world2body but your parameters are
 * expressed as body2world, then this decorator can perform the conversion on the fly.
 *
 * The poseParamGroupPos is the position of the pose arguments in the argument pack.
 */
template<typename FunctorT, int poseParamGroupPos = 0>
class InvertPose : private FunctorT
{
public:

    template <typename ... P>
    InvertPose(P... args) :
        FunctorT(args...)
    {

    }

    template <typename ... P>
    bool operator()(P ... params) const {

        std::tuple<P...> args(params...);

        using T = std::remove_const_t<
        std::remove_pointer_t<
        std::remove_reference_t<decltype (std::get<0>(args))>
        >>;

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        const T* const r = std::get<poseParamGroupPos>(args);
        const T* const t = std::get<poseParamGroupPos+1>(args);

        V3T rot;
        rot << r[0], r[1], r[2];
        V3T tran;
        tran << t[0], t[1], t[2];
        StereoVision::Geometry::RigidBodyTransform<T> pose(rot,tran);

        StereoVision::Geometry::RigidBodyTransform<T> inverted = pose.inverse();

        T processed_r[3];
        T processed_t[3];

        //remove the old arguments and insert new arguments such that processed_r is
        // at pos poseParamGroupPos and processed_t at pos poseParamGroupPos+1.
        auto processedArgs =
                CallFromTuple::insertArgInTuple<poseParamGroupPos, const T*>(
                CallFromTuple::insertArgInTuple<poseParamGroupPos, const T*>(
                CallFromTuple::removeArgFromTuple<poseParamGroupPos>(
                    CallFromTuple::removeArgFromTuple<poseParamGroupPos>(
                        args)
                    ), processed_t
                    ), processed_r
                );

        for (int i = 0; i < 3; i++) {
            processed_r[i] = inverted.r[i];
            processed_t[i] = inverted.t[i];
        }

        auto variadic_lambda = [this] (auto... params) {
            return FunctorT::operator()(params...);
        };

        return CallFromTuple::call(variadic_lambda, processedArgs);

    }

};

/*!
 * \brief The InvertPoseDynamic class act like the InvertPose decorator, but for a Functor designed for DynamicAutoDiffCostFunction
 */
template<typename DynamicFunctorT, int poseParamGroupIdx = 0>
class InvertPoseDynamic : private DynamicFunctorT, public virtual BaseDynamicDecorator
{
public:

    template <typename ... P>
    InvertPoseDynamic(P... args) :
        DynamicFunctorT(args...)
    {

    }

    template <typename T, typename ... P>
    bool operator()(T const* const* parameters, T* residuals) const {

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        const T* const r = parameters[poseParamGroupIdx];
        const T* const t = parameters[poseParamGroupIdx+1];

        V3T rot;
        rot << r[0], r[1], r[2];
        V3T tran;
        tran << t[0], t[1], t[2];
        StereoVision::Geometry::RigidBodyTransform<T> pose(rot,tran);

        StereoVision::Geometry::RigidBodyTransform<T> inverted = pose.inverse();

        std::vector<T const*> params(nParams());

        for (int i = 0; i < nParams(); i++) {

            if (i == poseParamGroupIdx) {

                params[i] = inverted.r.data();
                continue;

            } else if (i == poseParamGroupIdx+1) {

                params[i] = inverted.t.data();
                continue;
            }

            params[i] = parameters[i];
        }

        return DynamicFunctorT::operator()(params.data(), residuals);
    }
};

template<typename FT, PoseTransformDirection poseTransformDirection, int leverArmConfig, int pParamId, int nRes, int... argsSizes>
class ModifiedPoseCostFunctionBuilderHelper {

public:

    using FunctorT = FT;
    static constexpr int nArgs = sizeof...(argsSizes);
    static constexpr int poseParamId = pParamId;
    static constexpr int poseRotParamId = pParamId;
    static constexpr int posePosParamId = pParamId+1;
    static constexpr int leverArmRotParamId = pParamId+2;
    static constexpr int leverArmPosParamId = pParamId+3;

    using LeverArmCost = LeverArm<FunctorT,leverArmConfig,0>;
    using PoseTransformCost = PoseTransform<FunctorT,poseTransformDirection,0>;
    using PoseTransformLeverArmCost = PoseTransform<LeverArmCost,poseTransformDirection,0>;

    using LeverArmCostDynamic = LeverArmDynamic<FunctorT,leverArmConfig,0>;
    using PoseTransformCostDynamic = PoseTransformDynamic<FunctorT,poseTransformDirection,0>;
    using PoseTransformLeverArmCostDynamic = PoseTransformDynamic<LeverArmCostDynamic,poseTransformDirection,0>;

protected:

    template<int n>
    struct counter {

    };

    template<typename counterT, size_t nR, typename CFType, typename SequenceT, int... argsS>
    struct leverArmAutoDiffCostHelper {
        using CostFuncT = void;
    };

    template<size_t posId, size_t nR, typename CFType, int... argsP, int arg0, int... argsS>
    struct leverArmAutoDiffCostHelper<counter<posId>, nR, CFType, std::integer_sequence<int, argsP...>, arg0, argsS...> {
        using CostFuncT = std::conditional_t<posId == 0,
                                             ceres::AutoDiffCostFunction<CFType, nR, argsP..., 3, 3, arg0, argsS...>,
                                             typename leverArmAutoDiffCostHelper<std::conditional_t<posId >= 2, counter<std::max<int>(posId-1,1)>, void>, nR, CFType, std::integer_sequence<int, argsP..., arg0>, argsS...>::CostFuncT>;
    };

    template<size_t nR, typename CFType, int... argsP>
    struct leverArmAutoDiffCostHelper<counter<0>, nR, CFType, std::integer_sequence<int, argsP...>> {
        using CostFuncT = ceres::AutoDiffCostFunction<CFType, nR, argsP..., 3, 3>;
    };

public:

    struct CostFunctionData {
        ceres::CostFunction* costFunction;
        std::vector<double*> params;
    };

    template<typename ... T>
    static CostFunctionData buildPoseShiftedCostFunction(double ** parameters,
                                              const StereoVision::Geometry::RigidBodyTransform<double> &offset,
                                              double *leverArmOrientation,
                                              double *leverArmPosition,
                                              T... constructorArgs) {



        if (offset.r.norm() < 1e-6 and offset.t.norm() < 1e-6) {

            if (leverArmOrientation != nullptr and leverArmPosition != nullptr) {

                LeverArmCost* costFunctor = new LeverArmCost(constructorArgs...);

                constexpr int modifiedNArgs = nArgs+2;

                std::vector<double*> params(modifiedNArgs);

                int modifiedArgsId = 0;

                for (int i = 0; i < nArgs; i++) {
                    params[modifiedArgsId] = parameters[i];

                    if (i == poseParamId+1) {
                        modifiedArgsId++;
                        params[modifiedArgsId] = leverArmOrientation;
                        modifiedArgsId++;
                        params[modifiedArgsId] = leverArmPosition;
                    }

                    modifiedArgsId++;
                }

                using lACersCF = typename leverArmAutoDiffCostHelper<counter<poseParamId>, nRes, LeverArmCost, std::integer_sequence<int>, argsSizes...>::CostFuncT;
                ceres::CostFunction* costFunc = new lACersCF(costFunctor);

                return {costFunc, params};

            } else {
                FunctorT* costFunctor = new FunctorT(constructorArgs...);

                std::vector<double*> params(nArgs);

                for (int i = 0; i < nArgs; i++) {
                    params[i] = parameters[i];
                }

                ceres::CostFunction* costFunc = new ceres::AutoDiffCostFunction<FunctorT, nRes, argsSizes...>(costFunctor);

                return {costFunc, params};
            }

        } else {

            if (leverArmOrientation != nullptr and leverArmPosition != nullptr) {

                PoseTransformLeverArmCost* costFunctor =
                    new PoseTransformLeverArmCost(
                        offset, constructorArgs...);

                constexpr int modifiedNArgs = nArgs+2;

                std::vector<double*> params(modifiedNArgs);

                int modifiedArgsId = 0;

                for (int i = 0; i < nArgs; i++) {
                    params[modifiedArgsId] = parameters[i];

                    if (i == poseParamId+1) {
                        modifiedArgsId++;
                        params[modifiedArgsId] = leverArmOrientation;
                        modifiedArgsId++;
                        params[modifiedArgsId] = leverArmPosition;
                    }

                    modifiedArgsId++;
                }

                using lACersCF = typename leverArmAutoDiffCostHelper<counter<poseParamId>, nRes, PoseTransformLeverArmCost, std::integer_sequence<int>, argsSizes...>::CostFuncT;
                ceres::CostFunction* costFunc = new lACersCF(costFunctor);

                return {costFunc, params};

            } else {

                PoseTransformCost* costFunctor = new PoseTransformCost(offset, constructorArgs...);

                std::vector<double*> params(nArgs);

                for (int i = 0; i < nArgs; i++) {
                    params[i] = parameters[i];
                }

                ceres::CostFunction* costFunc = new ceres::AutoDiffCostFunction<PoseTransformCost, nRes, argsSizes...>(costFunctor);

                return {costFunc, params};

            }
        }

        return {nullptr, std::vector<double*>()};

    }

    template<int stride = 4, typename ... T>
    static CostFunctionData buildPoseShiftedDynamicCostFunction(double ** parameters,
                                                                std::vector<int> const& parametersSizeInfos,
                                                                const StereoVision::Geometry::RigidBodyTransform<double> &offset,
                                                                double *leverArmOrientation,
                                                                double *leverArmPosition,
                                                                T... constructorArgs) {



        if (offset.r.norm() < 1e-6 and offset.t.norm() < 1e-6) {

            if (leverArmOrientation != nullptr and leverArmPosition != nullptr) {

                LeverArmCostDynamic* costFunctor = new LeverArmCostDynamic(constructorArgs...);

                int nArgs = parametersSizeInfos.size();

                int modifiedNArgs = nArgs+2;

                costFunctor->setNParams(modifiedNArgs);

                ceres::DynamicAutoDiffCostFunction<LeverArmCostDynamic, stride>* costFunc =
                    new ceres::DynamicAutoDiffCostFunction<LeverArmCostDynamic, stride>(costFunctor);

                std::vector<double*> params(modifiedNArgs);

                int modifiedArgsId = 0;

                for (int i = 0; i < nArgs; i++) {
                    params[modifiedArgsId] = parameters[i];
                    costFunc->AddParameterBlock(parametersSizeInfos[i]);

                    if (i == poseParamId+1) {
                        modifiedArgsId++;
                        params[modifiedArgsId] = leverArmOrientation;
                        costFunc->AddParameterBlock(3);
                        modifiedArgsId++;
                        params[modifiedArgsId] = leverArmPosition;
                        costFunc->AddParameterBlock(3);
                    }

                    modifiedArgsId++;
                }

                costFunc->SetNumResiduals(nRes);

                return {costFunc, params};

            } else {
                FunctorT* costFunctor = new FunctorT(constructorArgs...);

                ceres::DynamicAutoDiffCostFunction<FunctorT, stride>* costFunc =
                    new ceres::DynamicAutoDiffCostFunction<FunctorT, stride>(costFunctor);

                int nArgs = parametersSizeInfos.size();

                std::vector<double*> params(nArgs);

                for (int i = 0; i < nArgs; i++) {
                    params[i] = parameters[i];
                    costFunc->AddParameterBlock(parametersSizeInfos[i]);
                }

                costFunc->SetNumResiduals(nRes);

                return {costFunc, params};
            }

        } else {

            if (leverArmOrientation != nullptr and leverArmPosition != nullptr) {

                PoseTransformLeverArmCostDynamic* costFunctor =
                    new PoseTransformLeverArmCostDynamic(
                        offset, constructorArgs...);

                int nArgs = parametersSizeInfos.size();

                int modifiedNArgs = nArgs+2;

                costFunctor->setNParams(modifiedNArgs);

                ceres::DynamicAutoDiffCostFunction<PoseTransformLeverArmCostDynamic, stride>* costFunc =
                    new ceres::DynamicAutoDiffCostFunction<PoseTransformLeverArmCostDynamic, stride>(costFunctor);

                std::vector<double*> params(modifiedNArgs);

                int modifiedArgsId = 0;

                for (int i = 0; i < nArgs; i++) {
                    params[modifiedArgsId] = parameters[i];
                    costFunc->AddParameterBlock(parametersSizeInfos[i]);

                    if (i == poseParamId+1) {
                        modifiedArgsId++;
                        params[modifiedArgsId] = leverArmOrientation;
                        costFunc->AddParameterBlock(3);
                        modifiedArgsId++;
                        params[modifiedArgsId] = leverArmPosition;
                        costFunc->AddParameterBlock(3);
                    }

                    modifiedArgsId++;
                }

                costFunc->SetNumResiduals(nRes);

                return {costFunc, params};

            } else {

                int nArgs = parametersSizeInfos.size();

                PoseTransformCostDynamic* costFunctor = new PoseTransformCostDynamic(offset, constructorArgs...);
                costFunctor->setNParams(nArgs);

                ceres::DynamicAutoDiffCostFunction<PoseTransformCostDynamic, stride>* costFunc =
                    new ceres::DynamicAutoDiffCostFunction<PoseTransformCostDynamic, stride>(costFunctor);

                std::vector<double*> params(nArgs);

                for (int i = 0; i < nArgs; i++) {
                    params[i] = parameters[i];
                    costFunc->AddParameterBlock(parametersSizeInfos[i]);
                }

                costFunc->SetNumResiduals(nRes);

                return {costFunc, params};

            }
        }

        return {nullptr, std::vector<double*>()};

    }

};

template<typename FT, PoseTransformDirection poseTransformDirection, int leverArmConfig, typename PoseParamsPos, int nRes, int... argsSizes>
class ModifiedMultiPoseCostFunctionBuilderHelper {
public:

    using FunctorT = FT;

    struct CostFunctionData {
        ceres::CostFunction* costFunction;
        std::vector<double*> params;
    };

    struct PoseModificationData {
        const StereoVision::Geometry::RigidBodyTransform<double> offset; //fixed offset applied on top of the parametrized pose.
        double *leverArmOrientation; //parameter for the lever arm orientation
        double *leverArmPosition; //parameter for the lever arm orientation
    };

protected:

    template<typename CounterT>
    struct IndicesData {
        static constexpr std::array<int, 0> indices = {};
    };

    template<typename T, T ... idxs>
    struct IndicesData<std::integer_sequence<T, idxs...>> {
        static constexpr std::array<size_t, sizeof...(idxs)> indices = {static_cast<size_t>(idxs)...};
    };

    template<typename Container>
    static constexpr bool checkIndices(Container const& indices) {
        for (size_t i = 0; i < indices.size()-1; i++) {
            if (indices[i]+2 > indices[i+1]) {
                return false;
            }
        }

        return true;
    }

    static_assert(IndicesData<PoseParamsPos>::indices.size() > 0, "PoseParamsPos must be an integer sequence indicating the position of the poses parameters in the cost function");
    static_assert(checkIndices(IndicesData<PoseParamsPos>::indices), "PoseParamsPos must give indices in increasing order, and indices should be at least 2 position appart to accomodate the pose parameters!");

    using PoseDataContainerInternal = std::array<PoseModificationData, IndicesData<PoseParamsPos>::indices.size()>;

    template<typename FunctorT, int currentId = 0>
    struct DynamicCostFunctionBuilder {

        template<int stride = 4, int PosePosOffset = 0, typename ... T>
        static CostFunctionData buildCostFunction(double ** parameters,
                                           std::vector<int> const& parametersSizeInfos,
                                           PoseDataContainerInternal const& posesData,
                                           T... constructorArgs) {

            constexpr int currentPosePos = IndicesData<PoseParamsPos>::indices.at(std::min<size_t>(currentId,
                                                                                                   IndicesData<PoseParamsPos>::indices.size()-1)) + PosePosOffset;
            using PoseT = decltype(PoseModificationData::offset);

            if constexpr (currentId >= IndicesData<PoseParamsPos>::indices.size()) {

                int FinalNParams = parametersSizeInfos.size();

                for (PoseModificationData const& poseData : posesData) {
                    if (poseData.leverArmOrientation != nullptr and poseData.leverArmPosition != nullptr) {
                        FinalNParams += 2;
                    }
                }

                using CostFunction = ceres::DynamicAutoDiffCostFunction<IdentityDynamic<FunctorT>, stride>;

                IdentityDynamic<FunctorT>* costFunctor = new IdentityDynamic<FunctorT>(constructorArgs...);

                costFunctor->setNParams(FinalNParams);

                CostFunction* costFunc = new CostFunction(costFunctor);

                CostFunctionData ret;

                ret.costFunction = costFunc;

                ret.params.resize(FinalNParams);
                int fParamIdx = 0;
                int poseIdxIdx = 0;

                for (size_t i = 0; i < parametersSizeInfos.size(); i++) {
                    ret.params[fParamIdx] = parameters[i];
                    costFunc->AddParameterBlock(parametersSizeInfos[i]);
                    fParamIdx++;

                    if (i == IndicesData<PoseParamsPos>::indices[poseIdxIdx]) {

                        //add the second parameter of the pose
                        i++;
                        ret.params[fParamIdx] = parameters[i];
                        costFunc->AddParameterBlock(parametersSizeInfos[i]);
                        fParamIdx++;

                        //add the lever arm parameters, if required

                        PoseModificationData const& poseData = posesData[poseIdxIdx];

                        if (poseData.leverArmOrientation != nullptr and poseData.leverArmPosition != nullptr) {

                            ret.params[fParamIdx] = poseData.leverArmOrientation;
                            costFunc->AddParameterBlock(3);
                            fParamIdx++;

                            ret.params[fParamIdx] = poseData.leverArmPosition;
                            costFunc->AddParameterBlock(3);
                            fParamIdx++;

                        }

                        //move to the next pose, if required
                        if (poseIdxIdx < IndicesData<PoseParamsPos>::indices.size()-1) {
                            poseIdxIdx++;
                        }
                    }
                }

                costFunc->SetNumResiduals(nRes);

                return ret;

            } else {

                if (posesData[currentId].offset.r.norm() < 1e-6 and posesData[currentId].offset.t.norm() < 1e-6) {

                    if (posesData[currentId].leverArmOrientation != nullptr and posesData[currentId].leverArmPosition != nullptr) {

                        using DecoratedFunctor = LeverArmDynamic<FunctorT,
                                                                 leverArmConfig,
                                                                 currentPosePos>;

                        return DynamicCostFunctionBuilder<DecoratedFunctor, currentId+1>::template buildCostFunction<stride, PosePosOffset+2,T...>
                            (parameters, parametersSizeInfos, posesData, constructorArgs...);

                    } else {

                        using DecoratedFunctor = FunctorT;

                        return DynamicCostFunctionBuilder<DecoratedFunctor, currentId+1>::template buildCostFunction<stride, PosePosOffset,T...>
                            (parameters, parametersSizeInfos, posesData, constructorArgs...);

                    }

                } else {

                    if (posesData[currentId].leverArmOrientation != nullptr and posesData[currentId].leverArmPosition != nullptr) {

                        using DecoratedFunctor = LeverArmDynamic<PoseTransformDynamic<FunctorT,poseTransformDirection,currentPosePos>, leverArmConfig, currentPosePos>;

                        return DynamicCostFunctionBuilder<DecoratedFunctor, currentId+1>::template buildCostFunction<stride, PosePosOffset+2, PoseT const& , T...>
                            (parameters, parametersSizeInfos, posesData, posesData[currentId].offset, constructorArgs...);

                    } else {

                        using DecoratedFunctor = PoseTransformDynamic<FunctorT,poseTransformDirection,currentPosePos>;

                        return DynamicCostFunctionBuilder<DecoratedFunctor, currentId+1>::template buildCostFunction<stride, PosePosOffset, PoseT const& , T...>
                            (parameters, parametersSizeInfos, posesData, posesData[currentId].offset, constructorArgs...);

                    }
                }
            }


        }
    };

public:

    static constexpr int NModifiedPoses = IndicesData<PoseParamsPos>::indices.size();
    using PoseDataContainer = PoseDataContainerInternal;

    template<int stride = 4, typename ... T>
    static CostFunctionData buildPoseShiftedDynamicCostFunction(double ** parameters,
                                                                std::vector<int> const& parametersSizeInfos,
                                                                PoseDataContainerInternal const& posesData,
                                                                T... constructorArgs) {

        constexpr int strides = 4;
        return DynamicCostFunctionBuilder<FunctorT>::template buildCostFunction<stride>(parameters, parametersSizeInfos, posesData, constructorArgs...);

    }
};

} // namespace StereoVisionApp

#endif // POSEDECORATORFUNCTORS_H
