#ifndef POSEDECORATORFUNCTORS_H
#define POSEDECORATORFUNCTORS_H

#include <ceres/jet.h>
#include <Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>
#include <StereoVision/geometry/alignement.h>

#include <type_traits>

#include "../../utils/functional.h"

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
 * \brief The AddParam class is a decorator which ass a unused param to a functor which can be used by other decorators further down the line.
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

} // namespace StereoVisionApp

#endif // POSEDECORATORFUNCTORS_H
