#ifndef POSEDECORATORFUNCTORS_H
#define POSEDECORATORFUNCTORS_H

#include <ceres/jet.h>
#include <Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>
#include <StereoVision/geometry/alignement.h>

#include <type_traits>

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
 * \brief The AddLeverArm class represent a decorator for the functor type FunctorT, adding a lever arm parameter
 *
 * The FunctorT type is assumed to have an operator() const taking as first argument
 * "const T* const r, const T* const t" the orientation and position parameters of a pose,
 * and then additional arguments.
 * AddLeverArm<FunctorT> then has an operator() const taking as first argument
 * "const T* const r, const T* const t, const T* const boresight, const T* const leverArm",
 * and then the same additional arguments. The lever arm is applied to the pose on the
 * fly and then the underlying functor is called.
 *
 * The poseConfig indicate how the boresight has to be applied.
 *
 * if poseConfig & PoseInfoBit == Body2World, then the boresight is used to compute Sensor2World, which is passed to the underlying functor.
 * if poseConfig & PoseInfoBit == World2Body, then the boresight is used to compute World2Sensor, which is passed to the underlying functor.
 *
 * if poseConfig & BoresightInfoBit == Sensor2Body, then the boresight is assumed to represent the Sensor2Body transform.
 * if poseConfig & BoresightInfoBit == Body2Sensor, then the boresight is assumed to represent the Body2Sensor transform.
 */
template<typename FunctorT, int poseConfig = Sensor2Body | Body2World>
class LeverArm : private FunctorT
{
public:

    template <typename ... P>
    LeverArm(P... args) :
        FunctorT(args...)
    {

    }

    template <typename T, typename ... P>
    bool operator()(const T* const r,
                    const T* const t,
                    const T* const boresight,
                    const T* const leverArm,
                    P ... additionalParams) const {

        static_assert (((poseConfig & PoseInfoBit) == Body2World) or ((poseConfig & PoseInfoBit) == World2Body), "misconfigured DecoratorPoseConfiguration");
        static_assert (((poseConfig & BoresightInfoBit) == Sensor2Body) or ((poseConfig & BoresightInfoBit) == Body2Sensor), "misconfigured DecoratorPoseConfiguration");

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

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

        return FunctorT::template operator()<T>(processed_r, processed_t, additionalParams...);
    }

};

/*!
 * \brief The InterpolatedPose class represent a decorator for the functor type FunctorT, interpolating between two poses.
 *
 * The FunctorT type is assumed to have an operator() const taking as first argument
 * "const T* const r, const T* const t" the orientation and position parameters of a pose,
 * and then additional arguments.
 * InterpolatedPose<FunctorT> then has an operator() const taking as first argument
 * "const T* const r1, const T* const t1, const T* const r2, const T* const t2",
 * and then the same additional arguments. The pose passed to the underlying functor is
 * interpolated from r1,t1 and r2,t2 using wheights w1 and w2 passed to the constructor.
 * Interpolation is done on the SO(3) manifold.
 */
template<typename FunctorT>
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

    template <typename T, typename ... P>
    bool operator()(const T* const r1,
                    const T* const t1,
                    const T* const r2,
                    const T* const t2,
                    P ... additionalParams) const {

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

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

        for (int i = 0; i < 3; i++) {
            processed_r[i] = interpolated.r[i];
            processed_t[i] = interpolated.t[i];
        }

        return FunctorT::template operator()<T>(processed_r, processed_t, additionalParams...);

    }

protected:

    double _w1;
    double _w2;

};

/*!
 * \brief The InvertPose class represent a decorator for the functor type FunctorT, inverting the input pose.
 *
 * The FunctorT type is assumed to have an operator() const taking as first argument
 * "const T* const r, const T* const t" the orientation and position parameters of a pose,
 * and then additional arguments.
 * InterpolatedPose<FunctorT> then has an operator() const takingthe same arguments
 * as FunctorT. The pose passed to the underlying functor is
 * the inverted posed passed as input to InterpolatedPose<FunctorT>.
 * So if you have a functor taking as input world2body but your parameters are
 * expressed as body2world, then this decorator can perform the conversion on the fly.
 */
template<typename FunctorT>
class InvertPose : private FunctorT
{
public:

    template <typename ... P>
    InvertPose(P... args) :
        FunctorT(args...)
    {

    }

    template <typename T, typename ... P>
    bool operator()(const T* const r,
                    const T* const t,
                    P ... additionalParams) const {

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        V3T rot;
        rot << r[0], r[1], r[2];
        V3T tran;
        tran << t[0], t[1], t[2];
        StereoVision::Geometry::RigidBodyTransform<T> pose(rot,tran);

        StereoVision::Geometry::RigidBodyTransform<T> inverted = pose.inverse();

        T processed_r[3];
        T processed_t[3];

        for (int i = 0; i < 3; i++) {
            processed_r[i] = inverted.r[i];
            processed_t[i] = inverted.t[i];
        }

        return FunctorT::template operator()<T>(processed_r, processed_t, additionalParams...);

    }

};

} // namespace StereoVisionApp

#endif // POSEDECORATORFUNCTORS_H
