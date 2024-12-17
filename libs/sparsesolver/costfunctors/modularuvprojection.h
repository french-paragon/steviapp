#ifndef STEREOVISIONAPP_MODULARUVPROJECTION_H
#define STEREOVISIONAPP_MODULARUVPROJECTION_H

#include <ceres/jet.h>
#include <Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>
#include <StereoVision/geometry/alignement.h>

#include <type_traits>

namespace StereoVisionApp {

/*!
 * \brief The ModularUVProjection class is meant to manage a generic UV projector and used to build generic projection cost functions
 *
 * This class is used to build generic functor for a ceres solver using DynamicAutoDiffCostFunction
 *
 * Please keep in mind that AutoDiffCostFunction is more efficient than DynamicAutoDiffCostFunction, as AutoDiffCostFunction can call the cost function only once.
 * So whenever you can, you should use more specialized classes compatible with AutoDiffCostFunction.
 *
 * This class implement a collection of different dirFromUV that can be used with different floating point and ceres Jet types.
 * The returned dir vector is representing a direction in sensor frame, with the scale corresponding to the information matrix for a cost function.
 * So when a 3D point is projected in homogeneous coordinates, it has to be multiplied by dirFromUV.z before computing the error in x and y.
 *
 * This allows to build functor with generic UV projectors... but it also  limit the dimensionnality of the Jets used (max is set in the class constexpr MaxJetDim).
 *
 */
class ModularUVProjection
{
public:
    // maximal number of derivatives that can be computed at once fro DynamicAutoDiffCostFunction
    static constexpr int MaxJetDim = 10;

    ModularUVProjection();

    virtual Eigen::Matrix<float,3,1> dirFromUV(float const* uv, float const* const* parameters) const = 0;
    virtual Eigen::Matrix<double,3,1> dirFromUV(double const* uv, double const* const* parameters) const = 0;

    virtual Eigen::Matrix<ceres::Jet<float,1>,3,1> dirFromUV(ceres::Jet<float,1> const* uv, ceres::Jet<float,1> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<float,2>,3,1> dirFromUV(ceres::Jet<float,2> const* uv, ceres::Jet<float,2> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<float,3>,3,1> dirFromUV(ceres::Jet<float,3> const* uv, ceres::Jet<float,3> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<float,4>,3,1> dirFromUV(ceres::Jet<float,4> const* uv, ceres::Jet<float,4> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<float,5>,3,1> dirFromUV(ceres::Jet<float,5> const* uv, ceres::Jet<float,5> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<float,6>,3,1> dirFromUV(ceres::Jet<float,6> const* uv, ceres::Jet<float,6> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<float,7>,3,1> dirFromUV(ceres::Jet<float,7> const* uv, ceres::Jet<float,7> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<float,8>,3,1> dirFromUV(ceres::Jet<float,8> const* uv, ceres::Jet<float,8> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<float,9>,3,1> dirFromUV(ceres::Jet<float,9> const* uv, ceres::Jet<float,9> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<float,10>,3,1> dirFromUV(ceres::Jet<float,10> const* uv, ceres::Jet<float,10> const* const* parameters) const = 0;

    virtual Eigen::Matrix<ceres::Jet<double,1>,3,1> dirFromUV(ceres::Jet<double,1> const* uv, ceres::Jet<double,1> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<double,2>,3,1> dirFromUV(ceres::Jet<double,2> const* uv, ceres::Jet<double,2> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<double,3>,3,1> dirFromUV(ceres::Jet<double,3> const* uv, ceres::Jet<double,3> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<double,4>,3,1> dirFromUV(ceres::Jet<double,4> const* uv, ceres::Jet<double,4> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<double,5>,3,1> dirFromUV(ceres::Jet<double,5> const* uv, ceres::Jet<double,5> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<double,6>,3,1> dirFromUV(ceres::Jet<double,6> const* uv, ceres::Jet<double,6> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<double,7>,3,1> dirFromUV(ceres::Jet<double,7> const* uv, ceres::Jet<double,7> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<double,8>,3,1> dirFromUV(ceres::Jet<double,8> const* uv, ceres::Jet<double,8> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<double,9>,3,1> dirFromUV(ceres::Jet<double,9> const* uv, ceres::Jet<double,9> const* const* parameters) const = 0;
    virtual Eigen::Matrix<ceres::Jet<double,10>,3,1> dirFromUV(ceres::Jet<double,10> const* uv, ceres::Jet<double,10> const* const* parameters) const = 0;

    virtual int nParameters() const = 0;
};

template<typename UVProjFunctor, int nParams>
class ImplUVProjection : public ModularUVProjection {
public:

    ImplUVProjection()
    {

    }

    virtual Eigen::Matrix<float,3,1> dirFromUV(float const* uv, float const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<double,3,1> dirFromUV(double const* uv, double const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }

    virtual Eigen::Matrix<ceres::Jet<float,1>,3,1> dirFromUV(ceres::Jet<float,1> const* uv, ceres::Jet<float,1> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<float,2>,3,1> dirFromUV(ceres::Jet<float,2> const* uv, ceres::Jet<float,2> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<float,3>,3,1> dirFromUV(ceres::Jet<float,3> const* uv, ceres::Jet<float,3> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<float,4>,3,1> dirFromUV(ceres::Jet<float,4> const* uv, ceres::Jet<float,4> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<float,5>,3,1> dirFromUV(ceres::Jet<float,5> const* uv, ceres::Jet<float,5> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<float,6>,3,1> dirFromUV(ceres::Jet<float,6> const* uv, ceres::Jet<float,6> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<float,7>,3,1> dirFromUV(ceres::Jet<float,7> const* uv, ceres::Jet<float,7> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<float,8>,3,1> dirFromUV(ceres::Jet<float,8> const* uv, ceres::Jet<float,8> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<float,9>,3,1> dirFromUV(ceres::Jet<float,9> const* uv, ceres::Jet<float,9> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<float,10>,3,1> dirFromUV(ceres::Jet<float,10> const* uv, ceres::Jet<float,10> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }

    virtual Eigen::Matrix<ceres::Jet<double,1>,3,1> dirFromUV(ceres::Jet<double,1> const* uv, ceres::Jet<double,1> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<double,2>,3,1> dirFromUV(ceres::Jet<double,2> const* uv, ceres::Jet<double,2> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<double,3>,3,1> dirFromUV(ceres::Jet<double,3> const* uv, ceres::Jet<double,3> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<double,4>,3,1> dirFromUV(ceres::Jet<double,4> const* uv, ceres::Jet<double,4> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<double,5>,3,1> dirFromUV(ceres::Jet<double,5> const* uv, ceres::Jet<double,5> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<double,6>,3,1> dirFromUV(ceres::Jet<double,6> const* uv, ceres::Jet<double,6> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<double,7>,3,1> dirFromUV(ceres::Jet<double,7> const* uv, ceres::Jet<double,7> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<double,8>,3,1> dirFromUV(ceres::Jet<double,8> const* uv, ceres::Jet<double,8> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<double,9>,3,1> dirFromUV(ceres::Jet<double,9> const* uv, ceres::Jet<double,9> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }
    virtual Eigen::Matrix<ceres::Jet<double,10>,3,1> dirFromUV(ceres::Jet<double,10> const* uv, ceres::Jet<double,10> const* const* parameters) const override {
        return UVProjFunctor::dirFromUV(uv, parameters);
    }

    virtual int nParameters() const override {
        return nParams;
    }

protected:
};

template<typename UVProjector, int ... uvProjParamsSizes>
class UV2XYZCost
{
public:

    static constexpr int nProjParameters = sizeof... (uvProjParamsSizes);
    static constexpr int uvProjParametersSizes[] = {uvProjParamsSizes...};

    UV2XYZCost(UVProjector* projector,
               Eigen::Vector3d const& xyz,
               Eigen::Vector2d const& uv,
               Eigen::Matrix2d const& info,
               bool manageProjector = true) :
        _manageProjector(manageProjector),
        _projector(projector),
        _xyz(xyz),
        _uv(uv),
        _info(info)
    {

    }

    ~UV2XYZCost() {
        if (_manageProjector) {
            delete _projector;
        }
    }

    template <typename T, typename ... P>
    bool operator()(const T* const r,
                    const T* const t,
                    const T* const boresight,
                    const T* const leverArm,
                    const P* const ... projectorParams,
                    T* residual) const {

        static_assert(sizeof... (projectorParams) == nProjParameters, "wrong number of projection parameters provided");
        static_assert(std::integral_constant<bool, (... && std::is_same_v<T,P>)>::value, "Wrong type for projection parameters");

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        //projection

        StereoVision::Geometry::RigidBodyTransform<T> body2mapping;

        body2mapping.r << r[0], r[1], r[2];
        body2mapping.t << t[0], t[1], t[2];

        StereoVision::Geometry::RigidBodyTransform<T> body2sensor;
        body2sensor.r << boresight[0], boresight[1], boresight[2];
        body2sensor.t << leverArm[0], leverArm[1], leverArm[2];

        V3T Pbar = body2sensor*body2mapping.inverse()*_xyz.cast<T>();
        if (Pbar[2] < 0.0) {
            return false;
        }

        V2T proj = StereoVision::Geometry::projectPoints(Pbar);

        std::array<T,2> uv = {T(_uv[0]), T(_uv[1])};
        std::array<T,nProjParameters> projParams = {projectorParams...};

        V3T direction = _projector->dirFromUV(uv.data(), projParams.data());

        proj *= direction.z;

        V2T error;
        error << proj[0] - direction[0], proj[1] - direction[1];

        residual[0] = _info(0,0)*error[0] + _info(0,1)*error[1];
        residual[1] = _info(1,0)*error[0] + _info(1,1)*error[1];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1])) {
            std::cout << "Error in UV2XYZCost cost computation" << std::endl;
        }
#endif

        return true;

    }

protected:

    bool _manageProjector;
    UVProjector* _projector;

    Eigen::Vector3d _xyz;
    Eigen::Vector2d _uv;
    Eigen::Matrix2d _info;
};


template<typename UVProjector, int ... uvProjParamsSizes>
class UV2XYZCostInterpPose
{
public:

    static constexpr int nProjParameters = sizeof... (uvProjParamsSizes);
    static constexpr int uvProjParametersSizes[] = {uvProjParamsSizes...};

    UV2XYZCostInterpPose(UVProjector* projector,
                         double w1,
                         double w2,
                         Eigen::Vector3d const& xyz,
                         Eigen::Vector2d const& uv,
                         Eigen::Matrix2d const& info,
                         bool manageProjector = true) :
        _manageProjector(manageProjector),
        _projector(projector),
        _xyz(xyz),
        _uv(uv),
        _info(info),
        _w1(w1),
        _w2(w2)
    {

    }

    ~UV2XYZCostInterpPose() {
        if (_manageProjector) {
            delete _projector;
        }
    }

    template <typename T, typename ... P>
    bool operator()(const T* const r1,
                    const T* const t1,
                    const T* const r2,
                    const T* const t2,
                    const T* const boresight,
                    const T* const leverArm,
                    const P* const ... projectorParams,
                    T* residual) const {

        static_assert(sizeof... (projectorParams) == nProjParameters, "wrong number of projection parameters provided");
        static_assert(std::integral_constant<bool, (... && std::is_same_v<T,P>)>::value, "Wrong type for projection parameters");

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        //projection

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

        StereoVision::Geometry::RigidBodyTransform<T> body2mapping =
                StereoVision::Geometry::interpolateRigidBodyTransformOnManifold(T(_w1), transform1, T(_w2), transform2);


        StereoVision::Geometry::RigidBodyTransform<T> body2sensor;
        body2sensor.r << boresight[0], boresight[1], boresight[2];
        body2sensor.t << leverArm[0], leverArm[1], leverArm[2];

        V3T Pbar = body2sensor*body2mapping.inverse()*_xyz.cast<T>();
        if (Pbar[2] < 0.0) {
            return false;
        }

        V2T proj = StereoVision::Geometry::projectPoints(Pbar);

        std::array<T,2> uv = {T(_uv[0]), T(_uv[1])};
        std::array<T,nProjParameters> projParams = {projectorParams...};

        V3T direction = _projector->dirFromUV(uv.data(), projParams.data());

        proj *= direction.z;

        V2T error;
        error << proj[0] - direction[0], proj[1] - direction[1];

        residual[0] = _info(0,0)*error[0] + _info(0,1)*error[1];
        residual[1] = _info(1,0)*error[0] + _info(1,1)*error[1];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1])) {
            std::cout << "Error in UV2XYZCostInterpPose cost computation" << std::endl;
        }
#endif

        return true;

    }

protected:

    bool _manageProjector;
    UVProjector* _projector;

    double _w1;
    double _w2;

    Eigen::Vector3d _xyz;
    Eigen::Vector2d _uv;
    Eigen::Matrix2d _info;
};


template<typename UVProjector, int ... uvProjParamsSizes>
class UV2ParametrizedXYZCost
{
public:

    static constexpr int nProjParameters = sizeof... (uvProjParamsSizes);
    static constexpr int uvProjParametersSizes[] = {uvProjParamsSizes...};

    UV2ParametrizedXYZCost(UVProjector* projector,
               Eigen::Vector2d const& uv,
               Eigen::Matrix2d const& info,
               bool manageProjector = true) :
        _manageProjector(manageProjector),
        _projector(projector),
        _uv(uv),
        _info(info)
    {

    }

    ~UV2ParametrizedXYZCost() {
        if (_manageProjector) {
            delete _projector;
        }
    }

    template <typename T, typename ... P>
    bool operator()(const T* const lm,
                    const T* const r,
                    const T* const t,
                    const T* const boresight,
                    const T* const leverArm,
                    const P* const ... projectorParams,
                    T* residual) const {

        static_assert(sizeof... (projectorParams) == nProjParameters, "wrong number of projection parameters provided");
        static_assert(std::integral_constant<bool, (... && std::is_same_v<T,P>)>::value, "Wrong type for projection parameters");

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        //projection

        V3T lm_pos;
        lm_pos << lm[0], lm[1], lm[2];

        StereoVision::Geometry::RigidBodyTransform<T> body2mapping;

        body2mapping.r << r[0], r[1], r[2];
        body2mapping.t << t[0], t[1], t[2];

        StereoVision::Geometry::RigidBodyTransform<T> body2sensor;
        body2sensor.r << boresight[0], boresight[1], boresight[2];
        body2sensor.t << leverArm[0], leverArm[1], leverArm[2];

        V3T Pbar = body2sensor*body2mapping.inverse()*lm_pos;
        if (Pbar[2] < 0.0) {
            return false;
        }

        V2T proj = StereoVision::Geometry::projectPoints(Pbar);

        std::array<T,2> uv = {T(_uv[0]), T(_uv[1])};
        std::array<T,nProjParameters> projParams = {projectorParams...};

        V3T direction = _projector->dirFromUV(uv.data(), projParams.data());

        proj *= direction.z;

        V2T error;
        error << proj[0] - direction[0], proj[1] - direction[1];

        residual[0] = _info(0,0)*error[0] + _info(0,1)*error[1];
        residual[1] = _info(1,0)*error[0] + _info(1,1)*error[1];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1])) {
            std::cout << "Error in UV2ParametrizedXYZCost cost computation" << std::endl;
        }
#endif

        return true;

    }

protected:

    bool _manageProjector;
    UVProjector* _projector;

    Eigen::Vector2d _uv;
    Eigen::Matrix2d _info;
};

template<typename UVProjector, int ... uvProjParamsSizes>
class UV2ParametrizedXYZCostInterpPose
{
public:

    static constexpr int nProjParameters = sizeof... (uvProjParamsSizes);
    static constexpr int uvProjParametersSizes[] = {uvProjParamsSizes...};

    UV2ParametrizedXYZCostInterpPose(UVProjector* projector,
                                     double w1,
                                     double w2,
                                     Eigen::Vector2d const& uv,
                                     Eigen::Matrix2d const& info,
                                     bool manageProjector = true) :
        _manageProjector(manageProjector),
        _projector(projector),
        _uv(uv),
        _info(info),
        _w1(w1),
        _w2(w2)
    {

    }

    ~UV2ParametrizedXYZCostInterpPose() {
        if (_manageProjector) {
            delete _projector;
        }
    }

private:
    template <typename T>
    struct identity { typedef T type; };
public:

    template <typename T, typename ... P>
    bool operator()(const T* const lm,
                    const typename identity<T>::type * const r1,
                    const typename identity<T>::type * const t1,
                    const typename identity<T>::type * const r2,
                    const typename identity<T>::type * const t2,
                    const typename identity<T>::type * const boresight,
                    const typename identity<T>::type * const leverArm,
                    P* ... params) const {

        static_assert(sizeof... (params) == nProjParameters+1, "wrong number of projection parameters provided");

        std::array<const T*,nProjParameters+1> p = {params...};
        std::array<const T*,nProjParameters> projParams;

        for (int i = 0; i < nProjParameters; i++) {
            projParams[i] = p[i];
        }

        T* residual = const_cast<T*>(p[nProjParameters]);

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        //projection

        V3T lm_pos;
        lm_pos << lm[0], lm[1], lm[2];

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

        StereoVision::Geometry::RigidBodyTransform<T> body2maping =
                StereoVision::Geometry::interpolateRigidBodyTransformOnManifold(T(_w1), transform1, T(_w2), transform2);

        StereoVision::Geometry::RigidBodyTransform<T> body2sensor;
        body2sensor.r << boresight[0], boresight[1], boresight[2];
        body2sensor.t << leverArm[0], leverArm[1], leverArm[2];

        V3T Pbar = body2sensor*body2maping.inverse()*lm_pos;
        if (Pbar[2] < 0.0) {
            return false;
        }

        V2T proj = StereoVision::Geometry::projectPoints(Pbar);

        std::array<T,2> uv = {T(_uv[0]), T(_uv[1])};
        V3T direction = _projector->dirFromUV(uv.data(), projParams.data());

        proj *= direction[2];

        V2T error;
        error << proj[0] - direction[0], proj[1] - direction[1];

        residual[0] = _info(0,0)*error[0] + _info(0,1)*error[1];
        residual[1] = _info(1,0)*error[0] + _info(1,1)*error[1];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1])) {
            std::cout << "Error in UV2ParametrizedXYZCostInterpPose cost computation" << std::endl;
        }
#endif

        return true;

    }

protected:

    bool _manageProjector;
    UVProjector* _projector;

    double _w1;
    double _w2;

    Eigen::Vector2d _uv;
    Eigen::Matrix2d _info;
};

/*!
 * \brief The UV2ProjectedXYCost class represent a constaint between a UV measurement in a projector and a projected XY coordinate in mapping frame.
 *
 * The constraint derive from the relation M(t_cam + alpha*p_cam) = xy_map, with t_cam the position of the sensor in mapping frame,
 * è_cam the view direction from UV in mapping frame, M a projection matrix and xy_map the expected projected point.
 */
template<typename UVProjector, int ... uvProjParamsSizes>
class UV2ProjectedXYCost {
public:

    static constexpr int nProjParameters = sizeof... (uvProjParamsSizes);
    static constexpr int uvProjParametersSizes[] = {uvProjParamsSizes...};

    UV2ProjectedXYCost(UVProjector* projector,
                       Eigen::Vector2d const& xy,
                       Eigen::Matrix<double,2,3> const& M,
                       Eigen::Vector2d const& uv,
                       Eigen::Matrix2d const& info,
                       bool manageProjector = true) :
        _manageProjector(manageProjector),
        _projector(projector),
        _xy(xy),
        _M(M),
        _uv(uv),
        _info(info)
    {

    }

    ~UV2ProjectedXYCost() {
        if (_manageProjector) {
            delete _projector;
        }
    }

    template <typename T, typename ... P>
    bool operator()(const T* const r,
                    const T* const t,
                    const T* const boresight,
                    const T* const leverArm,
                    const P* const ... projectorParams,
                    T* residual) const {

        static_assert(sizeof... (projectorParams) == nProjParameters, "wrong number of projection parameters provided");
        static_assert(std::integral_constant<bool, (... && std::is_same_v<T,P>)>::value, "Wrong type for projection parameters");

        using M3T = Eigen::Matrix<T,3,3>;
        using M23T = Eigen::Matrix<T,2,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        //projection

        StereoVision::Geometry::RigidBodyTransform<T> body2mapping;

        body2mapping.r << r[0], r[1], r[2];
        body2mapping.t << t[0], t[1], t[2];

        StereoVision::Geometry::RigidBodyTransform<T> body2sensor;
        body2sensor.r << boresight[0], boresight[1], boresight[2];
        body2sensor.t << leverArm[0], leverArm[1], leverArm[2];

        StereoVision::Geometry::RigidBodyTransform<T> sensor2mapping =
                body2mapping*body2sensor.inverse();

        V2T xy = _xy.cast<T>();
        M23T M = _M.cast<T>();

        std::array<T,2> uv = {T(_uv[0]), T(_uv[1])};
        std::array<T,nProjParameters> projParams = {projectorParams...};

        V3T direction = _projector->dirFromUV(uv.data(), projParams.data());
        V3T dirMapping = StereoVision::Geometry::angleAxisRotate(sensor2mapping.r,direction);

        V2T Md = M*dirMapping;
        T MdTMdm1 = T(1)/Md.dot(Md);
        T alpha = MdTMdm1*Md.dot(xy - M*sensor2mapping.t);

        V2T error = M*(sensor2mapping.t + alpha*dirMapping) - xy;

        residual[0] = _info(0,0)*error[0] + _info(0,1)*error[1];
        residual[1] = _info(1,0)*error[0] + _info(1,1)*error[1];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1])) {
            std::cout << "Error in UV2ProjectedXYCost cost computation" << std::endl;
        }
#endif

        return true;

    }

protected:

    bool _manageProjector;
    UVProjector* _projector;

    Eigen::Vector2d _xy;
    Eigen::Matrix<double,2,3> _M;
    Eigen::Vector2d _uv;
    Eigen::Matrix2d _info;
};

/*!
 * \brief The UV2ProjectedXYCostInterpPose class is the same cost as UV2ProjectedXYCost
 */
template<typename UVProjector, int ... uvProjParamsSizes>
class UV2ProjectedXYCostInterpPose {
public:

    static constexpr int nProjParameters = sizeof... (uvProjParamsSizes);
    static constexpr int uvProjParametersSizes[] = {uvProjParamsSizes...};

    UV2ProjectedXYCostInterpPose(UVProjector* projector,
                                 double w1,
                                 double w2,
                                 Eigen::Vector2d const& xy,
                                 Eigen::Matrix<double,2,3> const& M,
                                 Eigen::Vector2d const& uv,
                                 Eigen::Matrix2d const& info,
                                 bool manageProjector = true) :
        _manageProjector(manageProjector),
        _projector(projector),
        _w1(w1),
        _w2(w2),
        _xy(xy),
        _M(M),
        _uv(uv),
        _info(info)
    {

    }

    ~UV2ProjectedXYCostInterpPose() {
        if (_manageProjector) {
            delete _projector;
        }
    }

    template <typename T, typename ... P>
    bool operator()(const T* const r1,
                    const T* const t1,
                    const T* const r2,
                    const T* const t2,
                    const T* const boresight,
                    const T* const leverArm,
                    const P* const ... projectorParams,
                    T* residual) const {

        static_assert(sizeof... (projectorParams) == nProjParameters, "wrong number of projection parameters provided");
        static_assert(std::integral_constant<bool, (... && std::is_same_v<T,P>)>::value, "Wrong type for projection parameters");

        using M3T = Eigen::Matrix<T,3,3>;
        using M23T = Eigen::Matrix<T,2,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        //projection

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

        StereoVision::Geometry::RigidBodyTransform<T> body2mapping =
                StereoVision::Geometry::interpolateRigidBodyTransformOnManifold(T(_w1), transform1, T(_w2), transform2);


        StereoVision::Geometry::RigidBodyTransform<T> body2sensor;
        body2sensor.r << boresight[0], boresight[1], boresight[2];
        body2sensor.t << leverArm[0], leverArm[1], leverArm[2];

        StereoVision::Geometry::RigidBodyTransform<T> sensor2mapping =
                body2mapping*body2sensor.inverse();
        V2T xy = _xy.cast<T>();
        M23T M = _M.cast<T>();

        std::array<T,2> uv = {T(_uv[0]), T(_uv[1])};
        std::array<T,nProjParameters> projParams = {projectorParams...};

        V3T direction = _projector->dirFromUV(uv.data(), projParams.data());
        V3T dirMapping = StereoVision::Geometry::angleAxisRotate(sensor2mapping.r, direction);

        V2T Md = M*dirMapping;
        T MdTMdm1 = T(1)/Md.dot(Md);
        T alpha = MdTMdm1*Md.dot(xy - M*sensor2mapping.t);

        V2T error = M*(sensor2mapping.t + alpha*dirMapping) - xy;

        residual[0] = _info(0,0)*error[0] + _info(0,1)*error[1];
        residual[1] = _info(1,0)*error[0] + _info(1,1)*error[1];

#ifndef NDEBUG
        if (!ceres::IsFinite(residual[0]) or !ceres::IsFinite(residual[1])) {
            std::cout << "Error in UV2ProjectedXYCostInterpPose cost computation" << std::endl;
        }
#endif

        return true;

    }

protected:

    bool _manageProjector;
    UVProjector* _projector;

    double _w1;
    double _w2;

    Eigen::Vector2d _xy;
    Eigen::Matrix<double,2,3> _M;
    Eigen::Vector2d _uv;
    Eigen::Matrix2d _info;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_MODULARUVPROJECTION_H
