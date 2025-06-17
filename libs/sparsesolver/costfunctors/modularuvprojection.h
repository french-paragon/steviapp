#ifndef STEREOVISIONAPP_MODULARUVPROJECTION_H
#define STEREOVISIONAPP_MODULARUVPROJECTION_H

#include <ceres/jet.h>
#include <Eigen/Core>

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>
#include <StereoVision/geometry/alignement.h>

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
    virtual ~ModularUVProjection();

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
                    P ... additionalParams) const {

        static_assert(sizeof... (additionalParams) == nProjParameters, "wrong number of projection parameters provided");

        std::array<const T*,nProjParameters+1> additionalParamsArray = {additionalParams...};
        std::tuple<P...> additionalParamsTuple(additionalParams...);

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        //projection

        StereoVision::Geometry::RigidBodyTransform<T> world2sensor;

        world2sensor.r << r[0], r[1], r[2];
        world2sensor.t << t[0], t[1], t[2];

        V3T Pbar = world2sensor*_xyz.cast<T>();
        if (Pbar[2] < 0.0) {
            return false;
        }

        V2T proj = StereoVision::Geometry::projectPoints(Pbar);

        std::array<T,2> uv = {T(_uv[0]), T(_uv[1])};
        std::array<const T*,nProjParameters> projParams;

        for (int i = 0; i < nProjParameters; i++) {
            projParams[i] = additionalParamsArray[i];
        }


        V3T direction = _projector->dirFromUV(uv.data(), projParams.data());

        proj *= direction.z;

        V2T error;
        error << proj[0] - direction[0], proj[1] - direction[1];

        T* residual = std::get<nProjParameters>(additionalParamsTuple);

        residual[0] = _info(0,0)*error[0] + _info(0,1)*error[1];
        residual[1] = _info(1,0)*error[0] + _info(1,1)*error[1];

#ifndef NDEBUG
        if (!ceres::isfinite(residual[0]) or !ceres::isfinite(residual[1])) {
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
    bool operator()(const T* const r,
                    const T* const t,
                    const T* const lm,
                    P ... additionalParams) const {

        static_assert(sizeof... (additionalParams) == nProjParameters+1, "wrong number of projection parameters provided");

        std::array<const T*,nProjParameters+1> additionalParamsArray = {additionalParams...};
        std::tuple<P...> additionalParamsTuple(additionalParams...);

        using M3T = Eigen::Matrix<T,3,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        //projection

        V3T lm_pos;
        lm_pos << lm[0], lm[1], lm[2];

        StereoVision::Geometry::RigidBodyTransform<T> world2sensor;

        world2sensor.r << r[0], r[1], r[2];
        world2sensor.t << t[0], t[1], t[2];

        V3T Pbar = world2sensor*lm_pos;
        if (Pbar[2] < 0.0) {
            return false;
        }

        V2T proj = StereoVision::Geometry::projectPoints(Pbar);

        std::array<T,2> uv = {T(_uv[0]), T(_uv[1])};

        std::array<const T*,nProjParameters> projParams;

        for (int i = 0; i < nProjParameters; i++) {
            projParams[i] = additionalParamsArray[i];
        }

        V3T direction = _projector->dirFromUV(uv.data(), projParams.data());

        proj *= direction.z();

        V2T error;
        error << proj[0] - direction[0], proj[1] - direction[1];

        T* residual = std::get<nProjParameters>(additionalParamsTuple);

        residual[0] = _info(0,0)*error[0] + _info(0,1)*error[1];
        residual[1] = _info(1,0)*error[0] + _info(1,1)*error[1];

#ifndef NDEBUG
        if (!ceres::isfinite(residual[0]) or !ceres::isfinite(residual[1])) {
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
                    P ... additionalParams) const {

        static_assert(sizeof... (additionalParams) == nProjParameters+1, "wrong number of projection parameters provided");

        std::array<const T*,nProjParameters+1> additionalParamsArray = {additionalParams...};
        std::tuple<P...> additionalParamsTuple(additionalParams...);

        using M3T = Eigen::Matrix<T,3,3>;
        using M23T = Eigen::Matrix<T,2,3>;

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        //projection

        StereoVision::Geometry::RigidBodyTransform<T> mapping2sensor;

        mapping2sensor.r << r[0], r[1], r[2];
        mapping2sensor.t << t[0], t[1], t[2];

        StereoVision::Geometry::RigidBodyTransform<T> sensor2mapping =
                mapping2sensor.inverse();

        V2T xy = _xy.cast<T>();
        M23T M = _M.cast<T>();

        std::array<T,2> uv = {T(_uv[0]), T(_uv[1])};

        std::array<const T*,nProjParameters> projParams;

        for (int i = 0; i < nProjParameters; i++) {
            projParams[i] = additionalParamsArray[i];
        }

        V3T direction = _projector->dirFromUV(uv.data(), projParams.data());
        V3T dirMapping = StereoVision::Geometry::angleAxisRotate(sensor2mapping.r,direction);

        V2T Md = M*dirMapping;
        T MdTMdm1 = T(1)/Md.dot(Md);
        T alpha = MdTMdm1*Md.dot(xy - M*sensor2mapping.t);

        V2T error = M*(sensor2mapping.t + alpha*dirMapping) - xy;

        T* residual = std::get<nProjParameters>(additionalParamsTuple);

        residual[0] = _info(0,0)*error[0] + _info(0,1)*error[1];
        residual[1] = _info(1,0)*error[0] + _info(1,1)*error[1];

#ifndef NDEBUG
        if (!ceres::isfinite(residual[0]) or !ceres::isfinite(residual[1])) {
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


} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_MODULARUVPROJECTION_H
