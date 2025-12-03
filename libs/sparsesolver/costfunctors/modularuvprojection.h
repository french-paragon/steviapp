#ifndef STEREOVISIONAPP_MODULARUVPROJECTION_H
#define STEREOVISIONAPP_MODULARUVPROJECTION_H

#include <ceres/jet.h>
#include <Eigen/Core>
#include <Eigen/Dense>

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

#define INSTANCE_ModularUVProjection_dirFromUV_FOR_TYPE(T) \
    virtual Eigen::Matrix<T,3,1> dirFromUV(T const* uv, T const* const* parameters) const = 0;

    INSTANCE_ModularUVProjection_dirFromUV_FOR_TYPE(float);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_TYPE(double);

#define INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(N) \
    virtual Eigen::Matrix<ceres::Jet<float,N>,3,1> dirFromUV(ceres::Jet<float,N> const* uv, ceres::Jet<float,N> const* const* parameters) const = 0; \
    virtual Eigen::Matrix<ceres::Jet<double,N>,3,1> dirFromUV(ceres::Jet<double,N> const* uv, ceres::Jet<double,N> const* const* parameters) const = 0

    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(1);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(2);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(3);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(4);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(5);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(6);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(7);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(8);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(9);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(10);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(11);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(12);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(13);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(14);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(15);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(16);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(17);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(18);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(19);
    INSTANCE_ModularUVProjection_dirFromUV_FOR_JET(20);

    virtual int nParameters() const = 0;
};

/*!
 * \brief The AnyUVProjection class is a templated ModularUVProjection that allow to use any projection function in a generic fashion
 */
template<typename UVProjFunctor, int nParams>
class AnyUVProjection : public ModularUVProjection {
public:

    AnyUVProjection(UVProjFunctor* functor, bool manage = true) :
        _functor(functor),
        _manage(manage)
    {

    }

    virtual ~AnyUVProjection() {
        if (_manage and _functor != nullptr) {
            delete _functor;
        }
    }

#define INSTANCE_ImplUVProjection_dirFromUV_FOR_TYPE(T) \
    virtual Eigen::Matrix<T,3,1> dirFromUV(T const* uv, T const* const* parameters) const override { \
        return _functor->dirFromUV(uv, parameters); \
    }

    INSTANCE_ImplUVProjection_dirFromUV_FOR_TYPE(float);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_TYPE(double);

#define INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(N) \
    virtual Eigen::Matrix<ceres::Jet<float,N>,3,1> dirFromUV(ceres::Jet<float,N> const* uv, ceres::Jet<float,N> const* const* parameters) const override { \
        return _functor->dirFromUV(uv, parameters); \
    } \
    virtual Eigen::Matrix<ceres::Jet<double,N>,3,1> dirFromUV(ceres::Jet<double,N> const* uv, ceres::Jet<double,N> const* const* parameters) const override { \
            return _functor->dirFromUV(uv, parameters); \
    }

    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(1);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(2);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(3);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(4);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(5);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(6);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(7);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(8);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(9);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(10);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(11);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(12);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(13);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(14);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(15);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(16);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(17);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(18);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(19);
    INSTANCE_ImplUVProjection_dirFromUV_FOR_JET(20);

    virtual int nParameters() const override {
        return nParams;
    }

protected:
    bool _manage;
    UVProjFunctor* _functor;
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

template<typename UVProjector>
class UV2XYZCostDynamic
{
public:

    static constexpr int nResiduals = 2;

    static constexpr int poseRParamId = 0;
    static constexpr int poseTParamId = 1;
    static constexpr int AdditionalParamsStartsId = 2;

    UV2XYZCostDynamic(UVProjector* projector,
                      Eigen::Vector3d const& xyz,
                      Eigen::Vector2d const& uv,
                      Eigen::Matrix2d const& info,
                      int nProjParams,
                      bool manageProjector = true) :
        _manageProjector(manageProjector),
        _projector(projector),
        _xyz(xyz),
        _uv(uv),
        _info(info),
        _nProjParams(nProjParams)
    {

    }

    ~UV2XYZCostDynamic() {
        if (_manageProjector) {
            delete _projector;
        }
    }

    template <typename T, typename ... P>
    bool operator()(T const* const* parameters, T* residuals) const {

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        //projection

        V3T lm_pos = _xyz.cast<T>();

        StereoVision::Geometry::RigidBodyTransform<T> world2sensor;

        world2sensor.r << parameters[poseRParamId][0], parameters[poseRParamId][1], parameters[poseRParamId][2];
        world2sensor.t << parameters[poseTParamId][0], parameters[poseTParamId][1], parameters[poseTParamId][2];

        V3T Pbar = world2sensor*lm_pos;
        if (Pbar[2] < 0.0) {
            return false;
        }

        V2T proj = StereoVision::Geometry::projectPoints(Pbar);

        std::array<T,2> uv = {T(_uv[0]), T(_uv[1])};

        std::vector<const T*> projParams(_nProjParams);

        for (int i = 0; i < projParams.size(); i++) {
            projParams[i] = parameters[AdditionalParamsStartsId+i];
        }

        V3T direction = _projector->dirFromUV(uv.data(), projParams.data());

        proj *= direction.z();

        V2T error;
        error << proj[0] - direction[0], proj[1] - direction[1];

        residuals[0] = _info(0,0)*error[0] + _info(0,1)*error[1];
        residuals[1] = _info(1,0)*error[0] + _info(1,1)*error[1];

#ifndef NDEBUG
        if (!ceres::isfinite(residuals[0]) or !ceres::isfinite(residuals[1])) {
            std::cout << "Error in UV2XYZCostDynamic cost computation" << std::endl;
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

    int _nProjParams;
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

template<typename UVProjector>
class UV2ParametrizedXYZCostDynamic
{
public:

    static constexpr int nResiduals = 2;

    static constexpr int poseRParamId = 0;
    static constexpr int poseTParamId = 1;
    static constexpr int lmPosParamId = 2;
    static constexpr int AdditionalParamsStartsId = 3;

    UV2ParametrizedXYZCostDynamic(UVProjector* projector,
                                  Eigen::Vector2d const& uv,
                                  Eigen::Matrix2d const& info,
                                  int nProjParams,
                                  bool manageProjector = true) :
        _manageProjector(manageProjector),
        _projector(projector),
        _uv(uv),
        _info(info),
        _nProjParams(nProjParams)
    {

    }

    ~UV2ParametrizedXYZCostDynamic() {
        if (_manageProjector) {
            delete _projector;
        }
    }

    template <typename T, typename ... P>
    bool operator()(T const* const* parameters, T* residuals) const {

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        //projection

        V3T lm_pos;
        lm_pos << parameters[lmPosParamId][0], parameters[lmPosParamId][1], parameters[lmPosParamId][2];

        StereoVision::Geometry::RigidBodyTransform<T> world2sensor;

        world2sensor.r << parameters[poseRParamId][0], parameters[poseRParamId][1], parameters[poseRParamId][2];
        world2sensor.t << parameters[poseTParamId][0], parameters[poseTParamId][1], parameters[poseTParamId][2];

        V3T Pbar = world2sensor*lm_pos;
        if (Pbar[2] < 0.0) {
            return false;
        }

        V2T proj = StereoVision::Geometry::projectPoints(Pbar);

        std::array<T,2> uv = {T(_uv[0]), T(_uv[1])};

        std::vector<const T*> projParams(_nProjParams);

        for (int i = 0; i < projParams.size(); i++) {
            projParams[i] = parameters[AdditionalParamsStartsId+i];
        }

        V3T direction = _projector->dirFromUV(uv.data(), projParams.data());

        proj *= direction.z();

        V2T error;
        error << proj[0] - direction[0], proj[1] - direction[1];

        residuals[0] = _info(0,0)*error[0] + _info(0,1)*error[1];
        residuals[1] = _info(1,0)*error[0] + _info(1,1)*error[1];

#ifndef NDEBUG
        if (!ceres::isfinite(residuals[0]) or !ceres::isfinite(residuals[1])) {
            std::cout << "Error in UV2ParametrizedXYZCostDynamic cost computation" << std::endl;
        }
#endif

        return true;

    }

protected:

    bool _manageProjector;
    UVProjector* _projector;

    Eigen::Vector2d _uv;
    Eigen::Matrix2d _info;

    int _nProjParams;
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

template<typename UVProjector1, typename UVProjector2>
class UV2UVCost {

public:
    static constexpr int nResiduals = 4;

    UV2UVCost(UVProjector1* projector1,
              Eigen::Vector2d const& uv1,
              Eigen::Matrix2d const& info1,
              std::vector<int> const& projector1ParamsMap,
              UVProjector2* projector2,
              Eigen::Vector2d const& uv2,
              Eigen::Matrix2d const& info2,
              std::vector<int> const& projector2ParamsMap,
              bool manageProjectors = true) :
        _manageProjectors(manageProjectors),
        _projector1(projector1),
        _uv1(uv1),
        _info1(info1),
        _projector1ParamsMap(projector1ParamsMap),
        _projector2(projector2),
        _uv2(uv2),
        _info2(info2),
        _projector2ParamsMap(projector2ParamsMap)
    {

    }

    ~UV2UVCost() {
        if (_manageProjectors) {
            delete _projector1;
            delete _projector2;
        }
    }

    template <typename T, typename ... P>
    bool operator()(T const* const* parameters, T* residuals) const {

        T const* r1 = parameters[0];
        T const* t1 = parameters[1];
        T const* r2 = parameters[2];
        T const* t2 = parameters[3];

        using V2T = Eigen::Vector<T,2>;
        using V3T = Eigen::Vector<T,3>;

        StereoVision::Geometry::RigidBodyTransform<T> world2sensor1;

        world2sensor1.r << r1[0], r1[1], r1[2];
        world2sensor1.t << t1[0], t1[1], t1[2];


        StereoVision::Geometry::RigidBodyTransform<T> sensor12world = world2sensor1.inverse();

        StereoVision::Geometry::RigidBodyTransform<T> world2sensor2;

        world2sensor2.r << r2[0], r2[1], r2[2];
        world2sensor2.t << t2[0], t2[1], t2[2];

        StereoVision::Geometry::RigidBodyTransform<T> sensor12sensor2 = world2sensor2*sensor12world;

        std::array<T,2> uv1 = {T(_uv1[0]), T(_uv1[1])};
        std::array<T,2> uv2 = {T(_uv2[0]), T(_uv2[1])};

        std::vector<const T*> projParams1(_projector1ParamsMap.size());

        for (size_t i = 0; i < _projector1ParamsMap.size(); i++) {
            projParams1[i] = parameters[_projector1ParamsMap[i]];
        }

        std::vector<const T*> projParams2(_projector2ParamsMap.size());

        for (size_t i = 0; i < _projector2ParamsMap.size(); i++) {
            projParams2[i] = parameters[_projector2ParamsMap[i]];
        }

        V3T direction1Img1 = _projector1->dirFromUV(uv1.data(), projParams1.data());
        V3T direction2Img2 = _projector2->dirFromUV(uv2.data(), projParams2.data());

        V3T direction1CamFrame = StereoVision::Geometry::angleAxisRotate(sensor12sensor2.r, direction1Img1);
        V3T direction2CamFrame = direction2Img2;

        //we have sensor12sensor2.t + x*direction1 = y*direction2, or [direction1 -direction2] * [x; y] = -sensor12sensor2.t

        Eigen::Matrix<T,3,2> MP;

        for (int i = 0; i < 3; i++) {
            MP(i,0) = direction1CamFrame[i];
            MP(i,1) = -direction2CamFrame[i];
        }

        V2T xy = MP.householderQr().solve(-sensor12sensor2.t);

        V3T inter1 = sensor12sensor2.t + xy.x()*direction1CamFrame;
        V3T inter2 = xy.y()*direction2CamFrame;

        V3T intersectionCam2 = (inter1 + inter2)/T(2);

        V3T intersectionCam1 = sensor12sensor2.inverse()*intersectionCam2;

        V2T projCam1 = intersectionCam1.template block<2,1>(0,0);
        projCam1 /= intersectionCam1.z();
        projCam1 *= direction2Img2.z();

        V2T projCam2 = intersectionCam2.template block<2,1>(0,0);
        projCam2 /= intersectionCam2.z();
        projCam2 *= direction1Img1.z();

        V2T error1;
        error1 << projCam1[0] - direction1Img1[0], projCam1[1] - direction1Img1[1];

        V2T error2;
        error2 << projCam2[0] - direction2Img2[0], projCam2[1] - direction2Img2[1];

        residuals[0] = _info1(0,0)*error1[0] + _info1(0,1)*error1[1];
        residuals[1] = _info1(1,0)*error1[0] + _info1(1,1)*error1[1];

        residuals[2] = _info2(0,0)*error2[0] + _info2(0,1)*error2[1];
        residuals[3] = _info2(1,0)*error2[0] + _info2(1,1)*error2[1];

        return true;

    }
protected:

    bool _manageProjectors;
    UVProjector1* _projector1;
    UVProjector2* _projector2;

    std::vector<int> _projector1ParamsMap;
    Eigen::Vector2d _uv1;
    Eigen::Matrix2d _info1;

    std::vector<int> _projector2ParamsMap;
    Eigen::Vector2d _uv2;
    Eigen::Matrix2d _info2;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_MODULARUVPROJECTION_H
