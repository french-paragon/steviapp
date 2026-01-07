#ifndef STEREOVISIONAPP_TRAJECTORYBASESBAMODULE_H
#define STEREOVISIONAPP_TRAJECTORYBASESBAMODULE_H

#include "../modularsbasolver.h"

#include "../../datablocks/trajectory.h"
#include "../../utils/statusoptionalreturn.h"
#include "../costfunctors/weightedcostfunction.h"
#include "../costfunctors/imustepcost.h"
#include "../costfunctors/gravityDecorators.h"
#include "../costfunctors/posedecoratorfunctors.h"
#include "../costfunctors/stochasticprocessaggregator.h"

namespace StereoVisionApp {

class TrajectoryBaseSBAModule : public ModularSBASolver::SBAModule
{

public:
    /*!
     * \brief insLeverArmPoseDefinition indicate how the mounting parameters for the ins lever arm are considered
     */
    static constexpr int insLeverArmPoseDefinition = Body2World | Sensor2Body;

    static const char* ModuleName;
    inline static void registerDefaultModuleFactory(SBASolverModulesInterface* interface) {
        interface->registerSBAModule(TrajectoryBaseSBAModule::ModuleName, [] (ModularSBASolver* solver) -> ModularSBASolver::SBAModule* {

            double gpsAccuracy = 0.02;
            double angularAccuracy = 0.1;
            double gyroAccuracy = 0.1;
            double accAccuracy = 0.5;
            double tiePointAccuracy = 0.5;

            double integrationtime = 0.5; //half a second

            StereoVisionApp::TrajectoryBaseSBAModule* trajectoryModule =
                    new StereoVisionApp::TrajectoryBaseSBAModule(integrationtime);

            trajectoryModule->setDefaultGpsAccuracy(gpsAccuracy);
            trajectoryModule->setDefaultOrientAccuracy(angularAccuracy);
            trajectoryModule->setDefaultGyroAccuracy(gyroAccuracy);
            trajectoryModule->setDefaultAccAccuracy(accAccuracy);

            return trajectoryModule;
        });
    }

    TrajectoryBaseSBAModule(double defaultIntegrationTime);

    virtual QString moduleName() const override;

    virtual bool addGraphReductorVariables(Project *currentProject, GenericSBAGraphReductor* graphReductor) override;
    virtual bool addGraphReductorObservations(Project *currentProject, GenericSBAGraphReductor* graphReductor) override;

    virtual bool setupParameters(ModularSBASolver* solver) override;
    virtual bool init(ModularSBASolver* solver, ceres::Problem & problem) override;
    virtual bool writeResults(ModularSBASolver* solver) override;
    virtual bool writeUncertainty(ModularSBASolver* solver) override;
    virtual void cleanup(ModularSBASolver* solver) override;

    inline double defaultGpsAccuracy() const
    {
        return _defaultGpsAccuracy;
    }

    inline void setDefaultGpsAccuracy(double newGpsAccuracy)
    {
        _defaultGpsAccuracy = newGpsAccuracy;
    }

    inline double defaultOrientAccuracy() const
    {
        return _defaultOrientAccuracy;
    }

    inline void setDefaultOrientAccuracy(double newOrientAccuracy)
    {
        _defaultOrientAccuracy = newOrientAccuracy;
    }

    inline double defaultAccAccuracy() const
    {
        return _defaultAccAccuracy;
    }

    inline void setDefaultAccAccuracy(double newAccAccuracy)
    {
        _defaultAccAccuracy = newAccAccuracy;
    }

    inline double defaultGyroAccuracy() const
    {
        return _defaultGyroAccuracy;
    }

    inline void setDefaultGyroAccuracy(double newGyroAccuracy)
    {
        _defaultGyroAccuracy = newGyroAccuracy;
    }

    inline bool hasOrientationPriorEnabled() const {
        return _useOrientationPriors;
    }

    inline void enableOrientationPrior(bool enabled) {
        _useOrientationPriors = enabled;
    }

protected:


    StatusOptionalReturn<void> addGpsObs(
        ModularSBASolver::TrajectoryNode* trajNode,
        Trajectory* traj,
        Trajectory::TimeCartesianSequence const& gpsSeq,
        int i,
        double t1,
        double t2,
        double gpsObs_t,
        double gpsAccuracy,
        int currentGPSNode,
        StereoVision::Geometry::AffineTransform<double> const& world2local,
        ceres::Problem & problem,
        ModularSBASolver* solver,
        bool addLogger);

    StatusOptionalReturn<void> addGyroObs(
        ModularSBASolver::TrajectoryNode* trajNode,
        Trajectory* traj,
        int trajIdx,
        int gyroId,
        int i,
        double time,
        Trajectory::TimeCartesianSequence const& gyroSeq,
        double gyroAccuracy,
        bool gyroBias,
        bool gyroScale,
        ceres::Problem & problem,
        ModularSBASolver* solver,
        bool addLogger);

    StatusOptionalReturn<void> addAccObs(
        ModularSBASolver::TrajectoryNode* trajNode,
        Trajectory* traj,
        int trajIdx,
        int gyroId,
        int accId,
        int i,
        double time,
        Trajectory::TimeCartesianSequence const& gyroSeq,
        Trajectory::TimeCartesianSequence const& imuSeq,
        double accAccuracy,
        bool gyroBias,
        bool gyroScale,
        bool accelerometerBias,
        bool accelerometerScale,
        ceres::Problem & problem,
        ModularSBASolver* solver,
        bool addLogger);

    template <size_t N>
    static inline std::array<double*,N+1> appendParamToArray(std::array<double*,N> const& initial, double* param) {
        std::array<double*,N+1> ret;
        for (int i = 0; i < N; i++) {
            ret[i] = initial[i];
        }
        ret[N] = param;
    }

    template <int flags>
    static constexpr int nAccCostParams() {
        return AccelerometerStepCostTraits<flags>::nAccCostParams();
    }

    template<bool WBias, bool WScale>
    static constexpr int nGyroCostParams() {

        int n = 2; //no bias or scale factors

        if (WBias) {
            n += 1;
        }

        if (WScale) {
            n += 1;
        }

        return n;
    }

    template <int flags>
    auto getParametersForAccCostFunc(ModularSBASolver::TrajectoryNode* trajNode,
                                     int accId,
                                     int gyroId,
                                     int i) {

        using Traits = AccelerometerStepCostTraits<flags>;
        constexpr int nParams = nAccCostParams<flags>();

        std::array<double*,nParams> params =
            {trajNode->nodes[i-2].t.data(),
             trajNode->nodes[i-1].rAxis.data(),
             trajNode->nodes[i-1].t.data(),
             trajNode->nodes[i].t.data()};

        int paramId = 0;
        params[paramId] = trajNode->nodes[i-2].t.data(); paramId++;
        params[paramId] = trajNode->nodes[i-1].rAxis.data(); paramId++;
        params[paramId] = trajNode->nodes[i-1].t.data(); paramId++;
        params[paramId] = trajNode->nodes[i].t.data(); paramId++;

        if (Traits::WGScale) {
            params[paramId] = _gyrosScales[gyroId].data(); paramId++;
        }

        if (Traits::WGBias) {
            params[paramId] = _gyrosBiases[gyroId].data(); paramId++;
        }

        if (Traits::WAScale) {
            params[paramId] = _accelerometersScales[accId].data(); paramId++;
        }

        if (Traits::WABias) {
            params[paramId] = _accelerometersBiases[accId].data(); paramId++;
        }

        params[paramId] = _gravity.data(); paramId++;

        assert(paramId == nParams);

        return params;

    }

    template<bool WBias, bool WScale>
    auto getParametersForGyroCostFunc(ModularSBASolver::TrajectoryNode* trajNode,
                                     int gyroId,
                                     int i) {

        constexpr int nParams = nGyroCostParams<WBias, WScale>();

        std::array<double*,nParams> params;

        int paramId = 0;
        params[paramId] = trajNode->nodes[i-1].rAxis.data(); paramId++;
        params[paramId] = trajNode->nodes[i].rAxis.data(); paramId++;

        if (WScale) {
            params[paramId] = _gyrosScales[gyroId].data(); paramId++;
        }

        if (WBias) {
            params[paramId] = _gyrosBiases[gyroId].data(); paramId++;
        }

        assert(paramId == nParams);

        return params;

    }

    static constexpr bool notDynamic = false; //the underlying functor is not dynamic
    static constexpr int insParamsDim = 3;

    template <template<class, int...> class CostT, class AccCost, int flags>
    using AccTemplateParametrizedCost =
        std::conditional_t<nAccCostParams<flags>() == 5,
            CostT<AccCost, 3,3,3,3,3,3>,
            std::conditional_t<nAccCostParams<flags>() == 6,
                CostT<AccCost, 3,3,3,3,3,3,3>,
                std::conditional_t<nAccCostParams<flags>() == 7,
                    CostT<AccCost, 3,3,3,3,3,3,3,3>,
                    std::conditional_t<nAccCostParams<flags>() == 8,
                        CostT<AccCost, 3,3,3,3,3,3,3,3,3>,
                        std::conditional_t<nAccCostParams<flags>() == 9,
                            CostT<AccCost, 3,3,3,3,3,3,3,3,3,3>,
                            void
                        >
                    >
                >
            >
        >;

    template <template<class, int...> class CostT, class AccCost, int flags>
    using LeverArmAccTemplateParametrizedCost =
        std::conditional_t<nAccCostParams<flags>() == 5,
            CostT<AccCost, 3,3,3,3,3,3,3,3,3,3>,
            std::conditional_t<nAccCostParams<flags>() == 6,
                CostT<AccCost, 3,3,3,3,3,3,3,3,3,3,3>,
                std::conditional_t<nAccCostParams<flags>() == 7,
                    CostT<AccCost, 3,3,3,3,3,3,3,3,3,3,3,3>,
                    std::conditional_t<nAccCostParams<flags>() == 8,
                        CostT<AccCost, 3,3,3,3,3,3,3,3,3,3,3,3,3>,
                        std::conditional_t<nAccCostParams<flags>() == 9,
                            CostT<AccCost, 3,3,3,3,3,3,3,3,3,3,3,3,3,3>,
                            void
                        >
                    >
                >
            >
        >;

    template <template<int...> class CostT, int flags>
    using AccParametrizedCost =
        std::conditional_t<nAccCostParams<flags>() == 5,
            CostT<3,3,3,3,3,3>,
            std::conditional_t<nAccCostParams<flags>() == 6,
                CostT<3,3,3,3,3,3,3>,
                std::conditional_t<nAccCostParams<flags>() == 7,
                    CostT<3,3,3,3,3,3,3,3>,
                    std::conditional_t<nAccCostParams<flags>() == 8,
                        CostT<3,3,3,3,3,3,3,3,3>,
                        std::conditional_t<nAccCostParams<flags>() == 9,
                            CostT<3,3,3,3,3,3,3,3,3,3>,
                            void
                        >
                    >
                >
            >
        >;



    template <template<int...> class CostT, int flags>
    using LeverArmAccParametrizedCost =
        std::conditional_t<nAccCostParams<flags>() == 5,
            CostT<3,3,3,3,3,3,3,3,3,3>,
            std::conditional_t<nAccCostParams<flags>() == 6,
                CostT<3,3,3,3,3,3,3,3,3,3,3>,
                std::conditional_t<nAccCostParams<flags>() == 7,
                    CostT<3,3,3,3,3,3,3,3,3,3,3,3>,
                    std::conditional_t<nAccCostParams<flags>() == 8,
                        CostT<3,3,3,3,3,3,3,3,3,3,3,3,3>,
                        std::conditional_t<nAccCostParams<flags>() == 9,
                            CostT<3,3,3,3,3,3,3,3,3,3,3,3,3,3>,
                            void
                        >
                    >
                >
            >
        >;

    template <int flags>
    struct AccelerometerStepCostGravity
    {

        static constexpr int refPosParamPos = AccelerometerStepCostTraits<flags>::refPosParamIdx();
        static constexpr int gravityParamPos = AccelerometerStepCostTraits<flags>::gravityParamIdx();

        template <typename T>
        using Decorator = GravityReoriented<T, refPosParamPos, gravityParamPos>;

        template <typename T>
        using StochasticProcessEnabledDecorator = StochasticProcessAggregator<Decorator<T>, notDynamic, insParamsDim, T::nParams>;
    };

    template <int flags>
    struct AccelerometerStepCostLeverArmGravity
    {

        static constexpr int refPosParamPos = AccelerometerStepCostTraits<flags>::refPosParamIdx();
        static constexpr int gravityParamPos = AccelerometerStepCostTraits<flags>::gravityParamIdx();

        template <typename T>
        using Decorator = ApplyLeverArm< //apply the lever arm to the t2 pose
            ApplyLeverArm< //apply the lever arm to the t1 pose
                ApplyLeverArm< //apply the lever arm to the t0 pose
                    AddPose< //add a pose for the lever arm
                        AddOrientation< //add orientation for the t0 pose
                            AddOrientation< //add orientation for the t2 pose
                                GravityReoriented<T, refPosParamPos, gravityParamPos>, 3
                            >, 0
                        >
                    ,0>
                ,0,2, insLeverArmPoseDefinition>
            ,0,4, insLeverArmPoseDefinition>
        ,0,6, insLeverArmPoseDefinition>;

        template <typename T>
        using StochasticProcessEnabledDecorator = StochasticProcessAggregator<Decorator<T>, notDynamic, insParamsDim, T::nParams+4>;
    };

    template <template<class, int...> class CostT, class GyroCost, bool WBias, bool WScale>
    using GyroTemplateParametrizedCost =
        std::conditional_t<nGyroCostParams<WBias, WScale>() == 2,
            CostT<GyroCost, 3,3,3>,
            std::conditional_t<nGyroCostParams<WBias, WScale>() == 3,
                CostT<GyroCost, 3,3,3,3>,
                std::conditional_t<nGyroCostParams<WBias, WScale>() == 4,
                    CostT<GyroCost, 3,3,3,3,3>,
                    void
                >
            >
        >;

    template <template<class, int...> class CostT, class GyroCost, bool WBias, bool WScale>
    using BoresightGyroTemplateParametrizedCost =
        std::conditional_t<nGyroCostParams<WBias, WScale>() == 2,
                           CostT<GyroCost, 3,3,3,3>,
                           std::conditional_t<nGyroCostParams<WBias, WScale>() == 3,
                                              CostT<GyroCost, 3,3,3,3,3>,
                                              std::conditional_t<nGyroCostParams<WBias, WScale>() == 4,
                                                                 CostT<GyroCost, 3,3,3,3,3,3>,
                                                                 void
                                                                 >
                                              >
                           >;


    template <template<int...> class CostT, bool WBias, bool WScale>
    using GyroParametrizedCost =
        std::conditional_t<nGyroCostParams<WBias, WScale>() == 2,
            CostT<3,3,3>,
            std::conditional_t<nGyroCostParams<WBias, WScale>() == 3,
                CostT<3,3,3,3>,
                std::conditional_t<nGyroCostParams<WBias, WScale>() == 4,
                    CostT<3,3,3,3,3>,
                    void
                >
            >
        >;


    template <template<int...> class CostT, bool WBias, bool WScale>
    using BoresightGyroParametrizedCost =
        std::conditional_t<nGyroCostParams<WBias, WScale>() == 2,
                           CostT<3,3,3,3>,
                           std::conditional_t<nGyroCostParams<WBias, WScale>() == 3,
                                              CostT<3,3,3,3,3>,
                                              std::conditional_t<nGyroCostParams<WBias, WScale>() == 4,
                                                                 CostT<3,3,3,3,3,3>,
                                                                 void
                                                                 >
                                              >
                           >;

    template <typename T>
    using BoresightGyroCostDecorator = ComposeBoresightRotation<
        ComposeBoresightRotation<
            AddOrientation<T,0>
            ,0,1, insLeverArmPoseDefinition>
        ,0,2, insLeverArmPoseDefinition>;


    template <typename D_T>
    using StochasticProcessBoresightGyroCostDecorator =
        StochasticProcessAggregator<BoresightGyroCostDecorator<D_T>, notDynamic, insParamsDim, D_T::nParams+1>;
    template <typename D_T>
    using StochasticProcessGyroCostDecorator =
        StochasticProcessAggregator<D_T, notDynamic, insParamsDim, D_T::nParams>;

    template<bool WBias, bool WScale>
    StatusOptionalReturn<void> addTempltGyroObs(
        ModularSBASolver::TrajectoryNode* trajNode,
        Trajectory* traj,
        int trajIdx,
        int gyroId,
        int i,
        double time,
        Trajectory::TimeCartesianSequence const& gyroSeq,
        double gyroAccuracy,
        ceres::Problem & problem,
        ModularSBASolver* solver,
        bool addLogger) {

        constexpr int nRes = 3;
        constexpr int paramBlockSize = 3;

        double dt = trajNode->nodes[i].time - trajNode->nodes[i-1].time;
        Eigen::Matrix<double,3,3> weigthMat = Eigen::Matrix<double,3,3>::Identity();

        double poseUncertainty = gyroAccuracy*dt;

        weigthMat(0,0) = 1/poseUncertainty;
        weigthMat(1,1) = 1/poseUncertainty;
        weigthMat(2,2) = 1/poseUncertainty;

        bool hasStochasticProcess = false;

        if (WBias) {
            if (!_gyrosBiasesStochasticProcesses[trajIdx].empty()) {
                hasStochasticProcess = true;
            }
        }

        if (WScale) {
            if (!_gyrosScalesStochasticProcesses[trajIdx].empty()) {
                hasStochasticProcess = true;
            }
        }

        int insMountingId = traj->insMountingId();

        ModularSBASolver::PoseNode* insMountingNode = solver->getNodeForMounting(insMountingId, false);

        if (insMountingNode != nullptr) {

            using CostF = BoresightGyroCostDecorator<GyroStepCost<WBias, WScale>>;
            using AutoDiffCostFuncT = BoresightGyroTemplateParametrizedCost<ceres::AutoDiffCostFunction,CostF, WBias, WScale>;
            using WeightedCostFuncT = BoresightGyroParametrizedCost<StereoVisionApp::WeightedSizedCostFunction, WBias, WScale>;

            static_assert(!std::is_same_v<AutoDiffCostFuncT, void>, "Unable to build autodiff cost function type");
            static_assert(!std::is_same_v<WeightedCostFuncT, void>, "Unable to build weighted cost function type");

            if (hasStochasticProcess) {
                constexpr int stride = 4;

                using DecoratedCostF = StochasticProcessBoresightGyroCostDecorator<GyroStepCost<WBias, WScale>>;
                using DecoratedAutoDiffCostFuncT = ceres::DynamicAutoDiffCostFunction<DecoratedCostF, stride>;
                using DecoratedWeigthedCostFunctT = StereoVisionApp::WeightedCostFunction<nRes>;


                DecoratedCostF* gyroStepCost =
                    GyroStepCostBase::getIntegratedIMUDiff<WBias, WScale, StochasticProcessBoresightGyroCostDecorator>(
                        gyroSeq,
                        trajNode->nodes[i-1].time,
                        trajNode->nodes[i].time);

                DecoratedAutoDiffCostFuncT* gyroStepCostFunction = new DecoratedAutoDiffCostFuncT(gyroStepCost);

                DecoratedWeigthedCostFunctT* weigthedGyroStepCost =
                    new DecoratedWeigthedCostFunctT(gyroStepCostFunction, weigthMat);


                auto paramsInitial = getParametersForGyroCostFunc<WBias, WScale>(trajNode,gyroId,i);

                std::vector<StochasticProcessParam> biasParams = collectParameterSet(time, _gyrosBiasesStochasticProcesses[trajIdx]);
                std::vector<StochasticProcessParam> scaleParams = collectParameterSet(time, _gyrosScalesStochasticProcesses[trajIdx]);

                std::vector<double*> params;
                params.reserve(paramsInitial.size() + biasParams.size() + scaleParams.size() + 1);

                params.push_back(insMountingNode->rAxis.data());

                for (double* p : paramsInitial) {
                    params.push_back(p);
                }

                for (StochasticProcessParam & p : biasParams) {
                    params.push_back(p.parameter);
                }

                for (StochasticProcessParam & p : scaleParams) {
                    params.push_back(p.parameter);
                }

                gyroStepCost->setNParams(params.size());

                std::vector<int> paramsMap;
                std::vector<double> weights;

                paramsMap.reserve(biasParams.size() + scaleParams.size());
                weights.reserve(biasParams.size() + scaleParams.size());

                constexpr int biasParamPos = GyroStepCost<WBias, WScale>::biasParamPos+1;
                constexpr int scaleParamPos = GyroStepCost<WBias, WScale>::scaleParamPos+1;


                for (StochasticProcessParam & p : biasParams) {
                    paramsMap.push_back(biasParamPos);
                    weights.push_back(p.weight);
                }

                for (StochasticProcessParam & p : scaleParams) {
                    paramsMap.push_back(scaleParamPos);
                    weights.push_back(p.weight);
                }

                gyroStepCost->setAccumulationWeights(paramsMap, weights);

                for (size_t i = 0; i < params.size(); i++) {
                    gyroStepCostFunction->AddParameterBlock(paramBlockSize);
                }
                gyroStepCostFunction->SetNumResiduals(nRes);

                weigthedGyroStepCost->syncParametersSizes();

                problem.AddResidualBlock(weigthedGyroStepCost, nullptr,
                                         params.data(),
                                         params.size());

            } else {

                CostF* gyroStepCost =
                    GyroStepCostBase::getIntegratedIMUDiff<WBias, WScale, BoresightGyroCostDecorator>(
                        gyroSeq,
                        trajNode->nodes[i-1].time,
                        trajNode->nodes[i].time);

                AutoDiffCostFuncT* gyroStepCostFunction =
                    new AutoDiffCostFuncT(gyroStepCost);

                WeightedCostFuncT* weigthedGyroStepCost =
                    new WeightedCostFuncT(gyroStepCostFunction, weigthMat);


                auto paramsInitial = getParametersForGyroCostFunc<WBias, WScale>(trajNode,gyroId,i);

                std::array<double*,paramsInitial.size()+1> params;

                params[0] = insMountingNode->rAxis.data();

                for (int i = 0; i < paramsInitial.size(); i++) {
                    params[i+1] = paramsInitial[i];
                }

                static_assert(params.size() == WeightedCostFuncT::nParamsBlocks,
                              "non compatible numbers of parameters in parameters array and cost function");

                static_assert(AutoDiffCostFuncT::ParameterDims::kNumParameterBlocks ==
                                  WeightedCostFuncT::nParamsBlocks,
                              "non compatible numbers of parameters in autodiff cost and weigthed cost function");


                problem.AddResidualBlock(weigthedGyroStepCost, nullptr,
                                         params.data(),
                                         params.size());

                if (addLogger) {
                    QString loggerName = QString("Gyro trajectory \"%1\" step time %2").arg(traj->objectName()).arg(time, 0, 'f', 2);
                    solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<nGyroCostParams<WBias, WScale>()+1,3>(gyroStepCostFunction, params));
                }

            }

        } else {

            using CostF = GyroStepCost<WBias, WScale>;
            using AutoDiffCostFuncT = GyroTemplateParametrizedCost<ceres::AutoDiffCostFunction,CostF, WBias, WScale>;
            using WeightedCostFuncT = GyroParametrizedCost<StereoVisionApp::WeightedSizedCostFunction, WBias, WScale>;

            static_assert(!std::is_same_v<AutoDiffCostFuncT, void>, "Unable to build autodiff cost function type");
            static_assert(!std::is_same_v<WeightedCostFuncT, void>, "Unable to build weighted cost function type");

            if (hasStochasticProcess) {
                constexpr int stride = 4;

                using DecoratedCostF = StochasticProcessGyroCostDecorator<GyroStepCost<WBias, WScale>>;
                using DecoratedAutoDiffCostFuncT = ceres::DynamicAutoDiffCostFunction<DecoratedCostF, stride>;
                using DecoratedWeigthedCostFunctT = StereoVisionApp::WeightedCostFunction<nRes>;

                DecoratedCostF* gyroStepCost =
                    GyroStepCostBase::getIntegratedIMUDiff<WBias, WScale, StochasticProcessGyroCostDecorator>(
                        gyroSeq,
                        trajNode->nodes[i-1].time,
                        trajNode->nodes[i].time);

                DecoratedAutoDiffCostFuncT* gyroStepCostFunction = new DecoratedAutoDiffCostFuncT(gyroStepCost);

                DecoratedWeigthedCostFunctT* weigthedGyroStepCost =
                    new DecoratedWeigthedCostFunctT(gyroStepCostFunction, weigthMat);


                auto paramsInitial = getParametersForGyroCostFunc<WBias, WScale>(trajNode,gyroId,i);

                std::vector<StochasticProcessParam> biasParams = collectParameterSet(time, _gyrosBiasesStochasticProcesses[trajIdx]);
                std::vector<StochasticProcessParam> scaleParams = collectParameterSet(time, _gyrosScalesStochasticProcesses[trajIdx]);

                std::vector<double*> params;
                params.reserve(paramsInitial.size() + biasParams.size() + scaleParams.size());

                for (double* p : paramsInitial) {
                    params.push_back(p);
                }

                for (StochasticProcessParam & p : biasParams) {
                    params.push_back(p.parameter);
                }

                for (StochasticProcessParam & p : scaleParams) {
                    params.push_back(p.parameter);
                }

                gyroStepCost->setNParams(params.size());

                std::vector<int> paramsMap;
                std::vector<double> weights;

                paramsMap.reserve(biasParams.size() + scaleParams.size());
                weights.reserve(biasParams.size() + scaleParams.size());

                constexpr int biasParamPos = GyroStepCost<WBias, WScale>::biasParamPos;
                constexpr int scaleParamPos = GyroStepCost<WBias, WScale>::scaleParamPos;

                for (StochasticProcessParam & p : biasParams) {
                    paramsMap.push_back(biasParamPos);
                    weights.push_back(p.weight);
                }

                for (StochasticProcessParam & p : scaleParams) {
                    paramsMap.push_back(scaleParamPos);
                    weights.push_back(p.weight);
                }

                gyroStepCost->setAccumulationWeights(paramsMap, weights);

                for (size_t i = 0; i < params.size(); i++) {
                    gyroStepCostFunction->AddParameterBlock(paramBlockSize);
                }
                gyroStepCostFunction->SetNumResiduals(nRes);

                weigthedGyroStepCost->syncParametersSizes();

                problem.AddResidualBlock(weigthedGyroStepCost, nullptr,
                                         params.data(),
                                         params.size());


            } else {

                CostF* gyroStepCost =
                    GyroStepCostBase::getIntegratedIMUDiff<WBias, WScale>(
                        gyroSeq,
                        trajNode->nodes[i-1].time,
                        trajNode->nodes[i].time);

                AutoDiffCostFuncT* gyroStepCostFunction =
                    new AutoDiffCostFuncT(gyroStepCost);

                WeightedCostFuncT* weigthedGyroStepCost =
                    new WeightedCostFuncT(gyroStepCostFunction, weigthMat);


                auto params = getParametersForGyroCostFunc<WBias, WScale>(trajNode,gyroId,i);

                static_assert(params.size() == WeightedCostFuncT::nParamsBlocks,
                              "non compatible numbers of parameters in parameters array and cost function");

                static_assert(AutoDiffCostFuncT::ParameterDims::kNumParameterBlocks ==
                                  WeightedCostFuncT::nParamsBlocks,
                              "non compatible numbers of parameters in autodiff cost and weigthed cost function");


                problem.AddResidualBlock(weigthedGyroStepCost, nullptr,
                                         params.data(),
                                         params.size());

                if (addLogger) {
                    QString loggerName = QString("Gyro trajectory \"%1\" step time %2").arg(traj->objectName()).arg(time, 0, 'f', 2);
                    solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<nGyroCostParams<WBias, WScale>(),3>(gyroStepCostFunction, params));
                }
            }
        }

        return StatusOptionalReturn<void>();
    }

    template <int flags>
    StatusOptionalReturn<void> addTempltAccObs(
        ModularSBASolver::TrajectoryNode* trajNode,
        Trajectory* traj,
        int trajIdx,
        int gyroId,
        int accId,
        int i,
        double time,
        Trajectory::TimeCartesianSequence const& gyroSeq,
        Trajectory::TimeCartesianSequence const& imuSeq,
        double accAccuracy,
        ceres::Problem & problem,
        ModularSBASolver* solver,
        bool addLogger) {

        using Traits = AccelerometerStepCostTraits<flags>;

        constexpr int nRes = 3;
        constexpr int paramBlockSize = 3;

        double dt = (trajNode->nodes[i].time - trajNode->nodes[i-2].time)/2;

        Eigen::Matrix<double,3,3> weigthMat = Eigen::Matrix<double,3,3>::Identity();

        double speedUncertainty = accAccuracy*dt;

        weigthMat(0,0) = 1/speedUncertainty;
        weigthMat(1,1) = 1/speedUncertainty;
        weigthMat(2,2) = 1/speedUncertainty;

        bool hasStochasticProcess = false;

        if (Traits::WABias) {
            if (!_accelerometersBiasesStochasticProcesses[trajIdx].empty()) {
                hasStochasticProcess = true;
            }
        }

        if (Traits::WAScale) {
            if (!_accelerometersScalesStochasticProcesses[trajIdx].empty()) {
                hasStochasticProcess = true;
            }
        }

        if (Traits::WGBias) {
            if (!_gyrosBiasesStochasticProcesses[trajIdx].empty()) {
                hasStochasticProcess = true;
            }
        }

        if (Traits::WGScale) {
            if (!_gyrosScalesStochasticProcesses[trajIdx].empty()) {
                hasStochasticProcess = true;
            }
        }

        int insMountingId = traj->insMountingId();

        ModularSBASolver::PoseNode* insMountingNode = solver->getNodeForMounting(insMountingId, false);

        if (insMountingNode != nullptr) {
            using AccCost = typename AccelerometerStepCostLeverArmGravity<flags>::template Decorator<AccelerometerStepCost<flags>>;
            using AutoDiffCostFuncT = LeverArmAccTemplateParametrizedCost<ceres::AutoDiffCostFunction,AccCost, flags>;
            using WeightedCostFuncT = LeverArmAccParametrizedCost<StereoVisionApp::WeightedSizedCostFunction, flags>;

            static_assert(!std::is_same_v<AutoDiffCostFuncT, void>, "Unable to build autodiff cost function type");
            static_assert(!std::is_same_v<WeightedCostFuncT, void>, "Unable to build weighted cost function type");

            if (hasStochasticProcess) {

                constexpr int stride = 4;

                using DecoratedAccCost = typename AccelerometerStepCostLeverArmGravity<flags>::template StochasticProcessEnabledDecorator<AccelerometerStepCost<flags>>;
                using DecoratedAutoDiffCostFuncT = ceres::DynamicAutoDiffCostFunction<DecoratedAccCost, stride>;
                using DecoratedWeightedCostFuncT = StereoVisionApp::WeightedCostFunction<nRes>;

                DecoratedAccCost* accStepCost =
                    AccelerometerStepCostBase::getIntegratedIMUDiff<flags, AccelerometerStepCostLeverArmGravity<flags>::template StochasticProcessEnabledDecorator>
                    (gyroSeq,
                     imuSeq,
                     trajNode->nodes[i-2].time,
                     trajNode->nodes[i-1].time,
                     trajNode->nodes[i].time,
                     _earth_center_pos);

                DecoratedAutoDiffCostFuncT* accStepCostFunction = new DecoratedAutoDiffCostFuncT(accStepCost);

                DecoratedWeightedCostFuncT* weigthedAccStepCost =
                    new DecoratedWeightedCostFuncT(accStepCostFunction, weigthMat);


                auto paramsInitial = getParametersForAccCostFunc<flags>(trajNode, accId, gyroId, i);

                std::vector<StochasticProcessParam> gyroBiasParams = collectParameterSet(time, _gyrosBiasesStochasticProcesses[trajIdx]);
                std::vector<StochasticProcessParam> gyroScaleParams = collectParameterSet(time, _gyrosScalesStochasticProcesses[trajIdx]);
                std::vector<StochasticProcessParam> accBiasParams = collectParameterSet(time, _accelerometersBiasesStochasticProcesses[trajIdx]);
                std::vector<StochasticProcessParam> accScaleParams = collectParameterSet(time, _accelerometersScalesStochasticProcesses[trajIdx]);

                std::vector<double*> params;
                params.reserve(paramsInitial.size() + gyroBiasParams.size() + gyroScaleParams.size() + accBiasParams.size() + accScaleParams.size() + 4);

                params[0] = insMountingNode->rAxis.data();
                params[1] = insMountingNode->t.data();
                params[2] = trajNode->nodes[i-2].rAxis.data();
                params[6] = trajNode->nodes[i].rAxis.data();

                //positions 0, 1, 2
                params.push_back(insMountingNode->rAxis.data());
                params.push_back(insMountingNode->t.data());
                params.push_back(trajNode->nodes[i-2].rAxis.data());

                //positions, 3, 4, 5
                for (int i = 0; i < 3; i++) {
                    params.push_back(paramsInitial[i]);
                }

                //postions 6
                params.push_back(trajNode->nodes[i].rAxis.data());

                for (int i = 3; i < paramsInitial.size(); i++) {
                    params.push_back(paramsInitial[i]);
                }

                for (StochasticProcessParam & p : gyroBiasParams) {
                    params.push_back(p.parameter);
                }

                for (StochasticProcessParam & p : gyroScaleParams) {
                    params.push_back(p.parameter);
                }

                for (StochasticProcessParam & p : accBiasParams) {
                    params.push_back(p.parameter);
                }

                for (StochasticProcessParam & p : accScaleParams) {
                    params.push_back(p.parameter);
                }

                accStepCost->setNParams(params.size());

                std::vector<int> paramsMap;
                std::vector<double> weights;

                paramsMap.reserve(gyroBiasParams.size() + gyroScaleParams.size() + accBiasParams.size() + accScaleParams.size());
                weights.reserve(gyroBiasParams.size() + gyroScaleParams.size() + accBiasParams.size() + accScaleParams.size());

                constexpr int gyroBiasParamPos = AccelerometerStepCost<flags>::gyroBiasPos+4;
                constexpr int gyroScaleParamPos = AccelerometerStepCost<flags>::gyroScalePos+4;
                constexpr int accBiasParamPos = AccelerometerStepCost<flags>::accBiasPos+4;
                constexpr int accScaleParamPos = AccelerometerStepCost<flags>::accScalePos+4;


                for (StochasticProcessParam & p : gyroBiasParams) {
                    paramsMap.push_back(gyroBiasParamPos);
                    weights.push_back(p.weight);
                }

                for (StochasticProcessParam & p : gyroScaleParams) {
                    paramsMap.push_back(gyroScaleParamPos);
                    weights.push_back(p.weight);
                }

                for (StochasticProcessParam & p : accBiasParams) {
                    paramsMap.push_back(accBiasParamPos);
                    weights.push_back(p.weight);
                }

                for (StochasticProcessParam & p : accScaleParams) {
                    paramsMap.push_back(accScaleParamPos);
                    weights.push_back(p.weight);
                }

                accStepCost->setAccumulationWeights(paramsMap, weights);

                for (size_t i = 0; i < params.size(); i++) {
                    accStepCostFunction->AddParameterBlock(paramBlockSize);
                }
                accStepCostFunction->SetNumResiduals(nRes);

                weigthedAccStepCost->syncParametersSizes();

                problem.AddResidualBlock(weigthedAccStepCost, nullptr,
                                         params.data(),
                                         params.size());

            } else {

                AccCost* accStepCost =
                    AccelerometerStepCostBase::getIntegratedIMUDiff<flags, AccelerometerStepCostLeverArmGravity<flags>::template Decorator>
                    (gyroSeq,
                     imuSeq,
                     trajNode->nodes[i-2].time,
                     trajNode->nodes[i-1].time,
                     trajNode->nodes[i].time,
                     _earth_center_pos);

                AutoDiffCostFuncT* accStepCostFunction =
                    new AutoDiffCostFuncT(accStepCost);

                WeightedCostFuncT* weigthedAccStepCost =
                    new WeightedCostFuncT(accStepCostFunction, weigthMat);

                auto paramsInitial = getParametersForAccCostFunc<flags>(trajNode, accId, gyroId, i);

                std::array<double*, paramsInitial.size()+4> params;

                params[0] = insMountingNode->rAxis.data();
                params[1] = insMountingNode->t.data();
                params[2] = trajNode->nodes[i-2].rAxis.data();
                params[6] = trajNode->nodes[i].rAxis.data();

                for (int i = 0; i < paramsInitial.size(); i++) {
                    int delta = 3;
                    if (i >= 3) {
                        delta = 4;
                    }
                    params[i+delta] = paramsInitial[i];
                }

                static_assert(params.size() == WeightedCostFuncT::nParamsBlocks,
                              "non compatible numbers of parameters in parameters array and cost function");

                static_assert(AutoDiffCostFuncT::ParameterDims::kNumParameterBlocks ==
                                  WeightedCostFuncT::nParamsBlocks,
                              "non compatible numbers of parameters in autodiff cost and weigthed cost function");

                problem.AddResidualBlock(weigthedAccStepCost, nullptr, params.data(), params.size());

                if (addLogger) {
                    QString loggerName = QString("Accelerometer trajectory \"%1\" step time %2").arg(traj->objectName()).arg(time, 0, 'f', 2);
                    solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<nAccCostParams<flags>()+4,3>(accStepCostFunction, params));
                }

            }
        } else {

            using AccCost = typename AccelerometerStepCostGravity<flags>::template Decorator<AccelerometerStepCost<flags>>;
            using AutoDiffCostFuncT = AccTemplateParametrizedCost<ceres::AutoDiffCostFunction,AccCost, flags>;
            using WeightedCostFuncT = AccParametrizedCost<StereoVisionApp::WeightedSizedCostFunction, flags>;

            static_assert(!std::is_same_v<AutoDiffCostFuncT, void>, "Unable to build autodiff cost function type");
            static_assert(!std::is_same_v<WeightedCostFuncT, void>, "Unable to build weighted cost function type");


            if (hasStochasticProcess) {

                constexpr int stride = 4;

                using DecoratedAccCost = typename AccelerometerStepCostGravity<flags>::template StochasticProcessEnabledDecorator<AccelerometerStepCost<flags>>;
                using DecoratedAutoDiffCostFuncT = ceres::DynamicAutoDiffCostFunction<DecoratedAccCost, stride>;
                using DecoratedWeightedCostFuncT = StereoVisionApp::WeightedCostFunction<nRes>;

                DecoratedAccCost* accStepCost =
                    AccelerometerStepCostBase::getIntegratedIMUDiff<flags, AccelerometerStepCostGravity<flags>::template StochasticProcessEnabledDecorator>
                    (gyroSeq,
                     imuSeq,
                     trajNode->nodes[i-2].time,
                     trajNode->nodes[i-1].time,
                     trajNode->nodes[i].time,
                     _earth_center_pos);

                DecoratedAutoDiffCostFuncT* accStepCostFunction = new DecoratedAutoDiffCostFuncT(accStepCost);

                DecoratedWeightedCostFuncT* weigthedAccStepCost =
                    new DecoratedWeightedCostFuncT(accStepCostFunction, weigthMat);


                auto paramsInitial = getParametersForAccCostFunc<flags>(trajNode, accId, gyroId, i);

                std::vector<StochasticProcessParam> gyroBiasParams = collectParameterSet(time, _gyrosBiasesStochasticProcesses[trajIdx]);
                std::vector<StochasticProcessParam> gyroScaleParams = collectParameterSet(time, _gyrosScalesStochasticProcesses[trajIdx]);
                std::vector<StochasticProcessParam> accBiasParams = collectParameterSet(time, _accelerometersBiasesStochasticProcesses[trajIdx]);
                std::vector<StochasticProcessParam> accScaleParams = collectParameterSet(time, _accelerometersScalesStochasticProcesses[trajIdx]);

                std::vector<double*> params;
                params.reserve(paramsInitial.size() + gyroBiasParams.size() + gyroScaleParams.size() + accBiasParams.size() + accScaleParams.size());

                params[0] = insMountingNode->rAxis.data();
                params[1] = insMountingNode->t.data();
                params[2] = trajNode->nodes[i-2].rAxis.data();
                params[6] = trajNode->nodes[i].rAxis.data();

                for (int i = 0; i < paramsInitial.size(); i++) {
                    params.push_back(paramsInitial[i]);
                }

                for (StochasticProcessParam & p : gyroBiasParams) {
                    params.push_back(p.parameter);
                }

                for (StochasticProcessParam & p : gyroScaleParams) {
                    params.push_back(p.parameter);
                }

                for (StochasticProcessParam & p : accBiasParams) {
                    params.push_back(p.parameter);
                }

                for (StochasticProcessParam & p : accScaleParams) {
                    params.push_back(p.parameter);
                }

                accStepCost->setNParams(params.size());

                std::vector<int> paramsMap;
                std::vector<double> weights;

                paramsMap.reserve(gyroBiasParams.size() + gyroScaleParams.size() + accBiasParams.size() + accScaleParams.size());
                weights.reserve(gyroBiasParams.size() + gyroScaleParams.size() + accBiasParams.size() + accScaleParams.size());

                constexpr int gyroBiasParamPos = AccelerometerStepCost<flags>::gyroBiasPos;
                constexpr int gyroScaleParamPos = AccelerometerStepCost<flags>::gyroScalePos;
                constexpr int accBiasParamPos = AccelerometerStepCost<flags>::accBiasPos;
                constexpr int accScaleParamPos = AccelerometerStepCost<flags>::accScalePos;


                for (StochasticProcessParam & p : gyroBiasParams) {
                    paramsMap.push_back(gyroBiasParamPos);
                    weights.push_back(p.weight);
                }

                for (StochasticProcessParam & p : gyroScaleParams) {
                    paramsMap.push_back(gyroScaleParamPos);
                    weights.push_back(p.weight);
                }

                for (StochasticProcessParam & p : accBiasParams) {
                    paramsMap.push_back(accBiasParamPos);
                    weights.push_back(p.weight);
                }

                for (StochasticProcessParam & p : accScaleParams) {
                    paramsMap.push_back(accScaleParamPos);
                    weights.push_back(p.weight);
                }

                accStepCost->setAccumulationWeights(paramsMap, weights);

                for (size_t i = 0; i < params.size(); i++) {
                    accStepCostFunction->AddParameterBlock(paramBlockSize);
                }
                accStepCostFunction->SetNumResiduals(nRes);

                weigthedAccStepCost->syncParametersSizes();

                problem.AddResidualBlock(weigthedAccStepCost, nullptr,
                                         params.data(),
                                         params.size());

            } else {

                AccCost* accStepCost =
                    AccelerometerStepCostBase::getIntegratedIMUDiff<flags, AccelerometerStepCostGravity<flags>::template Decorator>
                    (gyroSeq,
                     imuSeq,
                     trajNode->nodes[i-2].time,
                     trajNode->nodes[i-1].time,
                     trajNode->nodes[i].time,
                     _earth_center_pos);

                AutoDiffCostFuncT* accStepCostFunction =
                    new AutoDiffCostFuncT(accStepCost);

                WeightedCostFuncT* weigthedAccStepCost =
                    new WeightedCostFuncT(accStepCostFunction, weigthMat);

                auto params = getParametersForAccCostFunc<flags>(trajNode, accId, gyroId, i);

                static_assert(params.size() == WeightedCostFuncT::nParamsBlocks,
                              "non compatible numbers of parameters in parameters array and cost function");

                static_assert(AutoDiffCostFuncT::ParameterDims::kNumParameterBlocks ==
                                  WeightedCostFuncT::nParamsBlocks,
                              "non compatible numbers of parameters in autodiff cost and weigthed cost function");

                problem.AddResidualBlock(weigthedAccStepCost, nullptr, params.data(), params.size());

                if (addLogger) {
                    QString loggerName = QString("Accelerometer trajectory \"%1\" step time %2").arg(traj->objectName()).arg(time, 0, 'f', 2);
                    solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<nAccCostParams<flags>(),3>(accStepCostFunction, params));
                }

            }
        }

        return StatusOptionalReturn<void>();
    }

    bool _useOrientationPriors;

    double _defaultIntegrationTime;

    std::vector<std::array<double,3>> _accelerometersBiases;
    std::vector<std::array<double,3>> _accelerometersScales;

    std::vector<std::array<double,3>> _gyrosBiases;
    std::vector<std::array<double,3>> _gyrosScales;

    struct INSStochasticProcessOptParams {
        std::vector<std::array<double,3>> vals;
        std::vector<double> times;
    };

    std::vector<std::vector<INSStochasticProcessOptParams>> _accelerometersBiasesStochasticProcesses;
    std::vector<std::vector<INSStochasticProcessOptParams>> _accelerometersScalesStochasticProcesses;

    std::vector<std::vector<INSStochasticProcessOptParams>> _gyrosBiasesStochasticProcesses;
    std::vector<std::vector<INSStochasticProcessOptParams>> _gyrosScalesStochasticProcesses;

    void setupStochasticProcesses(
        std::vector<INSStochasticProcessOptParams> & paramSet,
        Trajectory* traj,
        double t0,
        double tf,
        ceres::Problem & problem,
        QVector<Trajectory::InsStochasticProcessDef> const& processesParams);

    struct StochasticProcessParam {

        StochasticProcessParam(double w, double* p) :
            weight(w),
            parameter(p) {

        }

        double weight;
        double* parameter;
    };

    inline std::vector<StochasticProcessParam> collectParameterSet(double time, std::vector<INSStochasticProcessOptParams> & params) {
        std::vector<StochasticProcessParam> biasParams;

        for (size_t i = 0; i < params.size(); i++) {
            double startT = params[i].times.front();
            double endT = params[i].times.back();

            double pos = (time - startT)/(endT - startT) * (params[i].times.size()-1);

            int sI = std::floor(pos);
            int nI = std::ceil(pos);

            if (sI == nI) {
                if (sI == 0) {
                    nI = 1;
                } else {
                    sI = nI-1;
                }
            }

            double sW = (params[i].times[nI] - time)/(params[i].times[nI] - params[i].times[sI]);
            double nW = (time - params[i].times[sI])/(params[i].times[nI] - params[i].times[sI]);

            assert(sI >= 0);
            assert(static_cast<size_t>(sI) < params[i].vals.size());
            assert(nI >= 0);
            assert(static_cast<size_t>(nI) < params[i].vals.size());
            assert(nI > sI);

            biasParams.emplace_back(sW, params[i].vals[sI].data());
            biasParams.emplace_back(nW, params[i].vals[nI].data());
        }

        return biasParams;
    }

    QMap<int, int> _accelerometerParametersIndex;
    QMap<int, int> _gyroParametersIndex;

    double _defaultGpsAccuracy;
    double _defaultOrientAccuracy;

    double _defaultAccAccuracy;
    double _defaultGyroAccuracy;

    std::array<double,3> _gravity;
    Eigen::Vector3d _earth_center_pos;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYBASESBAMODULE_H
