#ifndef STEREOVISIONAPP_TRAJECTORYBASESBAMODULE_H
#define STEREOVISIONAPP_TRAJECTORYBASESBAMODULE_H

#include "../modularsbasolver.h"

#include "../../datablocks/trajectory.h"
#include "../../utils/statusoptionalreturn.h"
#include "../costfunctors/weightedcostfunction.h"
#include "../costfunctors/imustepcost.h"

namespace StereoVisionApp {

class TrajectoryBaseSBAModule : public ModularSBASolver::SBAModule
{

public:

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

        using Traits = AccelerometerStepCostTraits<flags>;

        int n = 5; //no bias or scale factors

        if (Traits::WGBias) {
            n += 1;
        }

        if (Traits::WGScale) {
            n += 1;
        }

        if (Traits::WABias) {
            n += 1;
        }

        if (Traits::WAScale) {
            n += 1;
        }

        return n;
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

    template<bool WBias, bool WScale>
    StatusOptionalReturn<void> addTempltGyroObs(
        ModularSBASolver::TrajectoryNode* trajNode,
        Trajectory* traj,
        int gyroId,
        int i,
        double time,
        Trajectory::TimeCartesianSequence const& gyroSeq,
        double gyroAccuracy,
        ceres::Problem & problem,
        ModularSBASolver* solver,
        bool addLogger) {

        using CostF = GyroStepCost<WBias, WScale>;
        using AutoDiffCostFuncT = GyroTemplateParametrizedCost<ceres::AutoDiffCostFunction,CostF, WBias, WScale>;
        using WeightedCostFuncT = GyroParametrizedCost<StereoVisionApp::WeightedCostFunction, WBias, WScale>;

        static_assert(!std::is_same_v<AutoDiffCostFuncT, void>, "Unable to build autodiff cost function type");
        static_assert(!std::is_same_v<WeightedCostFuncT, void>, "Unable to build weighted cost function type");

        CostF* gyroStepCost =
            GyroStepCostBase::getIntegratedIMUDiff<WBias, WScale>(
                gyroSeq,
                trajNode->nodes[i-1].time,
                trajNode->nodes[i].time);

        AutoDiffCostFuncT* gyroStepCostFunction =
            new AutoDiffCostFuncT(gyroStepCost);

        double dt = trajNode->nodes[i].time - trajNode->nodes[i-1].time;
        Eigen::Matrix<double,3,3> weigthMat = Eigen::Matrix<double,3,3>::Identity();

        double poseUncertainty = gyroAccuracy*dt;

        weigthMat(0,0) = 1/poseUncertainty;
        weigthMat(1,1) = 1/poseUncertainty;
        weigthMat(2,2) = 1/poseUncertainty;

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

        return StatusOptionalReturn<void>();
    }

    template <int flags>
    StatusOptionalReturn<void> addTempltAccObs(
        ModularSBASolver::TrajectoryNode* trajNode,
        Trajectory* traj,
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

        using AccCost = AccelerometerStepCost<flags>;
        using AutoDiffCostFuncT = AccTemplateParametrizedCost<ceres::AutoDiffCostFunction,AccCost, flags>;
        using WeightedCostFuncT = AccParametrizedCost<StereoVisionApp::WeightedCostFunction, flags>;

        static_assert(!std::is_same_v<AutoDiffCostFuncT, void>, "Unable to build autodiff cost function type");
        static_assert(!std::is_same_v<WeightedCostFuncT, void>, "Unable to build weighted cost function type");

        AccCost* accStepCost =
            AccelerometerStepCostBase::getIntegratedIMUDiff<flags>
            (gyroSeq,
             imuSeq,
             trajNode->nodes[i-2].time,
             trajNode->nodes[i-1].time,
             trajNode->nodes[i].time);

        AutoDiffCostFuncT* accStepCostFunction =
            new AutoDiffCostFuncT(accStepCost);
        double dt = (trajNode->nodes[i].time - trajNode->nodes[i-2].time)/2;
        Eigen::Matrix<double,3,3> weigthMat = Eigen::Matrix<double,3,3>::Identity();

        double speedUncertainty = accAccuracy*dt;

        weigthMat(0,0) = 1/speedUncertainty;
        weigthMat(1,1) = 1/speedUncertainty;
        weigthMat(2,2) = 1/speedUncertainty;

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

        return StatusOptionalReturn<void>();
    }

    bool _useOrientationPriors;

    double _defaultIntegrationTime;

    std::vector<std::array<double,3>> _accelerometersBiases;
    std::vector<std::array<double,3>> _accelerometersScales;

    std::vector<std::array<double,3>> _gyrosBiases;
    std::vector<std::array<double,3>> _gyrosScales;

    QMap<int, int> _accelerometerParametersIndex;
    QMap<int, int> _gyroParametersIndex;

    double _defaultGpsAccuracy;
    double _defaultOrientAccuracy;

    double _defaultAccAccuracy;
    double _defaultGyroAccuracy;

    std::array<double,3> _gravity;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYBASESBAMODULE_H
