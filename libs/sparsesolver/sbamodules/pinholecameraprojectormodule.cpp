#include "pinholecameraprojectormodule.h"

#include "sparsesolver/costfunctors/modularuvprojection.h"
#include "sparsesolver/costfunctors/posedecoratorfunctors.h"

#include "datablocks/cameras/pushbroompinholecamera.h"

#include "costfunctors/fixedsizenormalprior.h"

#include "datablocks/image.h"
#include "datablocks/camera.h"

namespace StereoVisionApp {

using UVCostPB = InvertPose<UV2ParametrizedXYZCost<PinholePushbroomUVProjector,1,1,6,6>,0>;

using UVCost = InvertPose<UV2ParametrizedXYZCost<PinholeUVProjector,1,2,3,2,2>,0>;
using LeverArmCost = LeverArm<UVCost,Body2World|Body2Sensor,0>;
using PoseTransformCost = PoseTransform<UVCost,PoseTransformDirection::SourceToInitial,0>;
using PoseTransformLeverArmCost = PoseTransform<LeverArmCost,PoseTransformDirection::SourceToInitial,0>;

PinholdeCamProjModule::PinholdeCamProjModule(Camera* associatedCamera) :
    _associatedCamera(associatedCamera)
{

}

PinholdeCamProjModule::~PinholdeCamProjModule() {

}

QString PinholdeCamProjModule::moduleName() const {
    return "PinholeCamProjModule";
}

bool PinholdeCamProjModule::addProjectionCostFunction(double* pointData,
                                                      double* poseOrientation,
                                                      double* posePosition,
                                                      Eigen::Vector2d const& ptProjPos,
                                                      Eigen::Matrix2d const& ptProjStiffness,
                                                      const StereoVision::Geometry::RigidBodyTransform<double> &offset,
                                                      double *leverArmOrientation,
                                                      double *leverArmPosition,
                                                      const QString &logLabel) {


    if (!isSetup()) {
        return false;
    }

    if (offset.r.norm() < 1e-6 and offset.t.norm() < 1e-6) {

        if (leverArmOrientation != nullptr and leverArmPosition != nullptr) {

            LeverArmCost* projCost = new LeverArmCost(
                new PinholeUVProjector(_associatedCamera->imHeight(), _associatedCamera->imWidth()),
                ptProjPos,
                ptProjStiffness);

            constexpr int nArgs = 10;

            std::array<double*, nArgs> params = {poseOrientation,
                                                  posePosition,
                                                  leverArmOrientation,
                                                  leverArmPosition,
                                                  pointData,
                                                  &_fLen,
                                                  _principalPoint.data(),
                                                  _radialDistortion.data(),
                                                  _tangentialDistortion.data(),
                                                  _skewDistortion.data()};

            problem().AddResidualBlock(new ceres::AutoDiffCostFunction<LeverArmCost, 2, 3, 3, 3, 3, 3, 1, 2, 3, 2, 2>(projCost), nullptr,
                                       params.data(), nArgs);

            if (isVerbose() and !logLabel.isEmpty()) {

                QString loggerName = logLabel;
                LeverArmCost* projCostLog = new LeverArmCost(
                    new PinholeUVProjector(_associatedCamera->imHeight(), _associatedCamera->imWidth()),
                    ptProjPos,
                    Eigen::Matrix2d::Identity());

                ModularSBASolver::AutoErrorBlockLogger<nArgs,2>* projErrorLogger =
                    new ModularSBASolver::AutoErrorBlockLogger<nArgs,2>(
                        new ceres::AutoDiffCostFunction<LeverArmCost, 2, 3, 3, 3, 3, 3, 1, 2, 3, 2, 2>(projCostLog),
                        params,
                        true);

                solver().addLogger(loggerName, projErrorLogger);

            }

            return true;

        } else {
            UVCost* projCost = new UVCost(
                new PinholeUVProjector(_associatedCamera->imHeight(), _associatedCamera->imWidth()),
                ptProjPos,
                ptProjStiffness);

            constexpr int nArgs = 8;

            std::array<double*, nArgs> params = {poseOrientation,
                                                  posePosition,
                                                  pointData,
                                                  &_fLen,
                                                  _principalPoint.data(),
                                                  _radialDistortion.data(),
                                                  _tangentialDistortion.data(),
                                                  _skewDistortion.data()};

            problem().AddResidualBlock(new ceres::AutoDiffCostFunction<UVCost, 2, 3, 3, 3, 1, 2, 3, 2, 2>(projCost), nullptr,
                                       params.data(), nArgs);

            if (isVerbose() and !logLabel.isEmpty()) {

                QString loggerName = logLabel;
                UVCost* projCostLog = new UVCost(
                    new PinholeUVProjector(_associatedCamera->imHeight(), _associatedCamera->imWidth()),
                    ptProjPos,
                    Eigen::Matrix2d::Identity());

                ModularSBASolver::AutoErrorBlockLogger<nArgs,2>* projErrorLogger =
                    new ModularSBASolver::AutoErrorBlockLogger<nArgs,2>(
                        new ceres::AutoDiffCostFunction<UVCost, 2, 3, 3, 3, 1, 2, 3, 2, 2>(projCostLog),
                        params,
                        true);

                solver().addLogger(loggerName, projErrorLogger);

            }

            return true;
        }

    } else {

        if (leverArmOrientation != nullptr and leverArmPosition != nullptr) {

            PoseTransformLeverArmCost* projCost =
                new PoseTransformLeverArmCost(
                    offset,new PinholeUVProjector(_associatedCamera->imHeight(), _associatedCamera->imWidth()),
                    ptProjPos,
                    ptProjStiffness);

            constexpr int nArgs = 10;

            std::array<double*, nArgs> params = {poseOrientation,
                                                  posePosition,
                                                  leverArmOrientation,
                                                  leverArmPosition,
                                                  pointData,
                                                  &_fLen,
                                                  _principalPoint.data(),
                                                  _radialDistortion.data(),
                                                  _tangentialDistortion.data(),
                                                  _skewDistortion.data()};

            problem().AddResidualBlock(new ceres::AutoDiffCostFunction<PoseTransformLeverArmCost, 2, 3, 3, 3, 3, 3, 1, 2, 3, 2, 2>(projCost), nullptr,
                                       params.data(), nArgs);


            if (isVerbose() and !logLabel.isEmpty()) {

                QString loggerName = logLabel;
                PoseTransformLeverArmCost* projCostLog =
                    new PoseTransformLeverArmCost(
                        offset,
                        new PinholeUVProjector(_associatedCamera->imHeight(), _associatedCamera->imWidth()),
                        ptProjPos,
                        Eigen::Matrix2d::Identity());

                ModularSBASolver::AutoErrorBlockLogger<nArgs,2>* projErrorLogger =
                    new ModularSBASolver::AutoErrorBlockLogger<nArgs,2>(
                        new ceres::AutoDiffCostFunction<PoseTransformLeverArmCost, 2, 3, 3, 3, 3, 3, 1, 2, 3, 2, 2>(projCostLog),
                        params,
                        true);

                solver().addLogger(loggerName, projErrorLogger);

            }


            return true;

        } else {

            PoseTransformCost* projCost = new PoseTransformCost(offset,
                                                                new PinholeUVProjector(_associatedCamera->imHeight(), _associatedCamera->imWidth()),
                                                                ptProjPos,
                                                                ptProjStiffness);

            constexpr int nArgs = 8;

            std::array<double*, nArgs> params = {poseOrientation,
                                                  posePosition,
                                                  pointData,
                                                  &_fLen,
                                                  _principalPoint.data(),
                                                  _radialDistortion.data(),
                                                  _tangentialDistortion.data(),
                                                  _skewDistortion.data()};

            problem().AddResidualBlock(new ceres::AutoDiffCostFunction<PoseTransformCost, 2, 3, 3, 3, 1, 2, 3, 2, 2>(projCost), nullptr,
                                       params.data(), nArgs);

            if (isVerbose() and !logLabel.isEmpty()) {

                QString loggerName = logLabel;
                PoseTransformCost* projCostLog = new PoseTransformCost(
                    offset,
                    new PinholeUVProjector(_associatedCamera->imHeight(), _associatedCamera->imWidth()),
                    ptProjPos,
                    Eigen::Matrix2d::Identity());

                ModularSBASolver::AutoErrorBlockLogger<nArgs,2>* projErrorLogger =
                    new ModularSBASolver::AutoErrorBlockLogger<nArgs,2>(
                        new ceres::AutoDiffCostFunction<PoseTransformCost, 2, 3, 3, 3, 1, 2, 3, 2, 2>(projCostLog),
                        params,
                        true);

                solver().addLogger(loggerName, projErrorLogger);

            }

            return true;

        }
    }

    return false;
}

ModularSBASolver::ProjectorModule::ProjectionInfos PinholdeCamProjModule::getProjectionInfos() {


    if (!isSetup()) {
        return ModularSBASolver::ProjectorModule::ProjectionInfos{nullptr,
                                                                  std::vector<int>(),
                                                                  std::vector<double*>()};
    }

    return ModularSBASolver::ProjectorModule::ProjectionInfos{nullptr,
                                                              std::vector<int>(),
                                                              std::vector<double*>()};
}

bool PinholdeCamProjModule::init() {

    if (!isSetup()) {
        return false;
    }

    Camera*& c = _associatedCamera;

    Eigen::Vector2d extend;
    extend.x() = c->imSize().width();
    extend.y() = c->imSize().height();

    if (c->optimizedOpticalCenterX().isSet() and
        c->optimizedOpticalCenterY().isSet()) {

        _principalPoint[0] = c->optimizedOpticalCenterX().value();
        _principalPoint[1] = c->optimizedOpticalCenterY().value();

    } else {

        _principalPoint[0] = c->opticalCenterX().value();
        _principalPoint[1] = c->opticalCenterY().value();
    }

    problem().AddParameterBlock(_principalPoint.data(), _principalPoint.size());

    if (c->optimizedFLen().isSet()) {
        _fLen = c->optimizedFLen().value();
    } else {
        _fLen = c->fLen().value();
    }

    problem().AddParameterBlock(&_fLen, 1);

    if (c->isFixed() or solver().getFixedParametersFlag()&FixedParameter::CameraInternal) {
        problem().SetParameterBlockConstant(&_fLen);
        problem().SetParameterBlockConstant(_principalPoint.data());
    }


    if (c->optimizedK1().isSet() and
        c->optimizedK2().isSet() and
        c->optimizedK3().isSet()) {

        _radialDistortion[0] = c->optimizedK1().value();
        _radialDistortion[1] = c->optimizedK2().value();
        _radialDistortion[2] = c->optimizedK3().value();

    } else {

        _radialDistortion[0] = c->k1().value();
        _radialDistortion[1] = c->k2().value();
        _radialDistortion[2] = c->k3().value();
    }

    problem().AddParameterBlock(_radialDistortion.data(), _radialDistortion.size());

    if (!c->useRadialDistortionModel()) {
        _radialDistortion[0] = 0;
        _radialDistortion[1] = 0;
        _radialDistortion[2] = 0;

        problem().SetParameterBlockConstant(_radialDistortion.data());
    }

    if (c->isFixed() or solver().getFixedParametersFlag()&FixedParameter::CameraInternal) {
        problem().SetParameterBlockConstant(_radialDistortion.data());
    }

    if (c->optimizedP1().isSet() and
        c->optimizedP2().isSet()) {

        _tangentialDistortion[0] = c->optimizedP1().value();
        _tangentialDistortion[1] = c->optimizedP2().value();

    } else {

        _tangentialDistortion[0] = c->p1().value();
        _tangentialDistortion[1] = c->p2().value();
    }

    problem().AddParameterBlock(_tangentialDistortion.data(), _tangentialDistortion.size());

    if (!c->useTangentialDistortionModel()) {
        _tangentialDistortion[0] = 0;
        _tangentialDistortion[1] = 0;

        problem().SetParameterBlockConstant(_tangentialDistortion.data());
    }

    if (c->isFixed() or solver().getFixedParametersFlag()&FixedParameter::CameraInternal) {
        problem().SetParameterBlockConstant(_tangentialDistortion.data());
    }

    if (c->optimizedB1().isSet() and
        c->optimizedB2().isSet()) {

        _skewDistortion[0] = c->optimizedB1().value();
        _skewDistortion[1] = c->optimizedB2().value();

    } else {

        _skewDistortion[0] = c->B1().value();
        _skewDistortion[1] = c->B2().value();
    }

    problem().AddParameterBlock(_skewDistortion.data(), _skewDistortion.size());

    if (!c->useSkewDistortionModel()) {
        _skewDistortion[0] = 0;
        _skewDistortion[1] = 0;

        problem().SetParameterBlockConstant(_skewDistortion.data());
    }

    if (c->isFixed() or solver().getFixedParametersFlag()&FixedParameter::CameraInternal) {

        problem().SetParameterBlockConstant(_skewDistortion.data());
    }

    return true;

}

bool PinholdeCamProjModule::writeResults() {

    if (!isSetup()) {
        return false;
    }

    Camera* cam = _associatedCamera;

    if (cam->isFixed()) {
        return true;
    }

    cam->clearOptimized();

    cam->setOptimizedFLen(static_cast<float>(_fLen));
    cam->setOptimizedOpticalCenterX(static_cast<float>(_principalPoint[0]));
    cam->setOptimizedOpticalCenterY(static_cast<float>(_principalPoint[1]));

    if (cam->useRadialDistortionModel()) {

        cam->setOptimizedK1(static_cast<float>(_radialDistortion[0]));
        cam->setOptimizedK2(static_cast<float>(_radialDistortion[1]));
        cam->setOptimizedK3(static_cast<float>(_radialDistortion[2]));
    }

    if (cam->useTangentialDistortionModel()) {

        cam->setOptimizedP1(static_cast<float>(_tangentialDistortion[0]));
        cam->setOptimizedP2(static_cast<float>(_tangentialDistortion[1]));
    }

    if (cam->useSkewDistortionModel()) {

        cam->setOptimizedB1(static_cast<float>(_skewDistortion[0]));
        cam->setOptimizedB2(static_cast<float>(_skewDistortion[1]));
    }

    return true;

}

bool PinholdeCamProjModule::writeUncertainty() {

    //TODO: get a way to write uncertainty.
    return true;
}

void PinholdeCamProjModule::cleanup() {
    return;
}


PinholePushBroomCamProjectorModule::PinholePushBroomCamProjectorModule(PushBroomPinholeCamera* associatedCamera) :
    _associatedCamera(associatedCamera)
{

}
PinholePushBroomCamProjectorModule::~PinholePushBroomCamProjectorModule() {

}

QString PinholePushBroomCamProjectorModule::moduleName() const {
    return "PinholePushBroomCamProjectorModule";
}

bool PinholePushBroomCamProjectorModule::addProjectionCostFunction(double* pointData,
                                                                   double* poseOrientation,
                                                                   double* posePosition,
                                                                   Eigen::Vector2d const& ptProjPos,
                                                                   Eigen::Matrix2d const& ptProjStiffness,
                                                                   const StereoVision::Geometry::RigidBodyTransform<double> &offset,
                                                                   double *leverArmOrientation,
                                                                   double *leverArmPosition,
                                                                   const QString &logLabel) {

    if (!isSetup()) {
        return false;
    }

    constexpr int LeverArmConfiguration = Body2World|Body2Sensor;
    constexpr PoseTransformDirection poseTransformDirection = PoseTransformDirection::SourceToInitial;
    constexpr int poseParamIdx = 0;
    /*using LeverArmCostPB = LeverArm<UVCostPB,Body2World|Body2Sensor,0>;
    using PoseTransformCostPB = PoseTransform<UVCostPB,PoseTransformDirection::SourceToInitial,0>;
    using PoseTransformLeverArmCostPB = PoseTransform<LeverArmCostPB,PoseTransformDirection::SourceToInitial,0>;*/

    using PoseCostFunctionBuilder =
        ModifiedPoseCostFunctionBuilderHelper<UVCostPB, poseTransformDirection, LeverArmConfiguration, poseParamIdx, 2, 3, 3, 3, 1, 1, 6, 6>;

    constexpr int baseNArgs = 7;

    std::array<double*, baseNArgs> baseParams = { poseOrientation,
                                                  posePosition,
                                                  pointData,
                                                  &_fLen,
                                                  &_principalPoint,
                                                  _horizontalDistortion.data(),
                                                  _verticalDistortion.data()};

    auto costFuncInfos = PoseCostFunctionBuilder::buildPoseShiftedCostFunction(baseParams.data(),
                                                                               offset,
                                                                               leverArmOrientation,
                                                                               leverArmPosition,
                                                                               new PinholePushbroomUVProjector(_associatedCamera->imWidth()),
                                                                               ptProjPos,
                                                                               ptProjStiffness);

    problem().AddResidualBlock(costFuncInfos.costFunction, nullptr,
                               costFuncInfos.params.data(),
                               costFuncInfos.params.size());

    if (costFuncInfos.costFunction != nullptr) {
        return false;
    }

    if (isVerbose() and !logLabel.isEmpty()) {

        QString loggerName = logLabel;

        auto logCostFuncInfos = PoseCostFunctionBuilder::buildPoseShiftedCostFunction(baseParams.data(),
                                                                                      offset,
                                                                                      leverArmOrientation,
                                                                                      leverArmPosition,
                                                                                      new PinholePushbroomUVProjector(_associatedCamera->imWidth()),
                                                                                      ptProjPos,
                                                                                      Eigen::Matrix2d::Identity());

        if (logCostFuncInfos.costFunction != nullptr) {

            if (costFuncInfos.params.size() == baseNArgs) {

                ModularSBASolver::AutoErrorBlockLogger<baseNArgs,2>::ParamsType paramsArray;

                for (int i = 0; i < baseNArgs; i++) {
                    paramsArray[i] = costFuncInfos.params[i];
                }

                ModularSBASolver::AutoErrorBlockLogger<baseNArgs,2>* projErrorLogger =
                    new ModularSBASolver::AutoErrorBlockLogger<baseNArgs,2>(
                    logCostFuncInfos.costFunction,
                    paramsArray,
                    true);

                solver().addLogger(loggerName, projErrorLogger);

            } else if (costFuncInfos.params.size() == baseNArgs+2) {

                ModularSBASolver::AutoErrorBlockLogger<baseNArgs+2,2>::ParamsType paramsArray;

                for (int i = 0; i < baseNArgs+2; i++) {
                    paramsArray[i] = costFuncInfos.params[i];
                }

                ModularSBASolver::AutoErrorBlockLogger<baseNArgs+2,2>* projErrorLogger =
                    new ModularSBASolver::AutoErrorBlockLogger<baseNArgs+2,2>(
                    logCostFuncInfos.costFunction,
                    paramsArray,
                    true);
                solver().addLogger(loggerName, projErrorLogger);
            }
        }

    }

    return true;

}

ModularSBASolver::ProjectorModule::ProjectionInfos PinholePushBroomCamProjectorModule::getProjectionInfos() {


    if (!isSetup()) {
        return ModularSBASolver::ProjectorModule::ProjectionInfos{nullptr,
                                                                  std::vector<int>(),
                                                                  std::vector<double*>()};
    }


    std::vector<double*> params = {&_fLen,
                                    &_principalPoint,
                                    _horizontalDistortion.data(),
                                    _verticalDistortion.data()};

    std::vector<int> paramsSizes = {1, 1, int(_horizontalDistortion.size()), int(_verticalDistortion.size())};

    ModularUVProjection* projector = new AnyUVProjection<PinholePushbroomUVProjector, 4>(
        new PinholePushbroomUVProjector(_associatedCamera->imWidth()));

    return {projector, paramsSizes, params};


}

bool PinholePushBroomCamProjectorModule::init() {

    if (!isSetup()) {
        return false;
    }

    PushBroomPinholeCamera*& c = _associatedCamera;

    if (c->optimizedOpticalCenterX().isSet()) {
        _principalPoint = c->optimizedOpticalCenterX().value();
    } else {
        _principalPoint = c->opticalCenterX().value();
    }

    problem().AddParameterBlock(&_principalPoint, 1);

    if (c->optimizedFLen().isSet()) {
        _fLen = c->optimizedFLen().value();
    } else {
        _fLen = c->fLen().value();
    }

    problem().AddParameterBlock(&_fLen, 1);

    if (c->isFixed() or solver().getFixedParametersFlag()&FixedParameter::CameraInternal) {
        problem().SetParameterBlockConstant(&_fLen);
        problem().SetParameterBlockConstant(&_principalPoint);

    } else {

        if (c->fLen().isSet() and c->fLen().isUncertain()) {

            Eigen::Matrix<double,1,1> stiffness;
            Eigen::Matrix<double,1,1> target;

            stiffness(0,0) = 1/c->fLen().stddev();
            target(0,0) = c->fLen().value();

            FixedSizeNormalPrior<1,1>* prior = new FixedSizeNormalPrior<1,1>(stiffness, target);

            problem().AddResidualBlock(prior, nullptr, &_fLen);
        }

        if (c->opticalCenterX().isSet() and c->opticalCenterX().isUncertain()) {

            Eigen::Matrix<double,1,1> stiffness;
            Eigen::Matrix<double,1,1> target;

            stiffness(0,0) = 1/c->opticalCenterX().stddev();
            target(0,0) = c->opticalCenterX().value();

            FixedSizeNormalPrior<1,1>* prior = new FixedSizeNormalPrior<1,1>(stiffness, target);

            problem().AddResidualBlock(prior, nullptr, &_principalPoint);
        }

    }

    if (c->optimizedA0().isSet() and
            c->optimizedA1().isSet() and
            c->optimizedA2().isSet() and
            c->optimizedA3().isSet() and
            c->optimizedA4().isSet() and
            c->optimizedA5().isSet()) {

        _horizontalDistortion[0] = c->optimizedA0().value();
        _horizontalDistortion[1] = c->optimizedA1().value();
        _horizontalDistortion[2] = c->optimizedA2().value();
        _horizontalDistortion[3] = c->optimizedA3().value();
        _horizontalDistortion[4] = c->optimizedA4().value();
        _horizontalDistortion[5] = c->optimizedA5().value();

    } else {

        _horizontalDistortion[0] = c->a0().value();
        _horizontalDistortion[1] = c->a1().value();
        _horizontalDistortion[2] = c->a2().value();
        _horizontalDistortion[3] = c->a3().value();
        _horizontalDistortion[4] = c->a4().value();
        _horizontalDistortion[5] = c->a5().value();
    }

    if (c->optimizedB0().isSet() and
            c->optimizedB1().isSet() and
            c->optimizedB2().isSet() and
            c->optimizedB3().isSet() and
            c->optimizedB4().isSet() and
            c->optimizedB5().isSet()) {

        _verticalDistortion[0] = c->optimizedB0().value();
        _verticalDistortion[1] = c->optimizedB1().value();
        _verticalDistortion[2] = c->optimizedB2().value();
        _verticalDistortion[3] = c->optimizedB3().value();
        _verticalDistortion[4] = c->optimizedB4().value();
        _verticalDistortion[5] = c->optimizedB5().value();

    } else {

        _verticalDistortion[0] = c->b0().value();
        _verticalDistortion[1] = c->b1().value();
        _verticalDistortion[2] = c->b2().value();
        _verticalDistortion[3] = c->b3().value();
        _verticalDistortion[4] = c->b4().value();
        _verticalDistortion[5] = c->b5().value();
    }

    problem().AddParameterBlock(_horizontalDistortion.data(), _horizontalDistortion.size());
    problem().AddParameterBlock(_verticalDistortion.data(), _verticalDistortion.size());

    if (c->isFixed() or solver().getFixedParametersFlag()&FixedParameter::CameraInternal) {
        problem().SetParameterBlockConstant(_horizontalDistortion.data());
        problem().SetParameterBlockConstant(_verticalDistortion.data());
    } else {

        if (c->a0().isUncertain() and
                c->a1().isUncertain() and
                c->a2().isUncertain() and
                c->a3().isUncertain() and
                c->a4().isUncertain() and
                c->a5().isUncertain()) {

            Eigen::Matrix<double,6,1> target;
            Eigen::Matrix<double,6,6> stiffness = Eigen::Matrix<double,6,6>::Identity();

            target[0] = c->a0().value();
            target[1] = c->a1().value();
            target[2] = c->a2().value();
            target[3] = c->a3().value();
            target[4] = c->a4().value();
            target[5] = c->a5().value();

            stiffness(0,0) = 1/c->a0().stddev();
            stiffness(1,1) = 1/c->a1().stddev();
            stiffness(2,2) = 1/c->a2().stddev();
            stiffness(3,3) = 1/c->a3().stddev();
            stiffness(4,4) = 1/c->a4().stddev();
            stiffness(5,5) = 1/c->a5().stddev();

            FixedSizeNormalPrior<6,6>* prior = new FixedSizeNormalPrior<6,6>(stiffness, target);

            problem().AddResidualBlock(prior, nullptr, _horizontalDistortion.data());

        }

        if (c->b0().isUncertain() and
                c->b1().isUncertain() and
                c->b2().isUncertain() and
                c->b3().isUncertain() and
                c->b4().isUncertain() and
                c->b5().isUncertain()) {

            Eigen::Matrix<double,6,1> target;
            Eigen::Matrix<double,6,6> stiffness = Eigen::Matrix<double,6,6>::Identity();

            target[0] = c->b0().value();
            target[1] = c->b1().value();
            target[2] = c->b2().value();
            target[3] = c->b3().value();
            target[4] = c->b4().value();
            target[5] = c->b5().value();

            stiffness(0,0) = 1/c->b0().stddev();
            stiffness(1,1) = 1/c->b1().stddev();
            stiffness(2,2) = 1/c->b2().stddev();
            stiffness(3,3) = 1/c->b3().stddev();
            stiffness(4,4) = 1/c->b4().stddev();
            stiffness(5,5) = 1/c->b5().stddev();

            FixedSizeNormalPrior<6,6>* prior = new FixedSizeNormalPrior<6,6>(stiffness, target);

            problem().AddResidualBlock(prior, nullptr, _verticalDistortion.data());

        }
    }

    return true;

}

bool PinholePushBroomCamProjectorModule::writeResults() {

    if (!isSetup()) {
        return false;
    }

    if (_associatedCamera == nullptr) {
        return false;
    }

    PushBroomPinholeCamera*& cam = _associatedCamera;

    if (cam->isFixed()) {
        return true;
    }

    cam->clearOptimized();

    cam->setOptimizedFLen(static_cast<float>(_fLen));
    cam->setOptimizedOpticalCenterX(static_cast<float>(_principalPoint));

    cam->setOptimizedA0(_horizontalDistortion[0]);
    cam->setOptimizedA1(_horizontalDistortion[1]);
    cam->setOptimizedA2(_horizontalDistortion[2]);
    cam->setOptimizedA3(_horizontalDistortion[3]);
    cam->setOptimizedA4(_horizontalDistortion[4]);
    cam->setOptimizedA5(_horizontalDistortion[5]);

    cam->setOptimizedB0(_verticalDistortion[0]);
    cam->setOptimizedB1(_verticalDistortion[1]);
    cam->setOptimizedB2(_verticalDistortion[2]);
    cam->setOptimizedB3(_verticalDistortion[3]);
    cam->setOptimizedB4(_verticalDistortion[4]);
    cam->setOptimizedB5(_verticalDistortion[5]);

    return true;

}

std::vector<std::pair<const double*, const double*>> PinholePushBroomCamProjectorModule::requestUncertainty() {


    if (!isSetup()) {
        return std::vector<std::pair<const double*, const double*>>();
    }

    std::vector<std::pair<const double*, const double*>> ret;
    ret.reserve(4);

    if (!problem().IsParameterBlockConstant(&_fLen)) {
        ret.push_back({&_fLen,&_fLen});
    }

    if (!problem().IsParameterBlockConstant(&_principalPoint)) {
        ret.push_back({&_principalPoint,&_principalPoint});
    }

    if (!problem().IsParameterBlockConstant(_horizontalDistortion.data())) {
        ret.push_back({_horizontalDistortion.data(),_horizontalDistortion.data()});
    }

    if (!problem().IsParameterBlockConstant(_verticalDistortion.data())) {
        ret.push_back({_verticalDistortion.data(),_verticalDistortion.data()});
    }

    return ret;

}

bool PinholePushBroomCamProjectorModule::writeUncertainty() {

    if (!isSetup()) {
        return false;
    }

    if (_associatedCamera == nullptr) {
        return false;
    }

    std::optional<Eigen::MatrixXd> covBlock = solver().getCovarianceBlock({&_fLen,&_fLen});

    if (covBlock.has_value()) {
        floatParameter fLen = _associatedCamera->optimizedFLen();
        fLen.setUncertainty(std::sqrt(covBlock.value()(0,0)));
        _associatedCamera->setOptimizedFLen(fLen);
    }

    covBlock = solver().getCovarianceBlock({&_principalPoint,&_principalPoint});

    if (covBlock.has_value()) {
        floatParameter oCX = _associatedCamera->optimizedOpticalCenterX();
        oCX.setUncertainty(std::sqrt(covBlock.value()(0,0)));
        _associatedCamera->setOptimizedOpticalCenterX(oCX);
    }

    covBlock = solver().getCovarianceBlock({_horizontalDistortion.data(),_horizontalDistortion.data()});

    if (covBlock.has_value()) {

        floatParameter oA0 = _associatedCamera->optimizedA0();
        oA0.setUncertainty(std::sqrt(covBlock.value()(0,0)));
        _associatedCamera->setOptimizedA0(oA0);

        floatParameter oA1 = _associatedCamera->optimizedA1();
        oA1.setUncertainty(std::sqrt(covBlock.value()(1,1)));
        _associatedCamera->setOptimizedA1(oA1);

        floatParameter oA2 = _associatedCamera->optimizedA2();
        oA2.setUncertainty(std::sqrt(covBlock.value()(2,2)));
        _associatedCamera->setOptimizedA2(oA2);

        floatParameter oA3 = _associatedCamera->optimizedA3();
        oA3.setUncertainty(std::sqrt(covBlock.value()(3,3)));
        _associatedCamera->setOptimizedA3(oA3);

        floatParameter oA4 = _associatedCamera->optimizedA4();
        oA4.setUncertainty(std::sqrt(covBlock.value()(4,4)));
        _associatedCamera->setOptimizedA4(oA4);

        floatParameter oA5 = _associatedCamera->optimizedA5();
        oA5.setUncertainty(std::sqrt(covBlock.value()(5,5)));
        _associatedCamera->setOptimizedA5(oA5);
    }

    covBlock = solver().getCovarianceBlock({_verticalDistortion.data(),_verticalDistortion.data()});

    if (covBlock.has_value()) {

        floatParameter oB0 = _associatedCamera->optimizedB0();
        oB0.setUncertainty(std::sqrt(covBlock.value()(0,0)));
        _associatedCamera->setOptimizedB0(oB0);

        floatParameter oB1 = _associatedCamera->optimizedB1();
        oB1.setUncertainty(std::sqrt(covBlock.value()(1,1)));
        _associatedCamera->setOptimizedB1(oB1);

        floatParameter oB2 = _associatedCamera->optimizedB2();
        oB2.setUncertainty(std::sqrt(covBlock.value()(2,2)));
        _associatedCamera->setOptimizedB2(oB2);

        floatParameter oB3 = _associatedCamera->optimizedB3();
        oB3.setUncertainty(std::sqrt(covBlock.value()(3,3)));
        _associatedCamera->setOptimizedB3(oB3);

        floatParameter oB4 = _associatedCamera->optimizedB4();
        oB4.setUncertainty(std::sqrt(covBlock.value()(4,4)));
        _associatedCamera->setOptimizedB4(oB4);

        floatParameter oB5 = _associatedCamera->optimizedB5();
        oB5.setUncertainty(std::sqrt(covBlock.value()(5,5)));
        _associatedCamera->setOptimizedB5(oB5);
    }

    return true;
}

void PinholePushBroomCamProjectorModule::cleanup() {

    if (!isSetup()) {
        return;
    }

    return;
}

} // namespace StereoVisionApp
