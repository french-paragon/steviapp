#include "imagealignementsbamodule.h"

#include "datablocks/image.h"
#include "datablocks/camera.h"
#include "datablocks/landmark.h"

#include "../costfunctors/modularuvprojection.h"
#include "../costfunctors/parametrizedxyz2uvcost.h"
#include "../costfunctors/leverarmcostfunctor.h"

#include "../costfunctors/fixedsizenormalprior.h"

namespace StereoVisionApp {

const char* ImageAlignementSBAModule::ModuleName = "SBAModule::ImageAlignement";

ImageAlignementSBAModule::ImageAlignementSBAModule()
{

}

QString ImageAlignementSBAModule::moduleName() const {
    return QObject::tr("Image alignement SBA module");
}

bool ImageAlignementSBAModule::addGraphReductorVariables(Project *currentProject, GenericSBAGraphReductor* graphReductor) {

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> imgs_v = currentProject->getIdsByClass(ImageFactory::imageClassName());

    for (qint64 id : imgs_v) {
        graphReductor->insertItem(id, 6);
    }

    return true;

}
bool ImageAlignementSBAModule::addGraphReductorObservations(Project *currentProject, GenericSBAGraphReductor* graphReductor) {

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> imgs_v = currentProject->getIdsByClass(ImageFactory::imageClassName());

    for (qint64 id : imgs_v) {
        Image* im = qobject_cast<Image*>(currentProject->getById(id));
        if (im == nullptr) {
            continue;
        }

        if (im->isEnabled() == false) {
            continue;
        }

        int nSelfObs = 0;

        if (im->xCoord().isSet() and im->yCoord().isSet() and im->zCoord().isSet()) {
            nSelfObs += 3;
        }

        if(im->xRot().isSet() and im->yRot().isSet() and im->zRot().isSet()) {
            nSelfObs += 3;
        }

        graphReductor->insertSelfObservation(id, nSelfObs);

        QVector<qint64> connections = im->getAttachedLandmarksIds();

        for (qint64 lmId : connections) {
            graphReductor->insertObservation(id, lmId, 2);
        }

        qint64 trajId = im->assignedTrajectory(); //if the image has a trajectory

        if (trajId >= 0) {
            graphReductor->insertObservation(id, trajId, 6);
        }
    }

    return true;

}

bool ImageAlignementSBAModule::setupParameters(ModularSBASolver* solver) {

    Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> imIdxs = currentProject->getIdsByClass(Image::staticMetaObject.className());

    for (qint64 imId : imIdxs) {

        if (!solver->itemIsObservable(imId)) {
            continue;
        }

        Image* im = qobject_cast<Image*>(currentProject->getById(imId));

        if (im == nullptr) {
            continue;
        }

        ModularSBASolver::PoseNode* imNode = solver->getNodeForFrame(imId, true);

        if (imNode == nullptr) {
            continue;
        }

        //check if a previous module assigned a projection module already to the frame.
        ModularSBASolver::ProjectorModule* projectionModule = solver->getProjectorForFrame(imId);

        if (projectionModule == nullptr) {

            Camera* cam = im->getAssignedCamera();

            if (cam == nullptr) {
                continue;
            }

            if (_cameraProjectors.contains(cam->internalId())) {
                projectionModule = _cameraProjectors[cam->internalId()];
                solver->assignProjectorToFrame(projectionModule, imId);
            } else {
                PinholdeCamProjModule* pcpm = new PinholdeCamProjModule(cam);
                projectionModule = pcpm;

                _cameraProjectors[cam->internalId()] = projectionModule;
                solver->addProjector(projectionModule);
                solver->assignProjectorToFrame(projectionModule, imId);
            }
        }

        if (projectionModule == nullptr) {
            continue;
        }

    }

    return true;
}

bool ImageAlignementSBAModule::init(ModularSBASolver* solver, ceres::Problem & problem) {

    Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    //add images
    QVector<qint64> imIdxs = currentProject->getIdsByClass(Image::staticMetaObject.className());


    //TODO: manage geo transforms for the images

    for (qint64 imId : imIdxs) {

        ModularSBASolver::PoseNode* imNode = solver->getPoseNode(imId);

        if (imNode == nullptr) {
            continue;
        }

        //check if a previous module assigned a projection module already to the frame.
        ModularSBASolver::ProjectorModule* projectionModule = solver->getProjectorForFrame(imId);

        if (projectionModule == nullptr) {
            continue;
        }

        Image* im = qobject_cast<Image*>(currentProject->getById(imId));

        if (im == nullptr) {
            continue;
        }

        //priors

        std::array<double, 3> raxis_prior;
        std::array<double, 3> t_prior;

        //TODO: check if the use of axis angle representation here is consistent.
        if (im->xRot().isSet() and im->yRot().isSet() and im->zRot().isSet()) {
            raxis_prior[0] = im->xRot().value();
            raxis_prior[1] = im->yRot().value();
            raxis_prior[2] = im->zRot().value();
        }

        if (im->xCoord().isSet() and im->yCoord().isSet() and im->zCoord().isSet()) {
            t_prior[0] = im->xCoord().value();
            t_prior[1] = im->yCoord().value();
            t_prior[2] = im->zCoord().value();
        }

        if (!im->isFixed() and im->xCoord().isUncertain() and im->yCoord().isUncertain() and im->zCoord().isUncertain()) {

            Eigen::Matrix3d At = Eigen::Matrix3d::Identity();

            At(0,0) = 1./std::abs(im->xCoord().stddev());
            At(1,1) = 1./std::abs(im->yCoord().stddev());
            At(2,2) = 1./std::abs(im->zCoord().stddev());

            Eigen::Vector3d bt;
            bt << t_prior[0],t_prior[1], t_prior[2];

            FixedSizeNormalPrior<3,3>* tPrior = new FixedSizeNormalPrior<3,3>(At, bt);
            problem.AddResidualBlock(tPrior, nullptr, imNode->t.data());

        }

        if (!im->isFixed() and im->xRot().isUncertain() and im->yRot().isUncertain() and im->zRot().isUncertain()) {

            Eigen::Matrix3d Araxis = Eigen::Matrix3d::Identity();

            Araxis(0,0) = 1./std::abs(im->xRot().stddev());
            Araxis(1,1) = 1./std::abs(im->yRot().stddev());
            Araxis(2,2) = 1./std::abs(im->zRot().stddev());

            Eigen::Vector3d  braxis;
            braxis << raxis_prior[0],raxis_prior[1], raxis_prior[2];

            FixedSizeNormalPrior<3,3>* rAxisPrior = new FixedSizeNormalPrior<3,3>(Araxis, (braxis));
            problem.AddResidualBlock(rAxisPrior, nullptr, imNode->rAxis.data());

        }

        //GCPs
        for (qint64 imlmId : im->listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName)) {

            ImageLandmark* iml = im->getImageLandmark(imlmId);

            if (iml == nullptr) {
                continue;
            }

            qint64 lmId = iml->attachedLandmarkid();

            if (!solver->itemIsObservable(lmId)) {
                continue;
            }

            ModularSBASolver::PositionNode* lmNode = solver->getPositionNode(lmId);

            if (lmNode == nullptr) {
                continue;
            }

            Eigen::Vector2d ptPos;
            ptPos.x() = iml->x().value();
            ptPos.y() = iml->y().value();

            Eigen::Matrix2d info = Eigen::Matrix2d::Identity();
            info(0,0) = (iml->x().isUncertain()) ? 1./iml->x().stddev() : 1;
            info(1,1) = (iml->y().isUncertain()) ? 1./iml->y().stddev() : 1;

            QString loggingLabel = QString("Projection cost for image id %1 (%2), landmark id %3 (%4)")
                                       .arg(im->internalId()).arg(im->objectName())
                                       .arg(lmId).arg(iml->attachedLandmarkName());

            projectionModule->addProjectionCostFunction(lmNode->pos.data(),
                                                        imNode->rAxis.data(),
                                                        imNode->t.data(),
                                                        ptPos,
                                                        info,
                                                        loggingLabel);
        }

        //stick to trajectory
        qint64 trajId = imNode->trajectoryId.value_or(-1);
        std::optional<double>& imTime = imNode->time;

        ModularSBASolver::TrajectoryNode* tNode = solver->getNodeForTrajectory(trajId, false);
        ModularSBASolver::LeverArmNode* lvNode = solver->getNodeForLeverArm(QPair<qint64,qint64>(im->assignedCamera(),trajId), false);

        if (tNode != nullptr and lvNode != nullptr and imTime.has_value()) {

            double time = imTime.value();
            int nodeId = tNode->getNodeForTime(time);

            if (nodeId >= 0 and nodeId < tNode->nodes.size()-1) {
                ModularSBASolver::TrajectoryPoseNode& previous = tNode->nodes[nodeId];
                ModularSBASolver::TrajectoryPoseNode& next = tNode->nodes[nodeId+1];

                double pTime = previous.time;
                double nTime = next.time;

                double wP = (nTime - time)/(nTime-pTime);
                double wN = (time - pTime)/(nTime-pTime);

                if (std::abs(wP-1) < 1e-3 or !std::isfinite(wP)) {

                    ParametrizedLeverArmCostFunctor* cost =
                            new ParametrizedLeverArmCostFunctor();

                    using ceresFunc = ceres::AutoDiffCostFunction<ParametrizedLeverArmCostFunctor,6,3,3,3,3,3,3>;
                    ceresFunc* costFunc = new ceresFunc(cost);

                    problem.AddResidualBlock(costFunc, nullptr,
                                             lvNode->rAxis.data(), lvNode->t.data(),
                                             previous.rAxis.data(), previous.t.data(),
                                             imNode->rAxis.data(), imNode->t.data());

                } else if (std::abs(wN-1) < 1e-3) {

                    ParametrizedLeverArmCostFunctor* cost =
                            new ParametrizedLeverArmCostFunctor();

                    using ceresFunc = ceres::AutoDiffCostFunction<ParametrizedLeverArmCostFunctor,6,3,3,3,3,3,3>;
                    ceresFunc* costFunc = new ceresFunc(cost);

                    problem.AddResidualBlock(costFunc, nullptr,
                                             lvNode->rAxis.data(), lvNode->t.data(),
                                             next.rAxis.data(), next.t.data(),
                                             imNode->rAxis.data(), imNode->t.data());

                } else {

                    ParametrizedInterpolatedLeverArmCostFunctor* cost =
                            new ParametrizedInterpolatedLeverArmCostFunctor(wP, wN);

                    using ceresFunc = ceres::AutoDiffCostFunction<ParametrizedInterpolatedLeverArmCostFunctor,6,3,3,3,3,3,3,3,3>;
                    ceresFunc* costFunc = new ceresFunc(cost);

                    problem.AddResidualBlock(costFunc, nullptr,
                                             lvNode->rAxis.data(), lvNode->t.data(),
                                             previous.rAxis.data(), previous.t.data(),
                                             next.rAxis.data(), next.t.data(),
                                             imNode->rAxis.data(), imNode->t.data());
                }
            }

        }

    }

    return true;

}

bool ImageAlignementSBAModule::writeResults(ModularSBASolver* solver) {


    Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    //add images and points
    QVector<qint64> imIdxs = currentProject->getIdsByClass(Image::staticMetaObject.className());

    for (qint64 imId : imIdxs) {

        ModularSBASolver::PoseNode* im_p = solver->getPoseNode(imId);

        if (im_p == nullptr) {
            continue;
        }

        Image* im = qobject_cast<Image*>(currentProject->getById(imId));

        if (im->isFixed()) {
            continue;
        }

        //TODO: manage geo transforms for the images
        floatParameterGroup<3> pos;
        pos.value(0) = static_cast<float>(im_p->t[0]);
        pos.value(1) = static_cast<float>(im_p->t[1]);
        pos.value(2) = static_cast<float>(im_p->t[2]);
        pos.setIsSet();
        im->setOptPos(pos);

        floatParameterGroup<3> rot;
        rot.value(0) = static_cast<float>(im_p->rAxis[0]);
        rot.value(1) = static_cast<float>(im_p->rAxis[1]);
        rot.value(2) = static_cast<float>(im_p->rAxis[2]);
        rot.setIsSet();
        im->setOptRot(rot);
    }

    return true;

}

bool ImageAlignementSBAModule::writeUncertainty(ModularSBASolver* solver) {

    //Basic data structures are managed by the modular sba solver directly.
    return true;
}

void ImageAlignementSBAModule::cleanup(ModularSBASolver* solver) {
    //Basic data structures are managed by the modular sba solver directly.
    Q_UNUSED(solver);
    return;
}


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

    ParametrizedXYZ2UVCost* projCost = new ParametrizedXYZ2UVCost(ptProjPos, ptProjStiffness);

    problem().AddResidualBlock(new ceres::AutoDiffCostFunction<ParametrizedXYZ2UVCost, 2, 3, 3, 3, 1, 2, 3, 2, 2>(projCost), nullptr,
                              pointData,
                              poseOrientation,
                              posePosition,
                              &_fLen,
                              _principalPoint.data(),
                              _radialDistortion.data(),
                              _tangentialDistortion.data(),
                              _skewDistortion.data());

    if (isVerbose() and !logLabel.isEmpty()) {

        ParametrizedXYZ2UVCost* projCostLog = new ParametrizedXYZ2UVCost(ptProjPos, Eigen::Matrix2d::Identity());

        constexpr int nArgs = 8;
        QString loggerName = logLabel;
        std::array<double*, nArgs> params = {pointData,
                                              poseOrientation,
                                              posePosition,
                                              &_fLen,
                                              _principalPoint.data(),
                                              _radialDistortion.data(),
                                              _tangentialDistortion.data(),
                                              _skewDistortion.data()};

        ModularSBASolver::AutoErrorBlockLogger<nArgs,2>* projErrorLogger =
            new ModularSBASolver::AutoErrorBlockLogger<nArgs,2>(
            new ceres::AutoDiffCostFunction<ParametrizedXYZ2UVCost, 2, 3, 3, 3, 1, 2, 3, 2, 2>(projCostLog),
            params,
            true);

        solver().addLogger(loggerName, projErrorLogger);
    }

    return true;
}

bool PinholdeCamProjModule::addCrossProjectionCostFunction(double* pose1Orientation,
                                                           double* pose1Position,
                                                           Eigen::Vector2d const& ptProj1Pos,
                                                           Eigen::Matrix2d const& ptProj1Stiffness,
                                                           double* pose2Orientation,
                                                           double* pose2Position,
                                                           Eigen::Vector2d const& ptProj2Pos,
                                                           Eigen::Matrix2d const& ptProj2Stiffness, const QString &logLabel) {


    if (!isSetup()) {
        return false;
    }

    //doing a uv2uv projection with distortion is non trivial. For this model of camera distortion it is better to use an intermediate landmark.
    return false;

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


} // namespace StereoVisionApp
