#include "imagealignementsbamodule.h"

#include "datablocks/image.h"
#include "datablocks/camera.h"

#include "../costfunctors/leverarmcostfunctor.h"

#include "../costfunctors/fixedsizenormalprior.h"

#include "./pinholecameraprojectormodule.h"

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
                PinholeCamProjModule* pcpm = new PinholeCamProjModule(cam);
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

} // namespace StereoVisionApp
