#include "localcoordinatesystemsbamodule.h"

#include "datablocks/trajectory.h"
#include "datablocks/landmark.h"
#include "datablocks/localcoordinatesystem.h"
#include "datablocks/correspondencesset.h"
#include "datablocks/image.h"

#include "costfunctors/localpointalignementcost.h"
#include "costfunctors/local3dcoalignementcost.h"

#include <ceres/normal_prior.h>

namespace StereoVisionApp {

const char* LocalCoordinateSystemSBAModule::ModuleName = "SBAModule::LocalCoordinateSystem";

LocalCoordinateSystemSBAModule::LocalCoordinateSystemSBAModule()
{

}

bool LocalCoordinateSystemSBAModule::addGraphReductorVariables(Project *currentProject, GenericSBAGraphReductor* graphReductor) {

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> lcsIdxs = currentProject->getIdsByClass(LocalCoordinateSystem::staticMetaObject.className());

    for (qint64 lcsId : lcsIdxs) {
        graphReductor->insertItem(lcsId, 6);
    }

    return true;
}
bool LocalCoordinateSystemSBAModule::addGraphReductorObservations(Project *currentProject, GenericSBAGraphReductor* graphReductor) {

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> lcsIdxs = currentProject->getIdsByClass(LocalCoordinateSystem::staticMetaObject.className());

    for (qint64 id : lcsIdxs) {

        LocalCoordinateSystem* lcs = qobject_cast<LocalCoordinateSystem*>(currentProject->getById(id));

        if (lcs == nullptr) {
            continue;
        }

        if (lcs->isEnabled() == false) {
            continue;
        }

        int nSelfObs = 0;

        if (lcs->xCoord().isSet() and lcs->yCoord().isSet() and lcs->zCoord().isSet()) {
            nSelfObs += 3;
        }

        if(lcs->xRot().isSet() and lcs->yRot().isSet() and lcs->zRot().isSet()) {
            nSelfObs += 3;
        }

        graphReductor->insertSelfObservation(id, nSelfObs);

        QVector<qint64> connections = lcs->getAttachedLandmarksIds();

        for (qint64 lmId : connections) {
            graphReductor->insertObservation(id, lmId, 3);
        }
    }

    return true;

}

bool LocalCoordinateSystemSBAModule::setupParameters(ModularSBASolver* solver) {

    Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    constexpr bool createIfMissing = true;
    constexpr bool dontCreateIfMissing = false;

    QVector<qint64> lcsIdxs = currentProject->getIdsByClass(LocalCoordinateSystem::staticMetaObject.className());

    for (qint64 lcsId : lcsIdxs) {

        if (!solver->itemIsObservable(lcsId)) {
            continue;
        }

        LocalCoordinateSystem* lcs = qobject_cast<LocalCoordinateSystem*>(currentProject->getById(lcsId));

        if (lcs == nullptr) {
            continue;
        }

        ModularSBASolver::PoseNode* lcsPoseNode = solver->getNodeForLocalCoordinates(lcsId, createIfMissing);

        if (lcsPoseNode == nullptr) {
            continue;
        }

    }

    return true;
}

bool LocalCoordinateSystemSBAModule::init(ModularSBASolver* solver, ceres::Problem & problem) {

    Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    constexpr bool dontCreateIfMissing = false;

    QVector<qint64> lcsIdxs = currentProject->getIdsByClass(LocalCoordinateSystem::staticMetaObject.className());

    for (qint64 lcsId : lcsIdxs) {

        if (!solver->itemIsObservable(lcsId)) {
            continue;
        }

        LocalCoordinateSystem* lcs = qobject_cast<LocalCoordinateSystem*>(currentProject->getById(lcsId));

        if (lcs == nullptr) {
            continue;
        }

        ModularSBASolver::PoseNode* lcsPoseNode = solver->getPoseNode(lcsId);

        if (lcsPoseNode == nullptr) {
            continue;
        }

        //priors

        std::array<double, 3> raxis_prior;
        std::array<double, 3> t_prior;

        //TODO: check if the use of axis angle representation here is consistent.
        if (lcs->xRot().isSet() and lcs->yRot().isSet() and lcs->zRot().isSet()) {
            raxis_prior[0] = lcs->xRot().value();
            raxis_prior[1] = lcs->yRot().value();
            raxis_prior[2] = lcs->zRot().value();
        }

        if (lcs->xCoord().isSet() and lcs->yCoord().isSet() and lcs->zCoord().isSet()) {
            t_prior[0] = lcs->xCoord().value();
            t_prior[1] = lcs->yCoord().value();
            t_prior[2] = lcs->zCoord().value();
        }

        if (!lcs->isFixed() and lcs->xCoord().isUncertain() and lcs->yCoord().isUncertain() and lcs->zCoord().isUncertain()) {

            ceres::Matrix At;
            At.setConstant(3,3,0);

            At(0,0) = 1./std::abs(lcs->xCoord().stddev());
            At(1,1) = 1./std::abs(lcs->yCoord().stddev());
            At(2,2) = 1./std::abs(lcs->zCoord().stddev());

            ceres::Vector bt;
            bt.resize(3);

            bt << t_prior[0],t_prior[1], t_prior[2];

            ceres::NormalPrior* tPrior = new ceres::NormalPrior(At, bt);
            problem.AddResidualBlock(tPrior, nullptr, lcsPoseNode->t.data());

        }

        if (!lcs->isFixed() and lcs->xRot().isUncertain() and lcs->yRot().isUncertain() and lcs->zRot().isUncertain()) {

            ceres::Matrix Araxis;
            Araxis.setConstant(3, 3, 0);

            Araxis(0,0) = 1./std::abs(lcs->xRot().stddev());
            Araxis(1,1) = 1./std::abs(lcs->yRot().stddev());
            Araxis(2,2) = 1./std::abs(lcs->zRot().stddev());

            ceres::Vector braxis;
            braxis.resize(3);

            braxis << raxis_prior[0],raxis_prior[1], raxis_prior[2];

            ceres::NormalPrior* rAxisPrior = new ceres::NormalPrior(Araxis, (braxis));
            problem.AddResidualBlock(rAxisPrior, nullptr, lcsPoseNode->rAxis.data());

        }

        //points constraints

        qint64 trajId = lcsPoseNode->trajectoryId.value_or(-1);

        ModularSBASolver::TrajectoryNode* trajNode = solver->getNodeForTrajectory(trajId, dontCreateIfMissing);

        for (qint64 lmId : lcs->listTypedSubDataBlocks(LandmarkLocalCoordinates::staticMetaObject.className())) {

            LandmarkLocalCoordinates* lmlc = lcs->getLandmarkLocalCoordinates(lmId);

            if (lmlc == nullptr) {
                continue;
            }

            if (!lmlc->xCoord().isSet() or !lmlc->yCoord().isSet() or !lmlc->zCoord().isSet()) {
                continue;
            }

            ModularSBASolver::PositionNode* l_v = solver->getNodeForLandmark(lmlc->attachedLandmarkid(), dontCreateIfMissing);

            if (l_v == nullptr) {
                continue;
            }

            Eigen::Vector3d localPos;
            localPos.x() = lmlc->xCoord().value();
            localPos.y() = lmlc->yCoord().value();
            localPos.z() = lmlc->zCoord().value();

            Eigen::Matrix3d stiffness = Eigen::Matrix3d::Identity();

            if (lmlc->xCoord().isUncertain()) {
                stiffness(0,0) = 1./std::abs(lmlc->xCoord().stddev());
            } else {
                stiffness(0,0) = 1e6;
            }

            if (lmlc->yCoord().isUncertain()) {
                stiffness(1,1) = 1./std::abs(lmlc->yCoord().stddev());
            } else {
                stiffness(1,1) = 1e6;
            }

            if (lmlc->zCoord().isUncertain()) {
                stiffness(2,2) = 1./std::abs(lmlc->zCoord().stddev());
            } else {
                stiffness(2,2) = 1e6;
            }

            if (trajNode == nullptr) {

                using CostFuncT = ceres::AutoDiffCostFunction<LocalPointAlignementCost,3,3,3,3>;

                LocalPointAlignementCost* localAlignementCost =
                        new LocalPointAlignementCost(localPos, stiffness);

                CostFuncT* costFunc = new CostFuncT(localAlignementCost);

                ModularSBASolver::AutoErrorBlockLogger<3,3>::ParamsType params =
                {l_v->pos.data(), lcsPoseNode->rAxis.data(), lcsPoseNode->t.data()};

                problem.AddResidualBlock(costFunc, nullptr,
                                         params.data(),
                                         params.size());

                QString loggerName = QString("Local system \"%1\" landmark %2 pos constaint").arg(lcs->objectName()).arg(lmlc->attachedLandmarkName());
                solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<3,3>(costFunc, params));

            } else {

                double t = lmlc->time();

                int trajNodeId = trajNode->getNodeForTime(t);

                if (trajNodeId < 0 or trajNodeId >= trajNode->nodes.size()) {

                    Trajectory* traj = currentProject->getDataBlock<Trajectory>(trajId);
                    QString trajName = "undefined";
                    if (traj != nullptr) {
                        trajName = traj->objectName();
                    }

                    QString message = QObject::tr("[Warning] Could not get timing for local coordinate system %1 (\"%2\"), bil sequence %3 (\"%4\"), landmark %5 (\"%6\"), skipping!")
                            .arg(lcsId)
                            .arg(lcs->objectName())
                            .arg(trajId)
                            .arg(trajName)
                            .arg(lmlc->attachedLandmarkid())
                            .arg(lmlc->attachedLandmarkName());

                    solver->logMessage(message);
                    continue;
                }


                StereoVisionApp::ModularSBASolver::TrajectoryPoseNode& previousPose = trajNode->nodes[trajNodeId];
                StereoVisionApp::ModularSBASolver::TrajectoryPoseNode& nextPose = trajNode->nodes[trajNodeId+1];

                double w1 = (nextPose.time - t)/(nextPose.time - previousPose.time);
                double w2 = (t - previousPose.time)/(nextPose.time - previousPose.time);

                LocalInterpRelativePointAlignementCost* cost =
                        new LocalInterpRelativePointAlignementCost(localPos, w1, w2, stiffness);

                using CostFuncT = ceres::AutoDiffCostFunction<LocalInterpRelativePointAlignementCost,3,3,3,3,3,3,3,3>;
                CostFuncT* costFunc = new CostFuncT(cost);

                ModularSBASolver::AutoErrorBlockLogger<7,3>::ParamsType params =
                {l_v->pos.data(),
                 previousPose.rAxis.data(),
                 previousPose.t.data(),
                 nextPose.rAxis.data(),
                 nextPose.t.data(),
                 lcsPoseNode->rAxis.data(),
                 lcsPoseNode->t.data()};

                problem.AddResidualBlock(costFunc, nullptr,
                                         params.data(),
                                         params.size());

                QString loggerName =
                        QString("Local system \"%1\" landmark %2 interpolated relative pos constaint").arg(lcs->objectName()).arg(lmlc->attachedLandmarkName());
                solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<7,3>(costFunc, params));

            }
        }

    }

    return true;

}


bool LocalCoordinateSystemSBAModule::writeResults(ModularSBASolver* solver) {

    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> lcsIdxs = currentProject->getIdsByClass(LocalCoordinateSystem::staticMetaObject.className());

    for (qint64 id : lcsIdxs) {

        ModularSBASolver::PoseNode* lcr_p = solver->getPoseNode(id);

        if (lcr_p == nullptr) {
            continue;
        }

        LocalCoordinateSystem* lcs = currentProject->getDataBlock<LocalCoordinateSystem>(id);

        if (lcs == nullptr) {
            continue;
        }

        if (lcs->isFixed()) {
            continue;
        }

        //TODO: add georeferencing support
        floatParameterGroup<3> pos;
        pos.value(0) = static_cast<float>(lcr_p->t[0]);
        pos.value(1) = static_cast<float>(lcr_p->t[1]);
        pos.value(2) = static_cast<float>(lcr_p->t[1]);
        pos.setIsSet();
        lcs->setOptPos(pos);

        floatParameterGroup<3> rot;
        rot.value(0) = static_cast<float>(lcr_p->rAxis[0]);
        rot.value(1) = static_cast<float>(lcr_p->rAxis[1]);
        rot.value(2) = static_cast<float>(lcr_p->rAxis[2]);
        rot.setIsSet();
        lcs->setOptRot(rot);

    }

    return true;
}
bool LocalCoordinateSystemSBAModule::writeUncertainty(ModularSBASolver* solver) {
    //Basic data structures are managed by the modular sba solver directly.
    return true;
}
void LocalCoordinateSystemSBAModule::cleanup(ModularSBASolver* solver) {
    //Basic data structures are managed by the modular sba solver directly.
    Q_UNUSED(solver);
    return;
}

} // namespace StereoVisionApp
