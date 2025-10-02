#include "correspondencessetsbamodule.h"

#include "datablocks/trajectory.h"
#include "datablocks/landmark.h"
#include "datablocks/localcoordinatesystem.h"
#include "datablocks/image.h"

#include "costfunctors/localpointalignementcost.h"
#include "costfunctors/local3dcoalignementcost.h"
#include "costfunctors/localpointprojectioncost.h"
#include "costfunctors/modularuvprojection.h"
#include "costfunctors/posedecoratorfunctors.h"

#include "costfunctors/fixedsizenormalprior.h"

namespace StereoVisionApp {

const char* CorrespondencesSetSBAModule::ModuleName = "SBAModule::CorrespondenceSet";

CorrespondencesSetSBAModule::CorrespondencesSetSBAModule()
{

}

QString CorrespondencesSetSBAModule::moduleName() const {
    return QObject::tr("Correspondences Set SBA Module");
}

bool CorrespondencesSetSBAModule::addGraphReductorVariables(Project *currentProject, GenericSBAGraphReductor* graphReductor) {

    Q_UNUSED(currentProject);
    Q_UNUSED(graphReductor);
    return true;
}

bool CorrespondencesSetSBAModule::addGraphReductorObservations(Project *currentProject, GenericSBAGraphReductor* graphReductor) {

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> lcsIdxs = currentProject->getIdsByClass(LocalCoordinateSystem::staticMetaObject.className());

    QVector<qint64> correspSetsIdxs = currentProject->getIdsByClass(CorrespondencesSet::staticMetaObject.className());

    for (qint64 correspSetId : correspSetsIdxs) {

        StereoVisionApp::CorrespondencesSet* correspSet = currentProject->getDataBlock<StereoVisionApp::CorrespondencesSet>(correspSetId);

        if (correspSet == nullptr) {
            continue;
        }

        using GenericPair = Correspondences::GenericPair;
        using GenericMatch = Correspondences::Generic;


        for (int i = 0; i < correspSet->nCorrespondence(); i++) {

            StereoVisionApp::Correspondences::GenericPair const& pair = correspSet->getCorrespondence(i);

            if (pair.holdsCorrespondancesType<Correspondences::PRIOR,Correspondences::XYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::PRIOR,Correspondences::XYZ>().value();
                graphReductor->insertSelfObservation(typedPair.c2.blockId,3);
            }

            if (pair.holdsCorrespondancesType<Correspondences::PRIORID,Correspondences::GEOXYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::PRIORID,Correspondences::GEOXYZ>().value();
                graphReductor->insertSelfObservation(typedPair.c1.blockId,3);
            }

            if (pair.holdsCorrespondancesType<Correspondences::XYZ,Correspondences::XYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::XYZ,Correspondences::XYZ>().value();
                graphReductor->insertObservation(typedPair.c1.blockId, typedPair.c2.blockId, 3);
            }

            if (pair.holdsCorrespondancesType<Correspondences::XYZ,Correspondences::Line3D>()) {

                auto typedPair = pair.getTypedPair<Correspondences::XYZ,Correspondences::Line3D>().value();
                //Do nothing, as this is not supported yet!
                //TODO: support line constaints
            }

            if (pair.holdsCorrespondancesType<Correspondences::XYZ,Correspondences::Plane3D>()) {

                auto typedPair = pair.getTypedPair<Correspondences::XYZ,Correspondences::Plane3D>().value();
                //Do nothing, as this is not supported yet!
                //TODO: support line constaints
            }

            if (pair.holdsCorrespondancesType<Correspondences::XYZ,Correspondences::GEOXYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::XYZ,Correspondences::GEOXYZ>().value();
                graphReductor->insertSelfObservation(typedPair.c1.blockId,3);
            }

            if (pair.holdsCorrespondancesType<Correspondences::XYZ,Correspondences::PRIORID>()) {

                auto typedPair = pair.getTypedPair<Correspondences::XYZ,Correspondences::PRIORID>().value();
                graphReductor->insertObservation(typedPair.c1.blockId, typedPair.c2.blockId, 3);
            }

            //image correspondences

            if (pair.holdsCorrespondancesType<Correspondences::UV,Correspondences::UV>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UV,Correspondences::UV>().value();
                graphReductor->insertObservation(typedPair.c1.blockId, typedPair.c2.blockId, 2);
            }

            if (pair.holdsCorrespondancesType<Correspondences::UV,Correspondences::XYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UV,Correspondences::XYZ>().value();
                graphReductor->insertObservation(typedPair.c1.blockId, typedPair.c2.blockId, 2);
            }

            if (pair.holdsCorrespondancesType<Correspondences::UV,Correspondences::XYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UV,Correspondences::XYZ>().value();
                graphReductor->insertObservation(typedPair.c1.blockId, typedPair.c2.blockId, 2);
            }

            if (pair.holdsCorrespondancesType<Correspondences::UV,Correspondences::GEOXYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UV,Correspondences::GEOXYZ>().value();
                graphReductor->insertSelfObservation(typedPair.c1.blockId, 2);
            }

            //timed image correspondences

            if (pair.holdsCorrespondancesType<Correspondences::UVT,Correspondences::UV>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UVT,Correspondences::UV>().value();
                graphReductor->insertObservation(typedPair.c1.blockId, typedPair.c2.blockId, 2);
            }

            if (pair.holdsCorrespondancesType<Correspondences::UVT,Correspondences::UVT>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UVT,Correspondences::UVT>().value();
                graphReductor->insertObservation(typedPair.c1.blockId, typedPair.c2.blockId, 2);
            }

            if (pair.holdsCorrespondancesType<Correspondences::UVT,Correspondences::XYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UVT,Correspondences::XYZ>().value();
                graphReductor->insertObservation(typedPair.c1.blockId, typedPair.c2.blockId, 2);
            }

            if (pair.holdsCorrespondancesType<Correspondences::UVT,Correspondences::XYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UVT,Correspondences::XYZ>().value();
                graphReductor->insertObservation(typedPair.c1.blockId, typedPair.c2.blockId, 2);
            }

            if (pair.holdsCorrespondancesType<Correspondences::UVT,Correspondences::GEOXYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UVT,Correspondences::GEOXYZ>().value();
                graphReductor->insertSelfObservation(typedPair.c1.blockId, 2);
            }
        }

    }

    return true;

}

bool CorrespondencesSetSBAModule::setupParameters(ModularSBASolver* solver) {
    Q_UNUSED(solver);
    return true;
}

bool CorrespondencesSetSBAModule::addGeoPosPrior(Correspondences::Typed<Correspondences::PRIORID> const& priorId,
                              Correspondences::Typed<Correspondences::GEOXYZ> const& geoPos,
                              StereoVisionApp::ModularSBASolver* solver,
                              ceres::Problem & problem) {

    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    double* posData = nullptr;

    ModularSBASolver::PoseNode* p = solver->getPoseNode(priorId.blockId);

    if (p != nullptr) {

        qint64 trajId = p->trajectoryId.value_or(-1);

        if (trajId >= 0 and !p->time.has_value()) {
            return false; //cannot have a geo pos prior for items on a trajectory, need a XYZT match
        }

        posData = p->t.data();
    } else {

        ModularSBASolver::PositionNode* p = solver->getPositionNode(priorId.blockId);

        if (p != nullptr) {
            posData = p->pos.data();
        }
    }

    if (posData == nullptr) {
        return false;
    }

    std::optional<Eigen::Vector3d> pointPos =
            Correspondences::getGeoXYZConstraintInfos(geoPos, solver->getTransform2LocalFrame());

    if (!pointPos.has_value()) {
        return false;
    }

    Eigen::Matrix3d stiffness = Eigen::Matrix3d::Identity();

    if (geoPos.sigmaX.has_value() and
            geoPos.sigmaY.has_value() and
            geoPos.sigmaZ.has_value()) {
        stiffness(0,0) = 1/(geoPos.sigmaX.value());
        stiffness(1,1) = 1/(geoPos.sigmaY.value());
        stiffness(2,2) = 1/(geoPos.sigmaZ.value());
    }

    FixedSizeNormalPrior<3,3>* costFunc =
        new FixedSizeNormalPrior<3,3>(stiffness, pointPos.value());

    problem.AddResidualBlock(costFunc, nullptr, posData);

    return true;

}



bool CorrespondencesSetSBAModule::addGeoProjPrior(Correspondences::Typed<Correspondences::PRIORID> const& priorId,
                           Correspondences::Typed<Correspondences::GEOXY> const& geoPos,
                           ModularSBASolver* solver,
                           ceres::Problem & problem) {

    Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    DataBlock* targetBlock = currentProject->getById(priorId.blockId);

    if (targetBlock == nullptr) {
        return false;
    }

    double* posData = nullptr;

    ModularSBASolver::PoseNode* p = solver->getPoseNode(priorId.blockId);

    if (p != nullptr) {

        qint64 trajId = p->trajectoryId.value_or(-1);

        if (trajId >= 0 and !p->time.has_value()) {
            return false; //cannot have a geo pos prior for items on a trajectory, need a XYZT match
        }

        posData = p->t.data();
    } else {

        ModularSBASolver::PositionNode* p = solver->getPositionNode(priorId.blockId);

        if (p != nullptr) {
            posData = p->pos.data();
        }
    }

    if (posData == nullptr) {
        return false;
    }

    std::optional<std::tuple<Eigen::Matrix<double,2,3>,Eigen::Vector2d>> projInfos =
            Correspondences::getGeoXYConstraintInfos(geoPos, solver->getTransform2LocalFrame());

    if (!projInfos.has_value()) {
        return false;
    }

    Eigen::Matrix2d stiffness = Eigen::Matrix2d::Identity();

    if (geoPos.sigmaX.has_value() and
            geoPos.sigmaY.has_value()) {
        stiffness(0,0) = 1/(geoPos.sigmaX.value());
        stiffness(1,1) = 1/(geoPos.sigmaY.value());
    }

    Eigen::Matrix<double,2,3> A = std::get<Eigen::Matrix<double,2,3>>(projInfos.value());
    Eigen::Matrix<double,2,3> M = stiffness*A;
    Eigen::Vector2d b = std::get<Eigen::Vector2d>(projInfos.value());
    Eigen::Vector3d x = A.fullPivHouseholderQr().solve(b);

    Eigen::Vector2d error = A*x - b;
    double errorNorm = error.norm();

    if (errorNorm > 1e-3) {
        solver->logMessage(QString("Reprojection matrix ill conditionned for correspondence %1,%2").arg(priorId.toStr(),geoPos.toStr()));
        return false;
    }

    FixedSizeNormalPrior<3,2>* costFunc =
            new FixedSizeNormalPrior<3,2>(M, x);

    FixedSizeNormalPrior<3,2>* errorFunc =
            new FixedSizeNormalPrior<3,2>(A, x);

    QString loggerName = QString("Geo projection constraint for %1 (id = %2, class = %3)")
            .arg(targetBlock->objectName())
            .arg(targetBlock->internalId())
            .arg(targetBlock->metaObject()->className());

    constexpr bool  manageFunc = true;
    solver->addLogger(loggerName,
                      new ModularSBASolver::AutoErrorBlockLogger<1,2>(errorFunc, {posData}, manageFunc));
    problem.AddResidualBlock(costFunc, nullptr, posData);

    return true;

}

bool CorrespondencesSetSBAModule::addXYZMatch(Correspondences::Typed<Correspondences::XYZ> const& xyz1,
                           Correspondences::Typed<Correspondences::XYZ> const& xyz2,
                           StereoVisionApp::ModularSBASolver* solver,
                           ceres::Problem & problem) {

    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    ModularSBASolver::PoseNode* p1 = solver->getPoseNode(xyz1.blockId);
    ModularSBASolver::PoseNode* p2 = solver->getPoseNode(xyz2.blockId);

    if (p1 == nullptr or p2 == nullptr) {
        return false;
    }

    qint64 traj1id = p1->trajectoryId.value_or(-1);
    qint64 traj2id = p2->trajectoryId.value_or(-1);

    if ((traj1id >= 0 and !p1->time.has_value()) or
            (traj2id >= 0 and !p2->time.has_value())) {
        return false; //cannot have just a plain XYZ match for items following a trajectory, need a XYZT match
    }

    Eigen::Vector3d localPos1;
    localPos1 << xyz1.x, xyz1.y, xyz1.z;
    Eigen::Vector3d localPos2;
    localPos2 << xyz2.x, xyz2.y, xyz2.z;

    Eigen::Matrix3d info = Eigen::Matrix3d::Zero();

    if (xyz1.sigmaX.has_value() and xyz1.sigmaY.has_value() and xyz1.sigmaZ.has_value()) {

        if (xyz2.sigmaX.has_value() and xyz2.sigmaY.has_value() and xyz2.sigmaZ.has_value()) {
            info(0,0) = 1/(xyz1.sigmaX.value() + xyz2.sigmaX.value());
            info(1,1) = 1/(xyz1.sigmaY.value() + xyz2.sigmaY.value());
            info(2,2) = 1/(xyz1.sigmaZ.value() + xyz2.sigmaZ.value());
        } else {
            info(0,0) = 1/(xyz1.sigmaX.value());
            info(1,1) = 1/(xyz1.sigmaY.value());
            info(2,2) = 1/(xyz1.sigmaZ.value());
        }

    } else if (xyz2.sigmaX.has_value() and xyz2.sigmaY.has_value() and xyz2.sigmaZ.has_value()) {

        info(0,0) = 1/(xyz2.sigmaX.value());
        info(1,1) = 1/(xyz2.sigmaY.value());
        info(2,2) = 1/(xyz2.sigmaZ.value());

    } else {
        info = Eigen::Matrix3d::Identity();
    }

    Local3DCoalignementCost* cost =
            new Local3DCoalignementCost(localPos1, localPos2, info);

    using CostFuncT = ceres::AutoDiffCostFunction<Local3DCoalignementCost,3,3,3,3,3>;

    CostFuncT* costFunc = new CostFuncT(cost);

    problem.AddResidualBlock(costFunc, nullptr, p1->rAxis.data(), p1->t.data(), p2->rAxis.data(), p2->t.data());

    return true;

}

bool CorrespondencesSetSBAModule::addXYZ2LineMatch(Correspondences::Typed<Correspondences::XYZ> const& xyz,
                                Correspondences::Typed<Correspondences::Line3D> const& line,
                                StereoVisionApp::ModularSBASolver* solver,
                                ceres::Problem & problem) {
    //TODO: add constraint
    solver->logMessage(QObject::tr("Tried to add a correspondance from a point in local coordinate system to a line in a local coordinate system, but the operation is not yet supported"));
    return false;
}

bool CorrespondencesSetSBAModule::addXYZ2PlaneMatch(Correspondences::Typed<Correspondences::XYZ> const& xyz,
                                 Correspondences::Typed<Correspondences::Plane3D> const& line,
                                 StereoVisionApp::ModularSBASolver* solver,
                                 ceres::Problem & problem) {
    //TODO: add constraint
    solver->logMessage(QObject::tr("Tried to add a correspondance from a point in local coordinate system to a plane in a local coordinate system, but the operation is not yet supported"));
    return false;
}

bool CorrespondencesSetSBAModule::addXYZ2GeoMatch(Correspondences::Typed<Correspondences::XYZ> const& xyz,
                               Correspondences::Typed<Correspondences::GEOXYZ> const& geoMatch,
                               StereoVisionApp::ModularSBASolver* solver,
                               ceres::Problem & problem) {

    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    ModularSBASolver::PoseNode* p = solver->getPoseNode(xyz.blockId);

    if (p == nullptr) {
        return false;
    }

    qint64 trajId = p->trajectoryId.value_or(-1);

    if (trajId >= 0 and !p->time.has_value()) {
        return false; //cannot have a geo pos prior for lcs on a trajectory, need a XYZT match
    }

    std::optional<Eigen::Vector3d> pointPos =
            Correspondences::getGeoXYZConstraintInfos(geoMatch, solver->getTransform2LocalFrame());

    if (!pointPos.has_value()) {
        return false;
    }

    Eigen::Vector3d localPos;
    localPos << xyz.x, xyz.y, xyz.z;

    Eigen::Matrix3d stiffness = Eigen::Matrix3d::Zero();

    if (xyz.sigmaX.has_value() and xyz.sigmaY.has_value() and xyz.sigmaZ.has_value()) {

        if (geoMatch.sigmaX.has_value() and geoMatch.sigmaY.has_value() and geoMatch.sigmaZ.has_value()) {
            stiffness(0,0) = 1/(xyz.sigmaX.value() + geoMatch.sigmaX.value());
            stiffness(1,1) = 1/(xyz.sigmaY.value() + geoMatch.sigmaY.value());
            stiffness(2,2) = 1/(xyz.sigmaZ.value() + geoMatch.sigmaZ.value());
        } else {
            stiffness(0,0) = 1/(xyz.sigmaX.value());
            stiffness(1,1) = 1/(xyz.sigmaY.value());
            stiffness(2,2) = 1/(xyz.sigmaZ.value());
        }

    } else if (geoMatch.sigmaX.has_value() and geoMatch.sigmaY.has_value() and geoMatch.sigmaZ.has_value()) {

        stiffness(0,0) = 1/(geoMatch.sigmaX.value());
        stiffness(1,1) = 1/(geoMatch.sigmaY.value());
        stiffness(2,2) = 1/(geoMatch.sigmaZ.value());

    } else {
        stiffness = Eigen::Matrix3d::Identity();
    }

    LocalPoint2TargetAlignementCost* cost =
            new LocalPoint2TargetAlignementCost(localPos, pointPos.value(), stiffness);

    using CostFuncT = ceres::AutoDiffCostFunction<LocalPoint2TargetAlignementCost,3,3,3>;

    CostFuncT* costFunc = new CostFuncT(cost);

    problem.AddResidualBlock(costFunc, nullptr, p->rAxis.data(), p->t.data());

    return true;

}

bool CorrespondencesSetSBAModule::addXYZ2GeoMatch(Correspondences::Typed<Correspondences::XYZ> const& xyz,
                            Correspondences::Typed<Correspondences::GEOXY> const& geoMatch,
                            StereoVisionApp::ModularSBASolver* solver,
                            ceres::Problem & problem) {

    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    ModularSBASolver::PoseNode* p = solver->getPoseNode(xyz.blockId);

    if (p == nullptr) {
        return false;
    }

    qint64 trajId = p->trajectoryId.value_or(-1);

    if (trajId >= 0 and !p->time.has_value()) {
        return false; //cannot have a geo pos prior for lcs on a trajectory, need a XYZT match
    }

    std::optional<std::tuple<Eigen::Matrix<double,2,3>,Eigen::Vector2d>> projInfos =
            Correspondences::getGeoXYConstraintInfos(geoMatch, solver->getTransform2LocalFrame());

    if (!projInfos.has_value()) {
        return false;
    }

    Eigen::Matrix<double,2,3> A = std::get<Eigen::Matrix<double,2,3>>(projInfos.value());
    Eigen::Vector2d b = std::get<Eigen::Vector2d>(projInfos.value());

    Eigen::Vector3d localPos;
    localPos << xyz.x, xyz.y, xyz.z;

    Eigen::Matrix2d stiffness = Eigen::Matrix2d::Zero();

    if (xyz.sigmaX.has_value() and xyz.sigmaY.has_value() and xyz.sigmaZ.has_value()) {

        Eigen::Vector3d xyz_sigma;
        xyz_sigma << xyz.sigmaX.value(), xyz.sigmaY.value(), xyz.sigmaZ.value();

        Eigen::Vector2d projSigma = A*xyz_sigma;

        if (geoMatch.sigmaX.has_value() and geoMatch.sigmaY.has_value()) {
            stiffness(0,0) = 1/(xyz.sigmaX.value() + projSigma.x());
            stiffness(1,1) = 1/(xyz.sigmaY.value() + projSigma.y());
        } else {
            stiffness(0,0) = 1/(projSigma.x());
            stiffness(1,1) = 1/(projSigma.y());
        }

    } else if (geoMatch.sigmaX.has_value() and geoMatch.sigmaY.has_value()) {

        stiffness(0,0) = 1/(geoMatch.sigmaX.value());
        stiffness(1,1) = 1/(geoMatch.sigmaY.value());

    } else {
        stiffness = Eigen::Matrix2d::Identity();
    }

    LocalPoint2TargetProjectionCost<2>* cost =
            new LocalPoint2TargetProjectionCost<2>(localPos, b, A, stiffness);

    using CostFuncT = ceres::AutoDiffCostFunction<LocalPoint2TargetProjectionCost<2>,3,3,3>;

    CostFuncT* costFunc = new CostFuncT(cost);

    problem.AddResidualBlock(costFunc, nullptr, p->rAxis.data(), p->t.data());

    return true;

}

bool CorrespondencesSetSBAModule::setupXYZPrior(Correspondences::Typed<Correspondences::XYZ> const& xyz,
                             StereoVisionApp::ModularSBASolver* solver,
                             ceres::Problem & problem) {

    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    double* targetData = nullptr;

    ModularSBASolver::PoseNode* pose = solver->getPoseNode(xyz.blockId);
    ModularSBASolver::PositionNode* position = solver->getPositionNode(xyz.blockId);

    if (pose != nullptr) {

        qint64 trajid = pose->trajectoryId.value_or(-1);

        if (trajid >= 0 and !pose->time.has_value()) {
            return false; //cannot have just a plain XYZ match for lcs on a trajectory, need a XYZT match
        }

        targetData = pose->t.data();

    } else if (position != nullptr) {
        targetData = position->pos.data();
    }

    if (targetData == nullptr) {
        return false;
    }

    Eigen::Vector3d localPos;
    localPos << xyz.x, xyz.y, xyz.z;

    Eigen::Matrix3d stiffness = Eigen::Matrix3d::Identity();

    if (xyz.sigmaX.has_value() and
            xyz.sigmaY.has_value() and
            xyz.sigmaZ.has_value()) {

        stiffness(0,0) = 1/(xyz.sigmaX.value());
        stiffness(1,1) = 1/(xyz.sigmaY.value());
        stiffness(2,2) = 1/(xyz.sigmaZ.value());
    }

    FixedSizeNormalPrior<3,3>* costFunc =
            new FixedSizeNormalPrior<3,3>(stiffness, localPos);

    problem.AddResidualBlock(costFunc, nullptr, targetData);

    return true;

}

bool CorrespondencesSetSBAModule::addXYZ2PriorMatch(Correspondences::Typed<Correspondences::XYZ> const& xyz,
                                 Correspondences::Typed<Correspondences::PRIORID> const& priorId,
                                 StereoVisionApp::ModularSBASolver* solver,
                                 ceres::Problem & problem) {

    ModularSBASolver::PoseNode* p1 = solver->getPoseNode(xyz.blockId);
    double* targetPosBuffer = nullptr;

    if (p1 == nullptr) {
        return false;
    }

    qint64 traj1id = p1->trajectoryId.value_or(-1);

    if (traj1id >= 0) {
        return false; //cannot have just a plain XYZ match for item following a trajectory, need a XYZT match
    }

    ModularSBASolver::PositionNode* pointNode = solver->getPositionNode(priorId.blockId);
    ModularSBASolver::PoseNode* poseNode = solver->getPoseNode(priorId.blockId);

    if (pointNode != nullptr) {

        targetPosBuffer = pointNode->pos.data();

    } else if (poseNode != nullptr) {

        qint64 traj2id = poseNode->trajectoryId.value_or(-1);

        if (traj2id >= 0 and !poseNode->time.has_value()) {
            return false; //cannot have just a plain XYZ match for item following a trajectory, need a XYZT match
        }

        targetPosBuffer = poseNode->t.data();

    }

    if (targetPosBuffer == nullptr) {
        return false;
    }

    Eigen::Vector3d localPos;
    localPos << xyz.x, xyz.y, xyz.z;

    Eigen::Matrix3d stiffness = Eigen::Matrix3d::Identity();

    if (xyz.sigmaX.has_value() and
            xyz.sigmaY.has_value() and
            xyz.sigmaZ.has_value()) {
        stiffness(0,0) = 1/(xyz.sigmaX.value());
        stiffness(1,1) = 1/(xyz.sigmaY.value());
        stiffness(2,2) = 1/(xyz.sigmaZ.value());
    }

    LocalPointAlignementCost* cost =
            new LocalPointAlignementCost(localPos, stiffness);

    using CostFuncT = ceres::AutoDiffCostFunction<LocalPointAlignementCost,3,3,3,3>;

    CostFuncT* costFunc = new CostFuncT(cost);

    problem.AddResidualBlock(costFunc, nullptr, targetPosBuffer, p1->rAxis.data(), p1->t.data());

    return true;
}

bool CorrespondencesSetSBAModule::addUV2UVMatch(const Correspondences::Typed<Correspondences::UV> &uv1,
                                                const Correspondences::Typed<Correspondences::UV> &uv2,
                                                StereoVisionApp::ModularSBASolver* solver,
                                                ceres::Problem & problem) {

    ModularSBASolver::PoseNode* p1 = solver->getPoseNode(uv1.blockId);
    ModularSBASolver::PoseNode* p2 = solver->getPoseNode(uv2.blockId);

    if (p1 == nullptr or p2 == nullptr) {
        return false;
    }

    if (p1 == p2) { //we cannot project an image onto itself (maybe we should, e.g. projectors seeing the same stuff twice)
        return false;
    }

    ModularSBASolver::ProjectorModule* m1 = solver->getProjectorForFrame(uv1.blockId);
    ModularSBASolver::ProjectorModule* m2 = solver->getProjectorForFrame(uv2.blockId);

    if (m1 == nullptr or m2 == nullptr) {
        return false;
    }

    Eigen::Vector2d pos1;
    pos1 << uv1.u, uv1.v;

    Eigen::Vector2d pos2;
    pos1 << uv2.u, uv2.v;

    Eigen::Matrix2d stiffness1 = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d stiffness2 = Eigen::Matrix2d::Identity();

    ModularSBASolver::ProjectorModule::addCrossProjectionCostFunction(m1, p1->rAxis.data(), p1->t.data(), pos1, stiffness1,
                                                                      m2, p2->rAxis.data(), p2->t.data(), pos2, stiffness2);

    return true;

}



bool CorrespondencesSetSBAModule::addUV2XYZMatch(Correspondences::Typed<Correspondences::UV> const& uv,
                           Correspondences::Typed<Correspondences::XYZ> const& xyz,
                           StereoVisionApp::ModularSBASolver* solver,
                           ceres::Problem & problem) {

    ModularSBASolver::PoseNode* pIm = solver->getPoseNode(uv.blockId);

    if (pIm == nullptr) {
        return false;
    }

    ModularSBASolver::ProjectorModule* mIm = solver->getProjectorForFrame(uv.blockId);

    if (mIm == nullptr) {
        return false;
    }

    ModularSBASolver::ProjectorModule::ProjectionInfos infosIm = mIm->getProjectionInfos();

    if (infosIm.modularProjector == nullptr) {
        return false;
    }

    ModularSBASolver::PoseNode* poseNode = solver->getPoseNode(xyz.blockId);

    if (poseNode == nullptr) { //the xyz data is not for a specific data point, but the world
        return false;
    }

    Eigen::Vector2d uvPos;
    uvPos << uv.u, uv.v;

    Eigen::Vector3d ptPos;
    ptPos << xyz.x, xyz.y, xyz.z;

    Eigen::Matrix2d stiffness = Eigen::Matrix2d::Identity();
    if (uv.sigmaU.has_value()) {
        stiffness(0,0) = 1/ uv.sigmaU.value();
    }
    if (uv.sigmaV.has_value()) {
        stiffness(1,1) = 1/ uv.sigmaV.value();
    }

    using Functor = UV2ParametrizedXYZCostDynamic<ModularUVProjection>;
    constexpr DecoratorPoseConfiguration poseOrder = DecoratorPoseConfiguration::Body2World;
    using DecoratedFunctor = InvertPoseDynamic<LocalFrame2MappedPointDynamic<Functor, poseOrder, Functor::lmPosParamId>>; //ensure the pose are given as world2sensor (the parameters are as sensor2world)
    constexpr int stride = 4;
    using CostFunction = ceres::DynamicAutoDiffCostFunction<DecoratedFunctor, stride>;


    DecoratedFunctor* functor = new DecoratedFunctor(ptPos, infosIm.modularProjector, uvPos, stiffness, infosIm.paramsSizeInfos.size());
    CostFunction* costFunction = new CostFunction(functor);

    int nParams = 4 + infosIm.paramsSizeInfos.size();
    std::vector<double*> params(nParams);

    params[0] = pIm->rAxis.data();
    costFunction->AddParameterBlock(3);
    params[1] = pIm->t.data();
    costFunction->AddParameterBlock(3);
    params[2] = poseNode->rAxis.data();
    costFunction->AddParameterBlock(3);
    params[3] = poseNode->t.data();
    costFunction->AddParameterBlock(3);

    for (int i = 0; i < infosIm.paramsSizeInfos.size(); i++) {
        params[4+i] = infosIm.projectionParams[i];
        costFunction->AddParameterBlock(infosIm.paramsSizeInfos[i]);
    }

    costFunction->SetNumResiduals(Functor::nResiduals);

    mIm->problem().AddResidualBlock(costFunction, nullptr,
                                        params.data(), nParams);

    return true;

}

bool CorrespondencesSetSBAModule::addUV2GeoXYZMatch(Correspondences::Typed<Correspondences::UV> const& uv,
                                                    Correspondences::Typed<Correspondences::GEOXYZ> const& geoMatch,
                                                    StereoVisionApp::ModularSBASolver* solver,
                                                    ceres::Problem & problem) {

    ModularSBASolver::PoseNode* pIm = solver->getPoseNode(uv.blockId);

    if (pIm == nullptr) {
        return false;
    }

    ModularSBASolver::ProjectorModule* mIm = solver->getProjectorForFrame(uv.blockId);

    if (mIm == nullptr) {
        return false;
    }

    ModularSBASolver::ProjectorModule::ProjectionInfos infosIm = mIm->getProjectionInfos();

    if (infosIm.modularProjector == nullptr) {
        return false;
    }

    double* targetPosBuffer = nullptr;

    std::optional<Eigen::Vector3d> lmPos =
        Correspondences::getGeoXYZConstraintInfos(geoMatch, solver->getTransform2LocalFrame());

    if (!lmPos.has_value()) { //the xyz data is not for a specific data point, but the world
        return false;
    }

    Eigen::Vector2d uvPos;
    uvPos << uv.u, uv.v;

    Eigen::Vector3d ptPos = lmPos.value();

    Eigen::Matrix2d stiffness = Eigen::Matrix2d::Identity();
    if (uv.sigmaU.has_value()) {
        stiffness(0,0) = 1/ uv.sigmaU.value();
    }
    if (uv.sigmaV.has_value()) {
        stiffness(1,1) = 1/ uv.sigmaV.value();
    }

    using Functor = UV2XYZCostDynamic<ModularUVProjection>;
    using DecoratedFunctor = InvertPoseDynamic<Functor>; //ensure the pose are given as world2sensor (the parameters are as sensor2world)
    constexpr int stride = 4;
    using CostFunction = ceres::DynamicAutoDiffCostFunction<DecoratedFunctor, stride>;


    DecoratedFunctor* functor = new DecoratedFunctor(infosIm.modularProjector, ptPos, uvPos, stiffness, infosIm.paramsSizeInfos.size());
    CostFunction* costFunction = new CostFunction(functor);

    int nParams = 4 + infosIm.paramsSizeInfos.size();
    std::vector<double*> params(nParams);

    params[0] = pIm->rAxis.data();
    costFunction->AddParameterBlock(3);
    params[1] = pIm->t.data();
    costFunction->AddParameterBlock(3);

    for (int i = 0; i < infosIm.paramsSizeInfos.size(); i++) {
        params[4+i] = infosIm.projectionParams[i];
        costFunction->AddParameterBlock(infosIm.paramsSizeInfos[i]);
    }

    costFunction->SetNumResiduals(Functor::nResiduals);

    mIm->problem().AddResidualBlock(costFunction, nullptr,
                                    params.data(), nParams);

    return true;
}

bool CorrespondencesSetSBAModule::addUV2XYZTMatch(Correspondences::Typed<Correspondences::UV> const& uv,
                                                  Correspondences::Typed<Correspondences::XYZT> const& xyzt,
                                                  StereoVisionApp::ModularSBASolver* solver,
                                                  ceres::Problem & problem) {

    ModularSBASolver::PoseNode* pIm = solver->getPoseNode(uv.blockId);

    if (pIm == nullptr) {
        return false;
    }

    ModularSBASolver::ProjectorModule* mIm = solver->getProjectorForFrame(uv.blockId);

    if (mIm == nullptr) {
        return false;
    }

    ModularSBASolver::ProjectorModule::ProjectionInfos infosIm = mIm->getProjectionInfos();

    if (infosIm.modularProjector == nullptr) {
        return false;
    }

    ModularSBASolver::ItemTrajectoryInfos itemTrajectoryInfos =
        solver->getItemTrajectoryInfos(xyzt.blockId);

    if (itemTrajectoryInfos.datablockId != xyzt.blockId) {
        return false;
    }

    bool createIfMissing = false;
    ModularSBASolver::TrajectoryNode* trajNode = solver->getNodeForTrajectory(itemTrajectoryInfos.TrajId, createIfMissing);

    if (trajNode == nullptr) { //the xyz data is not for a specific data point, but the world
        return false;
    }

    if (xyzt.t < trajNode->nodes[0].time or xyzt.t > trajNode->nodes.back().time) {
        return false; //outside of time range
    }

    size_t trajNodeId = trajNode->getNodeForTime(xyzt.t);
    size_t nextNodeId = trajNodeId+1;

    if (trajNodeId < 0 or nextNodeId >= trajNode->nodes.size()) {
        return false;
    }

    StereoVisionApp::ModularSBASolver::TrajectoryPoseNode& previousPose = trajNode->nodes[trajNodeId];
    StereoVisionApp::ModularSBASolver::TrajectoryPoseNode& nextPose = trajNode->nodes[nextNodeId];

    StereoVisionApp::ModularSBASolver::TrajectoryPoseNode& closest =
        (xyzt.t - previousPose.time < nextPose.time - xyzt.t) ? previousPose : nextPose;

    StereoVision::Geometry::RigidBodyTransform<double> measure2node =
        StereoVision::Geometry::RigidBodyTransform<double>(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (trajNode->initialTrajectory.nPoints() > 0) {

        auto nodeInitialPose = trajNode->initialTrajectory.getValueAtTime(closest.time);
        auto measureInitialPose = trajNode->initialTrajectory.getValueAtTime(xyzt.t);

        StereoVision::Geometry::RigidBodyTransform<double> node2worldInitial =
            StereoVision::Geometry::interpolateRigidBodyTransformOnManifold
            (nodeInitialPose.weigthLower, nodeInitialPose.valLower,
             nodeInitialPose.weigthUpper, nodeInitialPose.valUpper);

        StereoVision::Geometry::RigidBodyTransform<double> measure2worldInitial =
            StereoVision::Geometry::interpolateRigidBodyTransformOnManifold
            (measureInitialPose.weigthLower, measureInitialPose.valLower,
             measureInitialPose.weigthUpper, measureInitialPose.valUpper);

        StereoVision::Geometry::RigidBodyTransform<double> measure2node =
            node2worldInitial.inverse()*measure2worldInitial;

    }

    Eigen::Vector2d uvPos;
    uvPos << uv.u, uv.v;

    Eigen::Vector3d ptPos;
    ptPos << xyzt.x, xyzt.y, xyzt.z;

    Eigen::Matrix2d stiffness = Eigen::Matrix2d::Identity();
    if (uv.sigmaU.has_value()) {
        stiffness(0,0) = 1/ uv.sigmaU.value();
    }
    if (uv.sigmaV.has_value()) {
        stiffness(1,1) = 1/ uv.sigmaV.value();
    }

    double* leverArmR = nullptr;
    double* leverArmT = nullptr;

    ModularSBASolver::PoseNode* laNode = solver->getPoseNode(itemTrajectoryInfos.ExternalLeverArmId);

    if (laNode != nullptr) {
        leverArmR = laNode->rAxis.data();
        leverArmT = laNode->t.data();
    }

    using Functor = UV2ParametrizedXYZCostDynamic<ModularUVProjection>;
    constexpr DecoratorPoseConfiguration poseOrder = DecoratorPoseConfiguration::Body2World;
    using DecoratedFunctor = InvertPoseDynamic<LocalFrame2MappedPointDynamic<Functor, poseOrder, Functor::lmPosParamId>>; //ensure the pose are given as world2sensor (the parameters are as sensor2world)
    constexpr int stride = 4;

    constexpr int LeverArmConfiguration = Body2World|Body2Sensor;
    constexpr PoseTransformDirection poseTransformDirection = PoseTransformDirection::SourceToInitial;
    constexpr int poseParamIdx = 2;

    using CostBuildHelper = ModifiedPoseCostFunctionBuilderHelper<DecoratedFunctor, poseTransformDirection, LeverArmConfiguration, poseParamIdx, 2>;

    int nParams = 4 + infosIm.paramsSizeInfos.size();
    std::vector<double*> params(nParams);
    std::vector<int> paramsSizes(nParams);

    params[0] = pIm->rAxis.data();
    paramsSizes[0] = 3;
    params[1] = pIm->t.data();
    paramsSizes[1] = 3;
    params[2] = closest.rAxis.data();
    paramsSizes[2] = 3;
    params[3] = closest.t.data();
    paramsSizes[3] = 3;

    for (int i = 0; i < infosIm.paramsSizeInfos.size(); i++) {
        params[4+i] = infosIm.projectionParams[i];
        paramsSizes[4+i] = infosIm.paramsSizeInfos[i];
    }

    auto costFunctionData = CostBuildHelper::buildPoseShiftedDynamicCostFunction<stride>(params.data(),
                                                                                         paramsSizes,
                                                                                         measure2node,
                                                                                         leverArmR,
                                                                                         leverArmT,
                                                                                         ptPos,
                                                                                         infosIm.modularProjector,
                                                                                         uvPos,
                                                                                         stiffness,
                                                                                         infosIm.paramsSizeInfos.size());



    mIm->problem().AddResidualBlock(costFunctionData.costFunction,
                                    nullptr,
                                    costFunctionData.params.data(),
                                    costFunctionData.params.size());

    return true;
}

bool CorrespondencesSetSBAModule::addUVT2UVMatch(Correspondences::Typed<Correspondences::UVT> const& uv1,
                                                 Correspondences::Typed<Correspondences::UV> const& uv2,
                                                 StereoVisionApp::ModularSBASolver* solver,
                                                 ceres::Problem & problem) {
    return false;
}

bool CorrespondencesSetSBAModule::addUVT2UVTMatch(Correspondences::Typed<Correspondences::UVT> const& uv1,
                                                  Correspondences::Typed<Correspondences::UVT> const& uv2,
                                                  StereoVisionApp::ModularSBASolver* solver,
                                                  ceres::Problem & problem) {
    return false;
}

bool CorrespondencesSetSBAModule::addUVT2XYZMatch(Correspondences::Typed<Correspondences::UVT> const& uv,
                                                  Correspondences::Typed<Correspondences::XYZ> const& xyz,
                                                  StereoVisionApp::ModularSBASolver* solver,
                                                  ceres::Problem & problem) {
    return false;
}

bool CorrespondencesSetSBAModule::addUVT2GeoXYZMatch(Correspondences::Typed<Correspondences::UVT> const& uv,
                                                     Correspondences::Typed<Correspondences::GEOXYZ> const& xyz,
                                                     StereoVisionApp::ModularSBASolver* solver,
                                                     ceres::Problem & problem) {
    return false;
}

bool CorrespondencesSetSBAModule::addUVT2XYZTMatch(Correspondences::Typed<Correspondences::UVT> const& uv,
                                                   Correspondences::Typed<Correspondences::XYZT> const& xyz,
                                                   StereoVisionApp::ModularSBASolver* solver,
                                                   ceres::Problem & problem) {
    return false;
}

bool CorrespondencesSetSBAModule::init(ModularSBASolver* solver, ceres::Problem & problem) {

    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> lcsIdxs = currentProject->getIdsByClass(LocalCoordinateSystem::staticMetaObject.className());

    QVector<qint64> correspSetsIdxs = currentProject->getIdsByClass(CorrespondencesSet::staticMetaObject.className());

    for (qint64 correspSetId : correspSetsIdxs) {

        StereoVisionApp::CorrespondencesSet* correspSet = currentProject->getDataBlock<StereoVisionApp::CorrespondencesSet>(correspSetId);

        if (correspSet == nullptr) {
            continue;
        }

        if (!correspSet->isEnabled()) {
            continue;
        }

        using GenericPair = Correspondences::GenericPair;
        using GenericMatch = Correspondences::Generic;


        for (int i = 0; i < correspSet->nCorrespondence(); i++) {

            StereoVisionApp::Correspondences::GenericPair const& pair = correspSet->getCorrespondence(i);

            bool ok = false;

            if (pair.holdsCorrespondancesType<Correspondences::PRIOR,Correspondences::XYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::PRIOR,Correspondences::XYZ>().value();
                ok = setupXYZPrior(typedPair.c2, solver, problem);
            }

            if (pair.holdsCorrespondancesType<Correspondences::PRIORID,Correspondences::GEOXYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::PRIORID,Correspondences::GEOXYZ>().value();
                ok = addGeoPosPrior(typedPair.c1, typedPair.c2, solver, problem);
            }

            if (pair.holdsCorrespondancesType<Correspondences::PRIORID,Correspondences::GEOXY>()) {

                auto typedPair = pair.getTypedPair<Correspondences::PRIORID,Correspondences::GEOXY>().value();
                ok = addGeoProjPrior(typedPair.c1, typedPair.c2, solver, problem);
            }

            if (pair.holdsCorrespondancesType<Correspondences::XYZ,Correspondences::XYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::XYZ,Correspondences::XYZ>().value();
                ok = addXYZMatch(typedPair.c1, typedPair.c2, solver, problem);
            }

            if (pair.holdsCorrespondancesType<Correspondences::XYZ,Correspondences::Line3D>()) {

                auto typedPair = pair.getTypedPair<Correspondences::XYZ,Correspondences::Line3D>().value();
                ok = addXYZ2LineMatch(typedPair.c1, typedPair.c2, solver, problem);
            }

            if (pair.holdsCorrespondancesType<Correspondences::XYZ,Correspondences::Plane3D>()) {

                auto typedPair = pair.getTypedPair<Correspondences::XYZ,Correspondences::Plane3D>().value();
                ok = addXYZ2PlaneMatch(typedPair.c1, typedPair.c2, solver, problem);
            }

            if (pair.holdsCorrespondancesType<Correspondences::XYZ,Correspondences::GEOXYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::XYZ,Correspondences::GEOXYZ>().value();
                ok = addXYZ2GeoMatch(typedPair.c1, typedPair.c2, solver, problem);
            }

            if (pair.holdsCorrespondancesType<Correspondences::XYZ,Correspondences::GEOXY>()) {

                auto typedPair = pair.getTypedPair<Correspondences::XYZ,Correspondences::GEOXY>().value();
                ok = addXYZ2GeoMatch(typedPair.c1, typedPair.c2, solver, problem);
            }

            if (pair.holdsCorrespondancesType<Correspondences::XYZ,Correspondences::PRIORID>()) {

                auto typedPair = pair.getTypedPair<Correspondences::XYZ,Correspondences::PRIORID>().value();
                ok = addXYZ2PriorMatch(typedPair.c1, typedPair.c2, solver, problem);
            }

            //image correspondences
            if (pair.holdsCorrespondancesType<Correspondences::UV,Correspondences::UV>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UV,Correspondences::UV>().value();
                ok = addUV2UVMatch(typedPair.c1, typedPair.c2, solver, problem);
            }

            if (pair.holdsCorrespondancesType<Correspondences::UV,Correspondences::XYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UV,Correspondences::XYZ>().value();
                ok = addUV2XYZMatch(typedPair.c1, typedPair.c2, solver, problem);
            }

            if (pair.holdsCorrespondancesType<Correspondences::UV,Correspondences::GEOXYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UV,Correspondences::GEOXYZ>().value();
                ok = addUV2GeoXYZMatch(typedPair.c1, typedPair.c2, solver, problem);
            }

            if (pair.holdsCorrespondancesType<Correspondences::UV,Correspondences::XYZT>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UV,Correspondences::XYZT>().value();
                ok = addUV2XYZTMatch(typedPair.c1, typedPair.c2, solver, problem);
            }

            //timed image correspondences
            if (pair.holdsCorrespondancesType<Correspondences::UVT,Correspondences::UV>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UVT,Correspondences::UV>().value();
                ok = addUVT2UVMatch(typedPair.c1, typedPair.c2, solver, problem);
            }

            if (pair.holdsCorrespondancesType<Correspondences::UVT,Correspondences::UVT>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UVT,Correspondences::UVT>().value();
                ok = addUVT2UVTMatch(typedPair.c1, typedPair.c2, solver, problem);
            }

            if (pair.holdsCorrespondancesType<Correspondences::UVT,Correspondences::XYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UVT,Correspondences::XYZ>().value();
                ok = addUVT2XYZMatch(typedPair.c1, typedPair.c2, solver, problem);
            }

            if (pair.holdsCorrespondancesType<Correspondences::UVT,Correspondences::GEOXYZ>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UVT,Correspondences::GEOXYZ>().value();
                ok = addUVT2GeoXYZMatch(typedPair.c1, typedPair.c2, solver, problem);
            }

            if (pair.holdsCorrespondancesType<Correspondences::UVT,Correspondences::XYZT>()) {

                auto typedPair = pair.getTypedPair<Correspondences::UVT,Correspondences::XYZT>().value();
                ok = addUVT2XYZTMatch(typedPair.c1, typedPair.c2, solver, problem);
            }

            if (!ok) {
                solver->logMessage(QString("Could not add match %1 to factor graph").arg(pair.toString()));
            }
        }

    }

    return true;
}
bool CorrespondencesSetSBAModule::writeResults(ModularSBASolver* solver) {
    Q_UNUSED(solver);
    return true;
}
bool CorrespondencesSetSBAModule::writeUncertainty(ModularSBASolver* solver) {
    Q_UNUSED(solver);
    return true;
}
void CorrespondencesSetSBAModule::cleanup(ModularSBASolver* solver) {
    Q_UNUSED(solver);
}

} // namespace StereoVisionApp
