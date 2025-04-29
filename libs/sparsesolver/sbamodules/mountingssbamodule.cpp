#include "mountingssbamodule.h"

#include "datablocks/mounting.h"

#include "costfunctors/fixedsizenormalprior.h"

namespace StereoVisionApp {

const char* MountingsSBAModule::ModuleName = "SBAModule::Mountings";

MountingsSBAModule::MountingsSBAModule()
{

}

QString MountingsSBAModule::moduleName() const {
    return QObject::tr("Mountings SBA Module");
}

bool MountingsSBAModule::addGraphReductorVariables(Project *currentProject, GenericSBAGraphReductor* graphReductor) {

    return true;

}
bool MountingsSBAModule::addGraphReductorObservations(Project *currentProject, GenericSBAGraphReductor* graphReductor) {
    return true;
}

bool MountingsSBAModule::setupParameters(ModularSBASolver* solver) {

    return true; //don't do the initilization of mounting parameters itself
}
bool MountingsSBAModule::init(ModularSBASolver* solver, ceres::Problem & problem) {

    Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    constexpr bool dontCreateIfMissing = false;

    QVector<qint64> mountingIdxs = currentProject->getIdsByClass(Mounting::staticMetaObject.className());

    for (qint64 mountingId : mountingIdxs) {

        Mounting* mounting = qobject_cast<Mounting*>(currentProject->getById(mountingId));

        if (mounting == nullptr) {
            continue;
        }

        ModularSBASolver::PoseNode* mountingPoseNode = solver->getPoseNode(mountingId);

        if (mountingPoseNode == nullptr) {
            continue;
        }

        //priors

        std::array<double, 3> raxis_prior;
        std::array<double, 3> t_prior;

        //TODO: check if the use of axis angle representation here is consistent.
        if (mounting->xRot().isSet() and mounting->yRot().isSet() and mounting->zRot().isSet()) {
            raxis_prior[0] = mounting->xRot().value();
            raxis_prior[1] = mounting->yRot().value();
            raxis_prior[2] = mounting->zRot().value();
        }

        if (mounting->xCoord().isSet() and mounting->yCoord().isSet() and mounting->zCoord().isSet()) {
            t_prior[0] = mounting->xCoord().value();
            t_prior[1] = mounting->yCoord().value();
            t_prior[2] = mounting->zCoord().value();
        }

        if (!mounting->isFixed() and mounting->xCoord().isUncertain() and mounting->yCoord().isUncertain() and mounting->zCoord().isUncertain()) {

            Eigen::Matrix3d At = Eigen::Matrix3d::Identity();

            At(0,0) = 1./std::abs(mounting->xCoord().stddev());
            At(1,1) = 1./std::abs(mounting->yCoord().stddev());
            At(2,2) = 1./std::abs(mounting->zCoord().stddev());

            Eigen::Vector3d bt;
            bt << t_prior[0],t_prior[1], t_prior[2];

            FixedSizeNormalPrior<3,3>* tPrior = new FixedSizeNormalPrior<3,3>(At, bt);
            problem.AddResidualBlock(tPrior, nullptr, mountingPoseNode->t.data());

        }

        if (!mounting->isFixed() and mounting->xRot().isUncertain() and mounting->yRot().isUncertain() and mounting->zRot().isUncertain()) {

            Eigen::Matrix3d Araxis = Eigen::Matrix3d::Identity();

            Araxis(0,0) = 1./std::abs(mounting->xRot().stddev());
            Araxis(1,1) = 1./std::abs(mounting->yRot().stddev());
            Araxis(2,2) = 1./std::abs(mounting->zRot().stddev());

            Eigen::Vector3d  braxis;
            braxis << raxis_prior[0],raxis_prior[1], raxis_prior[2];

            FixedSizeNormalPrior<3,3>* rAxisPrior = new FixedSizeNormalPrior<3,3>(Araxis, (braxis));
            problem.AddResidualBlock(rAxisPrior, nullptr, mountingPoseNode->rAxis.data());

        }

    }

    return true;

}
bool MountingsSBAModule::writeResults(ModularSBASolver* solver) {

    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> mountingIdxs = currentProject->getIdsByClass(Mounting::staticMetaObject.className());

    for (qint64 id : mountingIdxs) {

        ModularSBASolver::PoseNode* mounting_p = solver->getPoseNode(id);

        if (mounting_p == nullptr) {
            continue;
        }

        Mounting* mounting = currentProject->getDataBlock<Mounting>(id);

        if (mounting == nullptr) {
            continue;
        }

        if (mounting->isFixed()) {
            continue;
        }

        //TODO: add georeferencing support
        floatParameterGroup<3> pos;
        pos.value(0) = static_cast<float>(mounting_p->t[0]);
        pos.value(1) = static_cast<float>(mounting_p->t[1]);
        pos.value(2) = static_cast<float>(mounting_p->t[2]);
        pos.setIsSet();
        mounting->setOptPos(pos);

        floatParameterGroup<3> rot;
        rot.value(0) = static_cast<float>(mounting_p->rAxis[0]);
        rot.value(1) = static_cast<float>(mounting_p->rAxis[1]);
        rot.value(2) = static_cast<float>(mounting_p->rAxis[2]);
        rot.setIsSet();
        mounting->setOptRot(rot);

    }

    return true;

}
std::vector<std::pair<const double*, const double*>> MountingsSBAModule::requestUncertainty(ModularSBASolver* solver, ceres::Problem & problem) {

    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return std::vector<std::pair<const double*, const double*>>();
    }

    QVector<qint64> mountingIdxs = currentProject->getIdsByClass(Mounting::staticMetaObject.className());

    std::vector<std::pair<const double*, const double*>> ret;
    ret.reserve(2*mountingIdxs.size());

    for (qint64 id : mountingIdxs) {

        ModularSBASolver::PoseNode* mounting_p = solver->getPoseNode(id);

        if (mounting_p == nullptr) {
            continue;
        }

        if (!problem.IsParameterBlockConstant(mounting_p->t.data())) {
            ret.push_back({mounting_p->t.data(),mounting_p->t.data()});
        }

        if (!problem.IsParameterBlockConstant(mounting_p->rAxis.data())) {
            ret.push_back({mounting_p->rAxis.data(),mounting_p->rAxis.data()});
        }
    }

    return ret;
}
bool MountingsSBAModule::writeUncertainty(ModularSBASolver* solver) {

    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> mountingIdxs = currentProject->getIdsByClass(Mounting::staticMetaObject.className());

    std::vector<std::pair<const double*, const double*>> ret;
    ret.reserve(2*mountingIdxs.size());

    for (qint64 id : mountingIdxs) {

        ModularSBASolver::PoseNode* mounting_p = solver->getPoseNode(id);

        if (mounting_p == nullptr) {
            continue;
        }

        Mounting* mounting = currentProject->getDataBlock<Mounting>(id);

        if (mounting == nullptr) {
            continue;
        }

        if (mounting->isFixed()) {
            continue;
        }

        std::optional<Eigen::MatrixXd> covBlock = solver->getCovarianceBlock({mounting_p->t.data(),mounting_p->t.data()});

        if (covBlock.has_value()) {
            floatParameterGroup<3> oT = mounting->optPos();
            oT.setUncertain();
            oT.stddev(0,0) = covBlock.value()(0,0);
            oT.stddev(1,1) = covBlock.value()(1,1);
            oT.stddev(2,2) = covBlock.value()(2,2);
            oT.stddev(0,1) = covBlock.value()(0,1);
            oT.stddev(1,2) = covBlock.value()(1,2);
            oT.stddev(2,0) = covBlock.value()(2,0);
            mounting->setOptPos(oT);
        }

        covBlock = solver->getCovarianceBlock({mounting_p->rAxis.data(),mounting_p->rAxis.data()});

        if (covBlock.has_value()) {
            floatParameterGroup<3> oR = mounting->optRot();
            oR.setUncertain();
            oR.stddev(0,0) = covBlock.value()(0,0);
            oR.stddev(1,1) = covBlock.value()(1,1);
            oR.stddev(2,2) = covBlock.value()(2,2);
            oR.stddev(0,1) = covBlock.value()(0,1);
            oR.stddev(1,2) = covBlock.value()(1,2);
            oR.stddev(2,0) = covBlock.value()(2,0);
            mounting->setOptRot(oR);
        }
    }

    return true;
}
void MountingsSBAModule::cleanup(ModularSBASolver* solver) {
    return;
}

} // namespace StereoVisionApp
