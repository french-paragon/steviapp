#include "landmarkssbamodule.h"

#include "datablocks/landmark.h"

#include <ceres/normal_prior.h>

namespace StereoVisionApp {

const char* LandmarksSBAModule::ModuleName = "SBAModule::Landmark";

LandmarksSBAModule::LandmarksSBAModule()
{

}

QString LandmarksSBAModule::moduleName() const {
    return QObject::tr("Landmarks SBA Module");
}

bool LandmarksSBAModule::addGraphReductorVariables(Project *currentProject, GenericSBAGraphReductor* graphReductor) {

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> lmks_v = currentProject->getIdsByClass(LandmarkFactory::landmarkClassName());

    for (qint64 id : lmks_v) {
        graphReductor->insertItem(id, 3);
    }

    return true;
}

bool LandmarksSBAModule::addGraphReductorObservations(Project *currentProject, GenericSBAGraphReductor* graphReductor) {
    return true;
}

bool LandmarksSBAModule::setupParameters(ModularSBASolver* solver) {

    Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> lmIdxs = currentProject->getIdsByClass(Landmark::staticMetaObject.className());

    for (qint64 lmId : lmIdxs) {

        if (!solver->itemIsObservable(lmId)) {
            continue;
        }

        Landmark* lm = qobject_cast<Landmark*>(currentProject->getById(lmId));

        if (lm == nullptr) {
            continue;
        }

        ModularSBASolver::PositionNode* lmNode = solver->getNodeForLandmark(lmId, true);

        if (lmNode == nullptr) {
            continue;
        }

    }

    return true;
}
bool LandmarksSBAModule::init(ModularSBASolver* solver, ceres::Problem & problem) {

    Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> lmIdxs = currentProject->getIdsByClass(Landmark::staticMetaObject.className());

    for (qint64 lmId : lmIdxs) {

        if (!solver->itemIsObservable(lmId)) {
            continue;
        }

        Landmark* lm = currentProject->getDataBlock<Landmark>(lmId);

        if (lm == nullptr) {
            return false;
        }

        if (!lm->isEnabled()) {
            return false;
        }

        ModularSBASolver::PositionNode* lmNode = solver->getNodeForLandmark(lmId, false);

        QString paramLogName = QString("Position for landmark %1").arg(lm->objectName());

        solver->addLogger(paramLogName, new ModularSBASolver::ParamsValsLogger<3>(lmNode->pos.data()));

        constexpr bool optimized = true;
        constexpr bool notOptimized = false;

        //get the initial optimization value in optimization frame (ecef for georeferenced).
        std::optional<Eigen::Vector3f> coordInitial = lm->getOptimizableCoordinates(optimized);

        if (!coordInitial.has_value()) {
            coordInitial = lm->getOptimizableCoordinates(notOptimized);
        }

        Eigen::Vector3d vecInitial;

        if (coordInitial.has_value()) {
            vecInitial = coordInitial.value().cast<double>();
        }

        //move to local optimization frame
        vecInitial = solver->getTransform2LocalFrame()*vecInitial;

        if (!coordInitial.has_value()) {
            vecInitial = Eigen::Vector3d::Zero();
        }

        lmNode->pos[0] = vecInitial.x();
        lmNode->pos[1] = vecInitial.y();
        lmNode->pos[2] = vecInitial.z();

        std::optional<Eigen::Vector3f> coordPrior = lm->getOptimizableCoordinates(notOptimized);

        if (coordPrior.has_value()) {

            Eigen::Vector3d vecPrior = coordPrior.value().cast<double>();

            vecPrior = solver->getTransform2LocalFrame()*vecPrior;

            ceres::Vector m;
            m.resize(3);

            m[0] = vecPrior.x();
            m[1] = vecPrior.y();
            m[2] = vecPrior.z();

            Eigen::Matrix3d stiffness = Eigen::Matrix3d::Identity();

            if (lm->xCoord().isUncertain()) {
                stiffness(0,0) = 1./std::abs(lm->xCoord().stddev());
            } else {
                stiffness(0,0) = 1e6;
            }

            if (lm->yCoord().isUncertain()) {
                stiffness(1,1) = 1./std::abs(lm->yCoord().stddev());
            } else {
                stiffness(1,1) = 1e6;
            }

            if (lm->zCoord().isUncertain()) {
                stiffness(2,2) = 1./std::abs(lm->zCoord().stddev());
            } else {
                stiffness(2,2) = 1e6;
            }

            ceres::NormalPrior* normalPrior = new ceres::NormalPrior(stiffness, m);

            problem.AddResidualBlock(normalPrior, nullptr, lmNode->pos.data());
        }

    }

    return true;

}

bool LandmarksSBAModule::writeResults(ModularSBASolver* solver) {

    Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> lmIdxs = currentProject->getIdsByClass(Landmark::staticMetaObject.className());

    StereoVision::Geometry::AffineTransform<double> ecef2local = solver->getTransform2LocalFrame();
    StereoVision::Geometry::AffineTransform<double> local2ecef(ecef2local.R.transpose(), -ecef2local.R.transpose()*ecef2local.t);

    for (qint64 lmId : lmIdxs) {

        Landmark* lm = qobject_cast<Landmark*>(currentProject->getById(lmId));

        if (lm->isFixed()) {
            continue;
        }

        ModularSBASolver::PositionNode* lm_p = solver->getPositionNode(lmId);

        if (lm_p == nullptr) {
            continue;
        }

        if (lm->geoReferenceSupportActive()) {

            Eigen::Vector3d pos;

            pos[0] = lm_p->pos[0];
            pos[1] = lm_p->pos[1];
            pos[2] = lm_p->pos[2];



            Eigen::Vector3d tmp = local2ecef*pos;
            pos = tmp;

            constexpr bool optimized = true;
            lm->setPositionFromEcef(pos.cast<float>(), optimized);

        } else {

            floatParameterGroup<3> pos;
            pos.value(0) = static_cast<float>(lm_p->pos[0]);
            pos.value(1) = static_cast<float>(lm_p->pos[1]);
            pos.value(2) = static_cast<float>(lm_p->pos[2]);
            pos.setIsSet();
            lm->setOptPos(pos);
        }
    }

    return true;

}

std::vector<std::pair<const double*, const double*>> LandmarksSBAModule::requestUncertainty(ModularSBASolver* solver, ceres::Problem & problem) {

    std::vector<std::pair<const double*, const double*>> ret;

    Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return ret;
    }

    QVector<qint64> lmIdxs = currentProject->getIdsByClass(Landmark::staticMetaObject.className());

    ret.reserve(lmIdxs.size());

    for (qint64 lmId : lmIdxs) {

        Landmark* lm = qobject_cast<Landmark*>(currentProject->getById(lmId));

        if (lm->isFixed()) {
            continue;
        }

        ModularSBASolver::PositionNode* lm_p = solver->getPositionNode(lmId);

        if (!problem.HasParameterBlock(lm_p->pos.data())) {
            continue;
        }

        ret.push_back({lm_p->pos.data(), lm_p->pos.data()});
    }

    return ret;

}

bool LandmarksSBAModule::writeUncertainty(ModularSBASolver* solver) {

    Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> lmIdxs = currentProject->getIdsByClass(Landmark::staticMetaObject.className());

    for (qint64 lmId : lmIdxs) {

        Landmark* lm = qobject_cast<Landmark*>(currentProject->getById(lmId));

        if (lm->isFixed()) {
            continue;
        }

        ModularSBASolver::PositionNode* lm_p = solver->getPositionNode(lmId);

        std::optional<Eigen::MatrixXd> covBlock = solver->getCovarianceBlock({lm_p->pos.data(),lm_p->pos.data()});

        if (covBlock.has_value()) {
            floatParameterGroup<3> oP = lm->optPos();
            oP.setUncertain();
            oP.stddev(0,0) = covBlock.value()(0,0);
            oP.stddev(1,1) = covBlock.value()(1,1);
            oP.stddev(2,2) = covBlock.value()(2,2);
            oP.stddev(0,1) = covBlock.value()(0,1);
            oP.stddev(1,2) = covBlock.value()(1,2);
            oP.stddev(2,0) = covBlock.value()(2,0);
            lm->setOptPos(oP);
        }
    }

    return true;

}
void LandmarksSBAModule::cleanup(ModularSBASolver* solver) {

    Q_UNUSED(solver);
    return;

}

} // namespace StereoVisionApp
