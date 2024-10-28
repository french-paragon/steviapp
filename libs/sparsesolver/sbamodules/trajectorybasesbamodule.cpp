#include "trajectorybasesbamodule.h"

#include "datablocks/trajectory.h"
#include "datablocks/datatable.h"

#include <ceres/normal_prior.h>

#include "../costfunctors/interpolatedvectorprior.h"
#include "../costfunctors/imustepcost.h"
#include "./costfunctors/weightedcostfunction.h"

#include "../sbagraphreductor.h"

#include "utils/statusoptionalreturn.h"

namespace StereoVisionApp {

const char* TrajectoryBaseSBAModule::ModuleName = "SBAModule::Trajectory";

TrajectoryBaseSBAModule::TrajectoryBaseSBAModule(double integrationTime) :
    _integrationTime(integrationTime)
{
    _accelerometerBias = true;
    _accelerometerScale = true;

    _accelerometersBiases = std::vector<std::array<double,3>>();
    _accelerometersScales = std::vector<std::array<double,1>>();
}

bool TrajectoryBaseSBAModule::addGraphReductorVariables(Project* currentProject, GenericSBAGraphReductor* graphReductor) {

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> trajectoriesIdxs = currentProject->getIdsByClass(Trajectory::staticMetaObject.className());

    for (qint64 trajId : trajectoriesIdxs) {
        graphReductor->insertItem(trajId, 0);
    }

    return true;

}
bool TrajectoryBaseSBAModule::addGraphReductorObservations(Project *currentProject, GenericSBAGraphReductor* graphReductor) {
    return true;
}

bool TrajectoryBaseSBAModule::init(ModularSBASolver* solver, ceres::Problem & problem) {

    Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    _gravity = {0,0,-9.81};
    problem.AddParameterBlock(_gravity.data(), _gravity.size());

   Eigen::Matrix3d gInfos = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; i++) {
        gInfos(i,i) = 1;
    }
    Eigen::Vector3d gVec;
    gVec << _gravity[0], _gravity[1], _gravity[2];

    ceres::NormalPrior* g_prior = new ceres::NormalPrior(gInfos, gVec);

    problem.AddResidualBlock(g_prior, nullptr, _gravity.data());

    QVector<qint64> trajectoriesIdxs = currentProject->getIdsByClass(Trajectory::staticMetaObject.className());

    _accelerometersBiases.reserve(trajectoriesIdxs.size());
    _accelerometersScales.reserve(trajectoriesIdxs.size());

    for (qint64 trajId : trajectoriesIdxs) {

        Trajectory* traj = currentProject->getDataBlock<Trajectory>(trajId);

        if (traj == nullptr) {
            continue;
        }

        if (!traj->isEnabled()) {
            continue;
        }

        int accId = traj->accelerometerId();

        if (!_accelerometerParametersIndex.contains(accId)) {
            if (_accelerometerBias or _accelerometerScale) {

                int id = _accelerometersBiases.size();

                if (_accelerometerScale) {
                    id = _accelerometersScales.size();
                }

                _accelerometerParametersIndex.insert(accId, id);
                accId = id;

                if (_accelerometerBias) {
                    _accelerometersBiases.push_back({0,0,0});
                    problem.AddParameterBlock(_accelerometersBiases[accId].data(), _accelerometersBiases[accId].size());

                }
                if (_accelerometerScale) {
                    _accelerometersScales.push_back({1});
                    problem.AddParameterBlock(_accelerometersScales[accId].data(), _accelerometersScales[accId].size());
                }
            }
        } else {
            accId = _accelerometerParametersIndex[accId];
        }

        ModularSBASolver::TrajectoryNode* trajNode = solver->getNodeForTrajectory(trajId, true);

        if (trajNode == nullptr) {
            continue;
        }

        StereoVision::Geometry::AffineTransform<double> world2local = solver->getTransform2LocalFrame();

        StatusOptionalReturn<Trajectory::TimeCartesianSequence> optGps = traj->loadPositionSequence(); //load in ecef
        StatusOptionalReturn<Trajectory::TimeTrajectorySequence> optPose = traj->loadTrajectorySequence(); //used for initialization

        //TODO: make initialization possible from just GPS + gyro

        if (!optGps.isValid() and !optPose.isValid()) { //Canot initialize without GPS and orientation //TODO: investigate if we can initialize with
            continue;
        }

        StatusOptionalReturn<Trajectory::TimeCartesianSequence> optGyro = traj->loadAngularSpeedSequence();
        StatusOptionalReturn<Trajectory::TimeCartesianSequence> optImu = traj->loadAccelerationSequence();

        double minTime = optGps.value().sequenceStartTime();
        double maxTime = optGps.value().sequenceEndTime();

        if (optGyro.isValid()) {

            minTime = std::max(minTime, optGyro.value().sequenceStartTime());
            maxTime = std::min(maxTime, optGyro.value().sequenceEndTime());

        }

        if (optImu.isValid()) {

            minTime = std::max(minTime, optImu.value().sequenceStartTime());
            maxTime = std::min(maxTime, optImu.value().sequenceEndTime());

        }

        double duration = maxTime - minTime;

        if (duration <= 0) {
            continue;
        }

        int nGPSNode = optGps.value().nPoints();

        if (nGPSNode <= 0) {
            continue;
        }

        int nSteps = std::ceil(duration/_integrationTime)+1;
        double stepTime = duration/nSteps;

        int nTrajLoggers = std::min(500,nSteps);
        int trajLoggersSteps = nSteps/nTrajLoggers;

        trajNode->nodes.resize(nSteps);

        int currentGPSNode = 0;

        int nAlignChar = std::ceil(std::log10(nSteps));

        for (int i = 0; i < nSteps; i++) {

            double time = minTime + i*stepTime;

            trajNode->nodes[i].time = time;

            //GPS initilation and cost
            auto interpolatablePos = optGps.value().getValueAtTime(time);
            Eigen::Vector3d pos =
                    interpolatablePos.weigthLower*interpolatablePos.valLower +
                    interpolatablePos.weigthUpper*interpolatablePos.valUpper;
            pos = world2local*pos;

            auto interpolatablePose = optPose.value().getValueAtTime(time); //plateform to world.
            Eigen::Vector3d rot =
                    interpolatablePose.weigthLower*interpolatablePose.valLower.r +
                    interpolatablePose.weigthUpper*interpolatablePose.valUpper.r;
            Eigen::Matrix3d R = world2local.R*StereoVision::Geometry::rodriguezFormula<double>(rot);
            rot = StereoVision::Geometry::inverseRodriguezFormula<double>(R);

            trajNode->nodes[i].t = {pos[0], pos[1], pos[2]};
            trajNode->nodes[i].rAxis = {rot[0], rot[1], rot[2]};

            problem.AddParameterBlock(trajNode->nodes[i].t.data(), trajNode->nodes[i].t.size());
            problem.AddParameterBlock(trajNode->nodes[i].rAxis.data(), trajNode->nodes[i].rAxis.size());

            if (traj->isFixed()) {
                problem.SetParameterBlockConstant(trajNode->nodes[i].t.data());
                problem.SetParameterBlockConstant(trajNode->nodes[i].rAxis.data());
            }

            if (i == 0) { //need at least two nodes for first order cost function
                continue;
            }

            //gps observations
            if (i > 0 and _gpsAccuracy > 1e-5) {

                double t1 = trajNode->nodes[i-1].time;
                double t2 = trajNode->nodes[i].time;

                for (int g = currentGPSNode; g < nGPSNode; g++) {
                    if (optGps.value()[g].time >= t1) {
                        currentGPSNode = g;
                        break;
                    }
                }

                double gpsObs_t = optGps.value()[currentGPSNode].time;

                if (gpsObs_t > t2) { // no GPS observation between  trajectory nodes
                    continue;
                }

                double w1 = (t2 - gpsObs_t)/(t2-t1);
                double w2 = (gpsObs_t-t1)/(t2-t1);

                Eigen::Matrix3d infos = Eigen::Matrix3d::Zero();
                Eigen::Vector3d vec;

                //position in local optimization frame
                vec = world2local*optGps.value()[currentGPSNode].val;

                for (int i = 0; i < 3; i++) {
                    infos(i,i) = 1/_gpsAccuracy;
                }

                //add gps based trajectory priors
                if (std::abs(w1-1) < 1e-3 or std::abs(w2-1) < 1e-3) {

                    ceres::NormalPrior* gpsPrior = new ceres::NormalPrior(infos, vec);

                    if (std::abs(w1-1) < 1e-3 or std::abs(w2-1) < 1e-3) {

                        ModularSBASolver::AutoErrorBlockLogger<1,3>::ParamsType params;

                        if (std::abs(w1-1) < 1e-3) {
                            params = {trajNode->nodes[i-1].t.data()};
                        } else if (std::abs(w2-1) < 1e-3) {
                            params = {trajNode->nodes[i].t.data()};
                        }

                        problem.AddResidualBlock(gpsPrior, nullptr,
                                                 params.data(),
                                                 params.size());


                        if ((i+1)%trajLoggersSteps == 0) {

                            QString posLoggerName = QString("Trajectory \"%1\" Position index %2").arg(traj->objectName()).arg(i, nAlignChar);
                            solver->addLogger(posLoggerName, new ModularSBASolver::ParamsValsLogger<3>(trajNode->nodes[i].t.data()));

                            QString rotLoggerName = QString("Trajectory \"%1\" Orientation index %2").arg(traj->objectName()).arg(i, nAlignChar);
                            solver->addLogger(rotLoggerName, new ModularSBASolver::ParamsValsLogger<3>(trajNode->nodes[i].rAxis.data()));

                            QString loggerName = QString("GPS trajectory \"%1\" time %2").arg(traj->objectName()).arg(gpsObs_t, 0, 'f', 2);
                            solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<1,3>(gpsPrior, params));
                        }


                    } else {
                        delete gpsPrior; //useless as branch is unreachable, but remove static analysis error
                    }

                } else {

                    InterpolatedVectorPrior<3>* interpolatedPriorCost = new InterpolatedVectorPrior<3>(vec, w1, w2, infos);

                    ceres::AutoDiffCostFunction<InterpolatedVectorPrior<3>, 3,3,3>*  interpolatedPrior =
                            new ceres::AutoDiffCostFunction<InterpolatedVectorPrior<3>, 3,3,3>(interpolatedPriorCost);


                    ModularSBASolver::AutoErrorBlockLogger<2,3>::ParamsType params = {trajNode->nodes[i-1].t.data(), trajNode->nodes[i].t.data()};

                    problem.AddResidualBlock(interpolatedPrior, nullptr,
                                             params.data(),
                                             params.size());

                    if ((i+1)%trajLoggersSteps == 0) {

                        QString posLoggerName = QString("Trajectory \"%1\" Position index %2").arg(traj->objectName()).arg(i, nAlignChar);
                        solver->addLogger(posLoggerName, new ModularSBASolver::ParamsValsLogger<3>(trajNode->nodes[i].t.data()));

                        QString rotLoggerName = QString("Trajectory \"%1\" Orientation index %2").arg(traj->objectName()).arg(i, nAlignChar);
                        solver->addLogger(rotLoggerName, new ModularSBASolver::ParamsValsLogger<3>(trajNode->nodes[i].rAxis.data()));

                        QString loggerName = QString("GPS trajectory \"%1\" time %2 (interpolated)").arg(traj->objectName()).arg(gpsObs_t, 0, 'f', 2);
                        solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<2,3>(interpolatedPrior, params));
                    }

                }

            }

            //gyro cost

            if (optGyro.isValid()) {

                GyroStepCost* gyroStepCost =
                        GyroStepCost::getIntegratedIMUDiff(optGyro.value(),
                                                           trajNode->nodes[i-1].time,
                                                           trajNode->nodes[i].time);

                ceres::AutoDiffCostFunction<StereoVisionApp::GyroStepCost, 3,3,3>* gyroStepCostFunction =
                        new ceres::AutoDiffCostFunction<StereoVisionApp::GyroStepCost, 3,3,3>(gyroStepCost);

                double dt = trajNode->nodes[i].time - trajNode->nodes[i-1].time;
                Eigen::Matrix<double,3,3> weigthMat = Eigen::Matrix<double,3,3>::Identity();

                double poseUncertainty = _gyroAccuracy*dt;

                weigthMat(0,0) = 1/poseUncertainty;
                weigthMat(1,1) = 1/poseUncertainty;
                weigthMat(2,2) = 1/poseUncertainty;

                StereoVisionApp::WeightedCostFunction<3,3,3>* weigthedGyroStepCost =
                        new StereoVisionApp::WeightedCostFunction<3,3,3>(gyroStepCostFunction, weigthMat);


                ModularSBASolver::AutoErrorBlockLogger<2,3>::ParamsType params = {trajNode->nodes[i-1].rAxis.data(), trajNode->nodes[i].rAxis.data()};

                problem.AddResidualBlock(weigthedGyroStepCost, nullptr,
                                          params.data(),
                                          params.size());

                if ((i+1)%trajLoggersSteps == 0) {
                    QString loggerName = QString("Gyro trajectory \"%1\" step time %2").arg(traj->objectName()).arg(time, 0, 'f', 2);
                    solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<2,3>(gyroStepCostFunction, params));
                }
            }

            //accelerometer cost

            if (optGyro.isValid() and optImu.isValid()) {

                if (i == 0 or i == 1) { //need at least three nodes for second order cost function
                    continue;
                }

                if (_accelerometerBias and _accelerometerScale) {

                    using AccCost = AccelerometerStepCost<true, true>;

                    AccCost* accStepCost =
                            AccelerometerStepCostBase::getIntegratedIMUDiff<true, true>(optGyro.value(),
                                                                        optImu.value(),
                                                                        trajNode->nodes[i-2].time,
                                                                        trajNode->nodes[i-1].time,
                                                                        trajNode->nodes[i].time);

                    ceres::AutoDiffCostFunction<AccCost, 3,3,3,3,3,3,3,1,3>* accStepCostFunction =
                            new ceres::AutoDiffCostFunction<AccCost, 3,3,3,3,3,3,3,1,3>(accStepCost);

                    double dt = (trajNode->nodes[i].time - trajNode->nodes[i-2].time)/2;
                    Eigen::Matrix<double,3,3> weigthMat = Eigen::Matrix<double,3,3>::Identity();

                    double speedUncertainty = _accAccuracy*dt;

                    weigthMat(0,0) = 1/speedUncertainty;
                    weigthMat(1,1) = 1/speedUncertainty;
                    weigthMat(2,2) = 1/speedUncertainty;

                    StereoVisionApp::WeightedCostFunction<3,3,3,3,3,3,3,1,3>* weigthedAccStepCost =
                            new StereoVisionApp::WeightedCostFunction<3,3,3,3,3,3,3,1,3>(accStepCostFunction, weigthMat);


                    ModularSBASolver::AutoErrorBlockLogger<8,3>::ParamsType params =
                        {trajNode->nodes[i-2].rAxis.data(),
                         trajNode->nodes[i-2].t.data(),
                         trajNode->nodes[i-1].rAxis.data(),
                         trajNode->nodes[i-1].t.data(),
                         trajNode->nodes[i].t.data(),
                         _gravity.data(),
                         _accelerometersScales[accId].data(),
                         _accelerometersBiases[accId].data()};

                    problem.AddResidualBlock(weigthedAccStepCost, nullptr, params.data(), params.size());

                    if ((i+1)%trajLoggersSteps == 0) {
                        QString loggerName = QString("Accelerometer trajectory \"%1\" step time %2").arg(traj->objectName()).arg(time, 0, 'f', 2);
                        solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<8,3>(accStepCostFunction, params));
                    }

                } else if (_accelerometerBias) {

                    using AccCost = AccelerometerStepCost<true, false>;

                    AccCost* accStepCost =
                            AccelerometerStepCostBase::getIntegratedIMUDiff<true, false>(optGyro.value(),
                                                                        optImu.value(),
                                                                        trajNode->nodes[i-2].time,
                                                                        trajNode->nodes[i-1].time,
                                                                        trajNode->nodes[i].time);

                    ceres::AutoDiffCostFunction<AccCost, 3,3,3,3,3,3,3,3>* accStepCostFunction =
                            new ceres::AutoDiffCostFunction<AccCost, 3,3,3,3,3,3,3,3>(accStepCost);

                    double dt = (trajNode->nodes[i].time - trajNode->nodes[i-2].time)/2;
                    Eigen::Matrix<double,3,3> weigthMat = Eigen::Matrix<double,3,3>::Identity();

                    double speedUncertainty = _accAccuracy*dt;

                    weigthMat(0,0) = 1/speedUncertainty;
                    weigthMat(1,1) = 1/speedUncertainty;
                    weigthMat(2,2) = 1/speedUncertainty;

                    StereoVisionApp::WeightedCostFunction<3,3,3,3,3,3,3,3>* weigthedAccStepCost =
                            new StereoVisionApp::WeightedCostFunction<3,3,3,3,3,3,3,3>(accStepCostFunction, weigthMat);


                    ModularSBASolver::AutoErrorBlockLogger<7,3>::ParamsType params =
                        {trajNode->nodes[i-2].rAxis.data(),
                         trajNode->nodes[i-2].t.data(),
                         trajNode->nodes[i-1].rAxis.data(),
                         trajNode->nodes[i-1].t.data(),
                         trajNode->nodes[i].t.data(),
                         _gravity.data(),
                         _accelerometersBiases[accId].data()};

                    problem.AddResidualBlock(weigthedAccStepCost, nullptr, params.data(), params.size());

                    if ((i+1)%trajLoggersSteps == 0) {
                        QString loggerName = QString("Accelerometer trajectory \"%1\" step time %2").arg(traj->objectName()).arg(time, 0, 'f', 2);
                        solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<7,3>(accStepCostFunction, params));
                    }
                } else if (_accelerometerScale) {

                    using AccCost = AccelerometerStepCost<false, true>;

                    AccCost* accStepCost =
                            AccelerometerStepCostBase::getIntegratedIMUDiff<false, true>(optGyro.value(),
                                                                        optImu.value(),
                                                                        trajNode->nodes[i-2].time,
                                                                        trajNode->nodes[i-1].time,
                                                                        trajNode->nodes[i].time);

                    ceres::AutoDiffCostFunction<AccCost, 3,3,3,3,3,3,3,1>* accStepCostFunction =
                            new ceres::AutoDiffCostFunction<AccCost, 3,3,3,3,3,3,3,1>(accStepCost);

                    double dt = (trajNode->nodes[i].time - trajNode->nodes[i-2].time)/2;
                    Eigen::Matrix<double,3,3> weigthMat = Eigen::Matrix<double,3,3>::Identity();

                    double speedUncertainty = _accAccuracy*dt;

                    weigthMat(0,0) = 1/speedUncertainty;
                    weigthMat(1,1) = 1/speedUncertainty;
                    weigthMat(2,2) = 1/speedUncertainty;

                    StereoVisionApp::WeightedCostFunction<3,3,3,3,3,3,3,1>* weigthedAccStepCost =
                            new StereoVisionApp::WeightedCostFunction<3,3,3,3,3,3,3,1>(accStepCostFunction, weigthMat);


                    ModularSBASolver::AutoErrorBlockLogger<7,3>::ParamsType params =
                        {trajNode->nodes[i-2].rAxis.data(),
                         trajNode->nodes[i-2].t.data(),
                         trajNode->nodes[i-1].rAxis.data(),
                         trajNode->nodes[i-1].t.data(),
                         trajNode->nodes[i].t.data(),
                         _gravity.data(),
                        _accelerometersScales[accId].data()};

                    problem.AddResidualBlock(weigthedAccStepCost, nullptr, params.data(), params.size());

                    if ((i+1)%trajLoggersSteps == 0) {
                        QString loggerName = QString("Accelerometer trajectory \"%1\" step time %2").arg(traj->objectName()).arg(time, 0, 'f', 2);
                        solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<7,3>(accStepCostFunction, params));
                    }

                } else {

                    using AccCost = AccelerometerStepCost<false, false>;

                    AccCost* accStepCost =
                            AccelerometerStepCostBase::getIntegratedIMUDiff<false, false>(optGyro.value(),
                                                                        optImu.value(),
                                                                        trajNode->nodes[i-2].time,
                                                                        trajNode->nodes[i-1].time,
                                                                        trajNode->nodes[i].time);

                    ceres::AutoDiffCostFunction<AccCost, 3,3,3,3,3,3,3>* accStepCostFunction =
                            new ceres::AutoDiffCostFunction<AccCost, 3,3,3,3,3,3,3>(accStepCost);

                    double dt = (trajNode->nodes[i].time - trajNode->nodes[i-2].time)/2;
                    Eigen::Matrix<double,3,3> weigthMat = Eigen::Matrix<double,3,3>::Identity();

                    double speedUncertainty = _accAccuracy*dt;

                    weigthMat(0,0) = 1/speedUncertainty;
                    weigthMat(1,1) = 1/speedUncertainty;
                    weigthMat(2,2) = 1/speedUncertainty;

                    StereoVisionApp::WeightedCostFunction<3,3,3,3,3,3,3>* weigthedAccStepCost =
                            new StereoVisionApp::WeightedCostFunction<3,3,3,3,3,3,3>(accStepCostFunction, weigthMat);


                    ModularSBASolver::AutoErrorBlockLogger<6,3>::ParamsType params =
                        {trajNode->nodes[i-2].rAxis.data(),
                         trajNode->nodes[i-2].t.data(),
                         trajNode->nodes[i-1].rAxis.data(),
                         trajNode->nodes[i-1].t.data(),
                         trajNode->nodes[i].t.data(),
                         _gravity.data()};

                    problem.AddResidualBlock(weigthedAccStepCost, nullptr, params.data(), params.size());

                    if ((i+1)%trajLoggersSteps == 0) {
                        QString loggerName = QString("Accelerometer trajectory \"%1\" step time %2").arg(traj->objectName()).arg(time, 0, 'f', 2);
                        solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<6,3>(accStepCostFunction, params));
                    }
                }
            }

        }
    }

    return true;

}

bool TrajectoryBaseSBAModule::writeResults(ModularSBASolver* solver) {

    StereoVisionApp::Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    QList<qint64> trajIdxs = solver->trajectoriesList();

    for (qint64 trajIdx : trajIdxs) {

        ModularSBASolver::TrajectoryNode* node = solver->getNodeForTrajectory(trajIdx, false);

        if (node == nullptr) {
            continue;
        }

        StereoVisionApp::Trajectory* trajectory = currentProject->getDataBlock<StereoVisionApp::Trajectory>(node->trajId);

        if (trajectory == nullptr) {
            continue;
        }

        int accId = trajectory->accelerometerId();

        if (_accelerometerParametersIndex.contains(accId)) {
            int paramIdx = _accelerometerParametersIndex[accId];

            if (_accelerometerBias) {

                std::array<double,3> const& bias = _accelerometersBiases[paramIdx];

                trajectory->setOptAccelerometerBiasX(bias[0]);
                trajectory->setOptAccelerometerBiasY(bias[1]);
                trajectory->setOptAccelerometerBiasZ(bias[2]);
            }

            if (_accelerometerScale) {

                std::array<double,1> const& scale = _accelerometersScales[paramIdx];

                trajectory->setOptAccelerometerScale(scale[0]);
            }
        }

        StereoVisionApp::DataTable* resultDataTable = trajectory->getOptimizedDataTable();

        if (resultDataTable == nullptr) {
            continue;
        }

        QVector<QVariant> finalTimes;

        QVector<QVariant> finalTrajPosX;
        QVector<QVariant> finalTrajPosY;
        QVector<QVariant> finalTrajPosZ;

        QVector<QVariant> finalTrajOrientX;
        QVector<QVariant> finalTrajOrientY;
        QVector<QVariant> finalTrajOrientZ;

        finalTimes.reserve(node->nodes.size());

        finalTrajPosX.reserve(node->nodes.size());
        finalTrajPosY.reserve(node->nodes.size());
        finalTrajPosZ.reserve(node->nodes.size());

        finalTrajOrientX.reserve(node->nodes.size());
        finalTrajOrientY.reserve(node->nodes.size());
        finalTrajOrientZ.reserve(node->nodes.size());

        for (ModularSBASolver::TrajectoryPoseNode const& pose : node->nodes) {

            finalTimes.push_back(pose.time);

            finalTrajPosX.push_back(pose.t[0]);
            finalTrajPosY.push_back(pose.t[1]);
            finalTrajPosZ.push_back(pose.t[2]);

            finalTrajOrientX.push_back(pose.rAxis[0]);
            finalTrajOrientY.push_back(pose.rAxis[1]);
            finalTrajOrientZ.push_back(pose.rAxis[2]);
        }

        QMap<QString, QVector<QVariant>> data;

        data.insert(Trajectory::OptDataTimeHeader, finalTimes);

        data.insert(Trajectory::OptDataPosXHeader, finalTrajPosX);
        data.insert(Trajectory::OptDataPosYHeader, finalTrajPosY);
        data.insert(Trajectory::OptDataPosZHeader, finalTrajPosZ);

        data.insert(Trajectory::OptDataRotXHeader, finalTrajOrientX);
        data.insert(Trajectory::OptDataRotYHeader, finalTrajOrientY);
        data.insert(Trajectory::OptDataRotZHeader, finalTrajOrientZ);

        resultDataTable->setData(data);

    }

    return true;

}

bool TrajectoryBaseSBAModule::writeUncertainty(ModularSBASolver* solver) {
    return true;
}

void TrajectoryBaseSBAModule::cleanup(ModularSBASolver* solver) {
    Q_UNUSED(solver);
    return;
}

} // namespace StereoVisionApp
