#include "trajectorybasesbamodule.h"

#include "datablocks/trajectory.h"
#include "datablocks/datatable.h"

#include "costfunctors/fixedsizenormalprior.h"

#include "../costfunctors/interpolatedvectorprior.h"
#include "../costfunctors/imustepcost.h"
#include "./costfunctors/weightedcostfunction.h"
#include "./costfunctors/posedecoratorfunctors.h"

#include "../sbagraphreductor.h"

#include "utils/statusoptionalreturn.h"

namespace StereoVisionApp {

const char* TrajectoryBaseSBAModule::ModuleName = "SBAModule::Trajectory";

TrajectoryBaseSBAModule::TrajectoryBaseSBAModule(double defaultIntegrationTime) :
    _defaultIntegrationTime(defaultIntegrationTime)
{

    _useOrientationPriors = true;

    _accelerometersBiases = std::vector<std::array<double,3>>();
    _accelerometersScales = std::vector<std::array<double,3>>();

    _gyrosBiases = std::vector<std::array<double,3>>();
    _gyrosScales = std::vector<std::array<double,3>>();

    _defaultGpsAccuracy = 1;
    _defaultOrientAccuracy = 1;

    _defaultAccAccuracy = 1;
    _defaultGyroAccuracy = 1;

    _gravity = {0,0,0};
}

QString TrajectoryBaseSBAModule::moduleName() const {
    return QObject::tr("Trajectory Base SBA Module");
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

bool TrajectoryBaseSBAModule::setupParameters(ModularSBASolver* solver) {

    Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    _gravity = {0,0,-9.81};

    if (currentProject->hasLocalCoordinateFrame()) {
        StereoVision::Geometry::AffineTransform<double> ecef2local = currentProject->ecef2local();
        Eigen::Vector3d ecefPos = -ecef2local.R.transpose()*ecef2local.t;

        PJ_CONTEXT* ctx = proj_context_create();

        if (ctx != nullptr) {
            const char* ecefCartesian = "EPSG:4978";
            const char* wgs84Ellipsoid = "EPSG:4979";
            PJ* projection = proj_create_crs_to_crs(
                ctx, ecefCartesian, wgs84Ellipsoid,
                nullptr);

            if (projection != nullptr) {

                PJ_COORD localFrameOriginECEF = proj_coord(ecefPos.x(), ecefPos.y(), ecefPos.z(), 0);
                PJ_COORD localFrameOriginWGS84 = proj_trans(projection, PJ_FWD, localFrameOriginECEF);
                PJ_COORD localFrameUpWGS84 = localFrameOriginWGS84;
                localFrameUpWGS84.lpz.z += 9.81;
                PJ_COORD localFrameUpECEF = proj_trans(projection, PJ_INV, localFrameUpWGS84);

                Eigen::Vector3d upEcef(localFrameUpECEF.xyz.x, localFrameUpECEF.xyz.y, localFrameUpECEF.xyz.z);
                Eigen::Vector3d gravityLocal = ecef2local*upEcef;

                _gravity = {-gravityLocal.x(), -gravityLocal.y(), -gravityLocal.z()};

                proj_destroy(projection);
            }

            proj_context_destroy(ctx);
        }
    }

    QVector<qint64> trajectoriesIdxs = currentProject->getIdsByClass(Trajectory::staticMetaObject.className());

    if (trajectoriesIdxs.isEmpty()) {
        return true;
    }

    _accelerometersBiases.reserve(trajectoriesIdxs.size());
    _accelerometersScales.reserve(trajectoriesIdxs.size());

    _gyrosBiases.reserve(trajectoriesIdxs.size());
    _gyrosScales.reserve(trajectoriesIdxs.size());

    for (qint64 trajId : trajectoriesIdxs) {

        Trajectory* traj = currentProject->getDataBlock<Trajectory>(trajId);

        if (traj == nullptr) {
            continue;
        }

        if (!traj->isEnabled()) {
            continue;
        }

        int accId = traj->accelerometerId();
        int gyroId = traj->gyroId();

        bool accelerometerBias = traj->estAccelerometerBias();
        bool accelerometerScale = traj->estAccelerometerScale();

        bool gyroBias = traj->estGyroBias();
        bool gyroScale = traj->estGyroScale();

        if (!_accelerometerParametersIndex.contains(accId)) {
            if (accelerometerBias or accelerometerScale) {

                int id = _accelerometersBiases.size();

                if (accelerometerScale) {
                    id = _accelerometersScales.size();
                }

                _accelerometerParametersIndex.insert(accId, id);

                if (accelerometerBias) {
                    _accelerometersBiases.push_back({0,0,0});

                }
                if (accelerometerScale) {
                    _accelerometersScales.push_back({1,1,1});
                }
            } else {
                _accelerometerParametersIndex.insert(accId, accId);
            }
        }

        if (!_gyroParametersIndex.contains(gyroId)) {
            if (gyroBias or gyroScale) {

                int id = _gyrosBiases.size();

                if (gyroScale) {
                    id = _gyrosScales.size();
                }

                _gyroParametersIndex.insert(gyroId, id);

                if (gyroBias) {
                    _gyrosBiases.push_back({0,0,0});

                }
                if (gyroScale) {
                    _gyrosScales.push_back({1,1,1});
                }
            } else {
                _gyroParametersIndex.insert(gyroId, gyroId);
            }
        }

        ModularSBASolver::TrajectoryNode* trajNode = solver->getNodeForTrajectory(trajId, true);

        if (trajNode == nullptr) {
            continue;
        }

        qint64 gpsMountingId = traj->gpsMountingId();
        qint64 insMountingId = traj->insMountingId();

        StereoVisionApp::ModularSBASolver::PoseNode* gpsMountingNode = solver->getNodeForMounting(gpsMountingId, true);
        StereoVisionApp::ModularSBASolver::PoseNode* insMountingNode = solver->getNodeForMounting(insMountingId, true);

        //no mounting node
        if (gpsMountingNode == nullptr) {
            QString message = QObject::tr("[Info] Traj %1 (\"%2\") do not have a GPS mounting configured!")
                                  .arg(trajId)
                                  .arg(traj->objectName());

            solver->logMessage(message);
        }
        if (insMountingNode == nullptr) {
            QString message = QObject::tr("[Info] Traj %1 (\"%2\") do not have an INS mounting configured!")
                                  .arg(trajId)
                                  .arg(traj->objectName());

            solver->logMessage(message);
        }

    }

    return true;

}

bool TrajectoryBaseSBAModule::init(ModularSBASolver* solver, ceres::Problem & problem) {

    Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> trajectoriesIdxs = currentProject->getIdsByClass(Trajectory::staticMetaObject.className());

    if (trajectoriesIdxs.isEmpty()) {
        return true;
    }

    problem.AddParameterBlock(_gravity.data(), _gravity.size());
    problem.SetParameterBlockConstant(_gravity.data());

   Eigen::Matrix3d gInfos = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; i++) {
        gInfos(i,i) = 1;
    }
    Eigen::Vector3d gVec;
    gVec << _gravity[0], _gravity[1], _gravity[2];

    if (currentProject->hasLocalCoordinateFrame()) { //georeferenced optimization disabled.
        _earth_center_pos = currentProject->ecef2local().t;
    } else {
        _earth_center_pos << std::nan(""), std::nan(""), std::nan("");
    }

    FixedSizeNormalPrior<3,3>* g_prior = new FixedSizeNormalPrior<3,3>(gInfos, gVec);

    problem.AddResidualBlock(g_prior, nullptr, _gravity.data());

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
            continue;
        }

        accId = _accelerometerParametersIndex[accId];

        bool accelerometerBias = traj->estAccelerometerBias();
        bool accelerometerScale = traj->estAccelerometerScale();

        bool gyroBias = traj->estGyroBias();
        bool gyroScale = traj->estGyroScale();

        if (accelerometerBias) {
            problem.AddParameterBlock(_accelerometersBiases[accId].data(), _accelerometersBiases[accId].size());
        }

        if (accelerometerScale) {
            problem.AddParameterBlock(_accelerometersScales[accId].data(), _accelerometersScales[accId].size());
        }

        int gyroId = traj->gyroId();

        if (!_gyroParametersIndex.contains(gyroId)) {
            continue;
        }

        gyroId = _gyroParametersIndex[gyroId];

        if (gyroBias) {
            problem.AddParameterBlock(_gyrosBiases[gyroId].data(), _gyrosBiases[gyroId].size());
        }

        if ( gyroScale) {
            problem.AddParameterBlock(_gyrosScales[gyroId].data(), _gyrosScales[gyroId].size());
        }

        //loading the trajectory data is really slow, so we setup the nodes at the same time we add them to the problem.
        //this just force the trajectory module to be run first.
        //TODO: find a way to streamline this.

        ModularSBASolver::TrajectoryNode* trajNode = solver->getNodeForTrajectory(trajId, true);

        if (trajNode == nullptr) {
            continue;
        }

        StereoVision::Geometry::AffineTransform<double> world2local = solver->getTransform2LocalFrame();

        StatusOptionalReturn<Trajectory::GpsData> optGpsData = traj->loadGpsSequences();
        StatusOptionalReturn<Trajectory::TimeCartesianSequence> optGps;

        if (optGpsData.isValid()) {
            Trajectory::GpsData& gpsData = optGpsData.value();

            if (gpsData.position.has_value()) {
                optGps = std::move(gpsData.position.value());
            }
        }

        if (!optGps.isValid()) {
            optGps = traj->loadPositionSequence(); //load in ecef
        }

        StatusOptionalReturn<Trajectory::TimeTrajectorySequence> optPose = traj->loadTrajectorySequence(); //used for initialization

        //TODO: make initialization possible from just GPS + gyro

        if (!optGps.isValid() and !optPose.isValid()) { //Canot initialize without GPS and orientation //TODO: investigate if we can initialize with
            continue;
        }

        trajNode->initialTrajectory = optPose.value();

        StatusOptionalReturn<Trajectory::TimeCartesianSequence> optGyro = traj->loadAngularSpeedSequence();
        StatusOptionalReturn<Trajectory::TimeCartesianSequence> optImu = traj->loadAccelerationSequence();

        double minTime;
        double maxTime;

        if (optGps.isValid()) {
            minTime = optGps.value().sequenceStartTime();
            maxTime = optGps.value().sequenceEndTime();

            if (optPose.isValid()) {
                minTime = std::max(minTime, optPose.value().sequenceStartTime());
                maxTime = std::min(maxTime, optPose.value().sequenceEndTime());
            }
        } else {
            minTime = optPose.value().sequenceStartTime();
            maxTime = optPose.value().sequenceEndTime();
        }

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

        double integrationTime = traj->getPreIntegrationTime();

        if (integrationTime <= 0) {
            integrationTime = _defaultIntegrationTime; //replace with default
        }

        int nSteps = std::ceil(duration/integrationTime)+1;
        double stepTime = duration/(nSteps-1);

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

            auto interpolatablePose = optPose.value().getValueAtTime(time); //plateform to world.
            StereoVision::Geometry::RigidBodyTransform<double> interpolated =
                    StereoVision::Geometry::interpolateRigidBodyTransformOnManifold(
                        interpolatablePose.weigthLower,
                        interpolatablePose.valLower,
                        interpolatablePose.weigthUpper,
                        interpolatablePose.valUpper);

            Eigen::Vector3d interpolatedPos =
                interpolatablePose.weigthLower*interpolatablePose.valLower.t +
                interpolatablePose.weigthUpper*interpolatablePose.valUpper.t; //use the reference trajectory as init
            Eigen::Vector3d pos = world2local*interpolatedPos;

            Eigen::Matrix3d R = world2local.R*StereoVision::Geometry::rodriguezFormula<double>(interpolated.r);
            Eigen::Vector3d rot = StereoVision::Geometry::inverseRodriguezFormula<double>(R);

            trajNode->nodes[i].t = {pos[0], pos[1], pos[2]};
            trajNode->nodes[i].rAxis = {rot[0], rot[1], rot[2]};

#ifndef NDEBUG
            bool posFinite = pos.allFinite();
            bool rotFinite = rot.allFinite();

            if (!posFinite or !rotFinite) {
                std::cerr << "Error while initializing trajectory" << traj->objectName().toStdString() << " at node " << i << std::endl;
            }
#endif

            problem.AddParameterBlock(trajNode->nodes[i].t.data(), trajNode->nodes[i].t.size());
            problem.AddParameterBlock(trajNode->nodes[i].rAxis.data(), trajNode->nodes[i].rAxis.size());

            bool addLogger = ((i+1)%trajLoggersSteps == 0);

            if (traj->isFixed()) {
                problem.SetParameterBlockConstant(trajNode->nodes[i].t.data());
                problem.SetParameterBlockConstant(trajNode->nodes[i].rAxis.data());

                continue;

            } else if (addLogger) { //if trajectory is not fixed, add the logger for the parameters value.

                QString posLoggerName = QString("Trajectory \"%1\" Position index %2 (t = %3)").arg(traj->objectName()).arg(i).arg(trajNode->nodes[i].time, 0, 'f', 2);
                solver->addLogger(posLoggerName, new ModularSBASolver::ParamsValsLogger<3>(trajNode->nodes[i].t.data()));

                QString rotLoggerName = QString("Trajectory \"%1\" Orientation index %2 (t = %3)").arg(traj->objectName()).arg(i).arg(trajNode->nodes[i].time, 0, 'f', 2);
                solver->addLogger(rotLoggerName, new ModularSBASolver::ParamsValsLogger<3>(trajNode->nodes[i].rAxis.data()));
            }

            double orientationAccuracy = _defaultOrientAccuracy;

            //orientation priors
            if (_useOrientationPriors and traj->useStartEndOrientationPrior()) {

                if (i == 0 or i == nSteps-1) {//add the priors only at beginning and end of sequence

                    Eigen::Matrix3d infos = Eigen::Matrix3d::Zero();
                    Eigen::Vector3d vec;

                    //position in local optimization frame
                    vec << trajNode->nodes[i].rAxis[0],trajNode->nodes[i].rAxis[1], trajNode->nodes[i].rAxis[2];

                    for (int i = 0; i < 3; i++) {
                        infos(i,i) = 100/orientationAccuracy;
                    }

                    ModularSBASolver::AutoErrorBlockLogger<1,3>::ParamsType params = {trajNode->nodes[i].rAxis.data()};

                    FixedSizeNormalPrior<3,3>* orientationPrior = new FixedSizeNormalPrior<3,3>(infos, vec);
                    FixedSizeNormalPrior<3,3>* orientationPriorError = new FixedSizeNormalPrior<3,3>(Eigen::Matrix3d::Identity(), vec);

                    problem.AddResidualBlock(orientationPrior, nullptr,
                                             params.data(),
                                             params.size());

                    QString loggerName = QString("Trajectory \"%1\" Orientation Prior index %2").arg(traj->objectName()).arg(i, nAlignChar);
                    solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<1,3>(orientationPriorError, params, true));

                }

            }

            double gpsAccuracy = traj->getGpsAccuracy();

            if (gpsAccuracy <= 0) {
                gpsAccuracy = _defaultGpsAccuracy;
            }

            if (i == 0) { //need at least two nodes for first order cost functions
                continue;
            }

            //gps observations
            if (i > 0 and gpsAccuracy > 1e-5) {

                double t1 = trajNode->nodes[i-1].time;
                double t2 = trajNode->nodes[i].time;

                for (int g = currentGPSNode; g < nGPSNode; g++) {
                    if (optGps.value()[g].time >= t1) {
                        currentGPSNode = g;
                        break;
                    }
                }

                double gpsObs_t = optGps.value()[currentGPSNode].time;

                if (gpsObs_t <= t2) { // GPS observation between  trajectory nodes

                    addGpsObs(
                        trajNode,
                        traj,
                        optGps.value(),
                        i,
                        t1,
                        t2,
                        gpsObs_t,
                        gpsAccuracy,
                        currentGPSNode,
                        world2local,
                        problem,
                        solver,
                        addLogger);
                }

            }

            //gyro cost

            double gyroAccuracy = traj->getGyroAccuracy();

            if (gyroAccuracy <= 0) {
                gyroAccuracy = _defaultGyroAccuracy;
            }

            if (optGyro.isValid()) {

                addGyroObs(trajNode,
                           traj,
                           gyroId,
                           i,
                           time,
                           optGyro.value(),
                           gyroAccuracy,
                           gyroBias,
                           gyroScale,
                           problem,
                           solver,
                           addLogger);
            }

            //accelerometer cost

            double accAccuracy = traj->getAccAccuracy();

            if (accAccuracy <= 0) {
                accAccuracy = _defaultAccAccuracy;
            }

            if (optGyro.isValid() and optImu.isValid() and i > 1) { //need at least three nodes for second order cost function

                addAccObs( //TODO: investigate why this function is slow!
                    trajNode,
                    traj,
                    gyroId,
                    accId,
                    i,
                    time,
                    optGyro.value(),
                    optImu.value(),
                    accAccuracy,
                    gyroBias,
                    gyroScale,
                    accelerometerBias,
                    accelerometerScale,
                    problem,
                    solver,
                    addLogger);

            }

        }
    }

    return true;

}

bool TrajectoryBaseSBAModule::writeResults(ModularSBASolver* solver) {

    QTextStream out(stdout);

    if (!solver->isSilent()) {
        out << "Optimized gravity: " << _gravity[0] << " " << _gravity[1] << " " << _gravity[2] << Qt::endl;
    }

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

        bool accelerometerBias = trajectory->estAccelerometerBias();
        bool accelerometerScale = trajectory->estAccelerometerScale();

        bool gyroBias = trajectory->estGyroBias();
        bool gyroScale = trajectory->estGyroScale();

        int accId = trajectory->accelerometerId();

        if (_accelerometerParametersIndex.contains(accId)) {
            int paramIdx = _accelerometerParametersIndex[accId];

            if (accelerometerBias) {

                std::array<double,3> const& bias = _accelerometersBiases[paramIdx];

                trajectory->setOptAccelerometerBiasX(bias[0]);
                trajectory->setOptAccelerometerBiasY(bias[1]);
                trajectory->setOptAccelerometerBiasZ(bias[2]);
            }

            if (accelerometerScale) {

                std::array<double,3> const& scale = _accelerometersScales[paramIdx];

                trajectory->setOptAccelerometerScaleX(scale[0]);
                trajectory->setOptAccelerometerScaleY(scale[1]);
                trajectory->setOptAccelerometerScaleZ(scale[2]);
            }
        }

        int gyroId = trajectory->gyroId();

        if (_gyroParametersIndex.contains(gyroId)) {
            int paramIdx = _gyroParametersIndex[gyroId];

            if (gyroBias) {

                std::array<double,3> const& bias = _gyrosBiases[paramIdx];

                trajectory->setOptGyroBiasX(bias[0]);
                trajectory->setOptGyroBiasY(bias[1]);
                trajectory->setOptGyroBiasZ(bias[2]);
            }

            if (gyroScale) {

                std::array<double,3> const& scale = _gyrosScales[paramIdx];

                trajectory->setOptGyroScaleX(scale[0]);
                trajectory->setOptGyroScaleY(scale[1]);
                trajectory->setOptGyroScaleZ(scale[2]);
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

StatusOptionalReturn<void> TrajectoryBaseSBAModule::addGpsObs(
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
    bool addLogger) {

    double w1 = (t2 - gpsObs_t)/(t2-t1);
    double w2 = (gpsObs_t-t1)/(t2-t1);

    Eigen::Matrix3d infos = Eigen::Matrix3d::Zero();
    Eigen::Vector3d vec;

    //position in local optimization frame
    vec = world2local*gpsSeq[currentGPSNode].val;

    for (int i = 0; i < 3; i++) {
        infos(i,i) = 1/gpsAccuracy;
    }

    int gpsMountingId = traj->gpsMountingId();

    ModularSBASolver::PoseNode* gpsMountingNode = solver->getNodeForMounting(gpsMountingId, false);


    //add gps based trajectory priors
    if (gpsMountingNode != nullptr) {

        constexpr int PoseConfig = Body2World | Sensor2Body;
        using LeverArmGPSPrior = ApplyLeverArm<
            AddPose<
                AddOrientation<
                    FixedSizeNormalCostFunctor<3,3>
                    ,0>
                , 0>
            , 0,2, PoseConfig>;

        using InterpolatedLeverArmGPSPrior =
            ApplyLeverArm<
            ApplyLeverArm<
                AddPose<
                    AddOrientation<
                        AddOrientation<
                            InterpolatedVectorPrior<3>
                            ,1>
                        ,0>
                    , 0>
                , 0,2, PoseConfig>
            , 0,4, PoseConfig>;

        if (std::abs(w1-1) < 1e-3 or std::abs(w2-1) < 1e-3) {

            LeverArmGPSPrior* gpsPriorCost = new LeverArmGPSPrior(infos, vec);
            LeverArmGPSPrior* gpsPriorErrorCost = new LeverArmGPSPrior(Eigen::Matrix3d::Identity(), vec);

            ceres::AutoDiffCostFunction<LeverArmGPSPrior, 3,3,3,3,3>* gpsPrior = new
                ceres::AutoDiffCostFunction<LeverArmGPSPrior, 3,3,3,3,3>(gpsPriorCost);

            ceres::AutoDiffCostFunction<LeverArmGPSPrior, 3,3,3,3,3>* gpsPriorError = new
                ceres::AutoDiffCostFunction<LeverArmGPSPrior, 3,3,3,3,3>(gpsPriorErrorCost);

            if (std::abs(w1-1) < 1e-3 or std::abs(w2-1) < 1e-3) {

                ModularSBASolver::AutoErrorBlockLogger<4,3>::ParamsType params;

                if (std::abs(w1-1) < 1e-3) {
                    params = {gpsMountingNode->rAxis.data(),
                              gpsMountingNode->t.data(),
                              trajNode->nodes[i-1].rAxis.data(),
                              trajNode->nodes[i-1].t.data()};
                } else if (std::abs(w2-1) < 1e-3) {
                    params = {gpsMountingNode->rAxis.data(),
                              gpsMountingNode->t.data(),
                              trajNode->nodes[i].rAxis.data(),
                              trajNode->nodes[i].t.data()};
                }

                problem.AddResidualBlock(gpsPrior, nullptr,
                                         params.data(),
                                         params.size());


                if (addLogger) {

                    QString loggerName = QString("GPS trajectory \"%1\" time %2").arg(traj->objectName()).arg(gpsObs_t, 0, 'f', 2);
                    solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<4,3>(gpsPriorError, params, true));
                }


            } else {
                delete gpsPrior; //useless as branch is unreachable, but remove static analysis error
                delete gpsPriorError;
            }

        } else {
            InterpolatedLeverArmGPSPrior* interpolatedPriorCost = new InterpolatedLeverArmGPSPrior(vec, w1, w2, infos);
            InterpolatedLeverArmGPSPrior* interpolatedPriorErrorCost = new InterpolatedLeverArmGPSPrior(vec, w1, w2, Eigen::Matrix3d::Identity());

            ceres::AutoDiffCostFunction<InterpolatedLeverArmGPSPrior, 3,3,3,3,3,3,3>*  interpolatedPrior =
                new ceres::AutoDiffCostFunction<InterpolatedLeverArmGPSPrior, 3,3,3,3,3,3,3>(interpolatedPriorCost);
            ceres::AutoDiffCostFunction<InterpolatedLeverArmGPSPrior, 3,3,3,3,3,3,3>*  interpolatedError =
                new ceres::AutoDiffCostFunction<InterpolatedLeverArmGPSPrior, 3,3,3,3,3,3,3>(interpolatedPriorErrorCost);


            ModularSBASolver::AutoErrorBlockLogger<6,3>::ParamsType params = {gpsMountingNode->rAxis.data(),
                                                                               gpsMountingNode->t.data(),
                                                                               trajNode->nodes[i-1].rAxis.data(),
                                                                               trajNode->nodes[i-1].t.data(),
                                                                               trajNode->nodes[i].rAxis.data(),
                                                                               trajNode->nodes[i].t.data()};

            problem.AddResidualBlock(interpolatedPrior, nullptr,
                                     params.data(),
                                     params.size());

            if (addLogger) {

                QString loggerName = QString("GPS trajectory \"%1\" time %2 (interpolated)").arg(traj->objectName()).arg(gpsObs_t, 0, 'f', 2);
                solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<6,3>(interpolatedError, params, true));
            }

        }

    } else {

        if (std::abs(w1-1) < 1e-3 or std::abs(w2-1) < 1e-3) {

            FixedSizeNormalPrior<3,3>* gpsPrior = new FixedSizeNormalPrior<3,3>(infos, vec);
            FixedSizeNormalPrior<3,3>* gpsPriorError = new FixedSizeNormalPrior<3,3>(Eigen::Matrix3d::Identity(), vec);

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


                if (addLogger) {

                    QString loggerName = QString("GPS trajectory \"%1\" time %2").arg(traj->objectName()).arg(gpsObs_t, 0, 'f', 2);
                    solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<1,3>(gpsPriorError, params, true));
                }


            } else {
                delete gpsPrior; //useless as branch is unreachable, but remove static analysis error
            }

        } else {
            InterpolatedVectorPrior<3>* interpolatedPriorCost = new InterpolatedVectorPrior<3>(vec, w1, w2, infos);
            InterpolatedVectorPrior<3>* interpolatedPriorError = new InterpolatedVectorPrior<3>(vec, w1, w2, Eigen::Matrix3d::Identity());

            ceres::AutoDiffCostFunction<InterpolatedVectorPrior<3>, 3,3,3>*  interpolatedPrior =
                new ceres::AutoDiffCostFunction<InterpolatedVectorPrior<3>, 3,3,3>(interpolatedPriorCost);
            ceres::AutoDiffCostFunction<InterpolatedVectorPrior<3>, 3,3,3>*  interpolatedError =
                new ceres::AutoDiffCostFunction<InterpolatedVectorPrior<3>, 3,3,3>(interpolatedPriorError);


            ModularSBASolver::AutoErrorBlockLogger<2,3>::ParamsType params = {trajNode->nodes[i-1].t.data(),
                                                                               trajNode->nodes[i].t.data()};

            problem.AddResidualBlock(interpolatedPrior, nullptr,
                                     params.data(),
                                     params.size());

            if (addLogger) {

                QString loggerName = QString("GPS trajectory \"%1\" time %2 (interpolated)").arg(traj->objectName()).arg(gpsObs_t, 0, 'f', 2);
                solver->addLogger(loggerName, new ModularSBASolver::AutoErrorBlockLogger<2,3>(interpolatedError, params, true));
            }

        }
    }

    return StatusOptionalReturn<void>();
}
StatusOptionalReturn<void> TrajectoryBaseSBAModule::addGyroObs(
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
    bool addLogger) {

    constexpr bool WBias = true;
    constexpr bool WoBias = false;

    constexpr bool WScale = true;
    constexpr bool WoScale = false;

    if (gyroBias and gyroScale) {

        return addTempltGyroObs<WBias, WScale>(trajNode, traj, gyroId,
                                               i, time, gyroSeq, gyroAccuracy,
                                               problem, solver, addLogger);

    } else if (gyroBias) {

        return addTempltGyroObs<WBias, WoScale>(trajNode, traj, gyroId,
                                               i, time, gyroSeq, gyroAccuracy,
                                               problem, solver, addLogger);

    } else if (gyroScale) {

        return addTempltGyroObs<WoBias, WScale>(trajNode, traj, gyroId,
                                               i, time, gyroSeq, gyroAccuracy,
                                               problem, solver, addLogger);

    } else {

        return addTempltGyroObs<WoBias, WoScale>(trajNode, traj, gyroId,
                                               i, time, gyroSeq, gyroAccuracy,
                                               problem, solver, addLogger);
    }

    return StatusOptionalReturn<void>();
}



StatusOptionalReturn<void> TrajectoryBaseSBAModule::addAccObs(
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
    bool addLogger) {

    if (gyroBias and gyroScale and accelerometerBias and accelerometerScale) {

        constexpr int flags = AccelerometerStepCostFlags::GyroBias |
                              AccelerometerStepCostFlags::GyroScale |
                              AccelerometerStepCostFlags::AccBias |
                              AccelerometerStepCostFlags::AccScale;

        return addTempltAccObs<flags>(trajNode, traj, gyroId, accId,
                                      i, time, gyroSeq, imuSeq, accAccuracy,
                                      problem, solver, addLogger);

    } else if (gyroBias and gyroScale and accelerometerBias) {

        constexpr int flags = AccelerometerStepCostFlags::GyroBias |
                              AccelerometerStepCostFlags::GyroScale |
                              AccelerometerStepCostFlags::AccBias;

        return addTempltAccObs<flags>(trajNode, traj, gyroId, accId,
                                      i, time, gyroSeq, imuSeq, accAccuracy,
                                      problem, solver, addLogger);

    } else if (gyroBias and gyroScale and accelerometerScale) {

        constexpr int flags = AccelerometerStepCostFlags::GyroBias |
                              AccelerometerStepCostFlags::GyroScale |
                              AccelerometerStepCostFlags::AccScale;

        return addTempltAccObs<flags>(trajNode, traj, gyroId, accId,
                                      i, time, gyroSeq, imuSeq, accAccuracy,
                                      problem, solver, addLogger);

    } else if (gyroBias and gyroScale) {

        constexpr int flags = AccelerometerStepCostFlags::GyroBias |
                              AccelerometerStepCostFlags::GyroScale;

        return addTempltAccObs<flags>(trajNode, traj, gyroId, accId,
                                      i, time, gyroSeq, imuSeq, accAccuracy,
                                      problem, solver, addLogger);

    } else if (gyroBias and accelerometerBias and accelerometerScale) {

        constexpr int flags = AccelerometerStepCostFlags::GyroBias |
                              AccelerometerStepCostFlags::AccBias |
                              AccelerometerStepCostFlags::AccScale;

        return addTempltAccObs<flags>(trajNode, traj, gyroId, accId,
                                      i, time, gyroSeq, imuSeq, accAccuracy,
                                      problem, solver, addLogger);

    } else if (gyroBias and accelerometerBias) {

        constexpr int flags = AccelerometerStepCostFlags::GyroBias |
                              AccelerometerStepCostFlags::AccBias;

        return addTempltAccObs<flags>(trajNode, traj, gyroId, accId,
                                      i, time, gyroSeq, imuSeq, accAccuracy,
                                      problem, solver, addLogger);

    } else if (gyroBias and accelerometerScale) {

        constexpr int flags = AccelerometerStepCostFlags::GyroBias |
                              AccelerometerStepCostFlags::AccScale;

        return addTempltAccObs<flags>(trajNode, traj, gyroId, accId,
                                      i, time, gyroSeq, imuSeq, accAccuracy,
                                      problem, solver, addLogger);

    } else if (gyroBias) {

        constexpr int flags = AccelerometerStepCostFlags::GyroBias;

        return addTempltAccObs<flags>(trajNode, traj, gyroId, accId,
                                      i, time, gyroSeq, imuSeq, accAccuracy,
                                      problem, solver, addLogger);

    } else if (gyroScale and accelerometerBias and accelerometerScale) {

        constexpr int flags = AccelerometerStepCostFlags::GyroScale |
                              AccelerometerStepCostFlags::AccBias |
                              AccelerometerStepCostFlags::AccScale;

        return addTempltAccObs<flags>(trajNode, traj, gyroId, accId,
                                      i, time, gyroSeq, imuSeq, accAccuracy,
                                      problem, solver, addLogger);

    } else if (gyroScale and accelerometerBias) {

        constexpr int flags = AccelerometerStepCostFlags::GyroScale |
                              AccelerometerStepCostFlags::AccBias;

        return addTempltAccObs<flags>(trajNode, traj, gyroId, accId,
                                      i, time, gyroSeq, imuSeq, accAccuracy,
                                      problem, solver, addLogger);

    } else if (gyroScale and accelerometerScale) {

        constexpr int flags = AccelerometerStepCostFlags::GyroScale |
                              AccelerometerStepCostFlags::AccScale;

        return addTempltAccObs<flags>(trajNode, traj, gyroId, accId,
                                      i, time, gyroSeq, imuSeq, accAccuracy,
                                      problem, solver, addLogger);

    } else if (gyroScale) {

        constexpr int flags = AccelerometerStepCostFlags::GyroScale;

        return addTempltAccObs<flags>(trajNode, traj, gyroId, accId,
                                      i, time, gyroSeq, imuSeq, accAccuracy,
                                      problem, solver, addLogger);

    } else if (accelerometerBias and accelerometerScale) {

        constexpr int flags = AccelerometerStepCostFlags::AccBias |
                              AccelerometerStepCostFlags::AccScale;

        return addTempltAccObs<flags>(trajNode, traj, gyroId, accId,
                                      i, time, gyroSeq, imuSeq, accAccuracy,
                                      problem, solver, addLogger);

    } else if (accelerometerBias) {

        constexpr int flags = AccelerometerStepCostFlags::AccBias;

        return addTempltAccObs<flags>(trajNode, traj, gyroId, accId,
                                      i, time, gyroSeq, imuSeq, accAccuracy,
                                      problem, solver, addLogger);

    } else if (accelerometerScale) {

        constexpr int flags = AccelerometerStepCostFlags::AccScale;

        return addTempltAccObs<flags>(trajNode, traj, gyroId, accId,
                                      i, time, gyroSeq, imuSeq, accAccuracy,
                                      problem, solver, addLogger);

    } else {

        constexpr int flags = AccelerometerStepCostFlags::NoBias;

        return addTempltAccObs<flags>(trajNode, traj, gyroId, accId,
                                      i, time, gyroSeq, imuSeq, accAccuracy,
                                      problem, solver, addLogger);
    }

    return StatusOptionalReturn<void>();

}

} // namespace StereoVisionApp
