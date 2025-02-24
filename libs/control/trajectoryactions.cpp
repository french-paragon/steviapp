#include "trajectoryactions.h"

#include "datablocks/trajectory.h"

#include "control/mainwindow.h"

#include "gui/editor.h"
#include "gui/sparsealignementeditor.h"

#include "gui/inputsWidgets/rigidbodytransforminputwidget.h"

#include "gui/opengl3dsceneviewwidget.h"
#include "gui/openGlDrawables/opengldrawabletrajectory.h"

#include "utils/statusoptionalreturn.h"

#include "vision/trajectoryImuPreIntegration.h"

#include "geo/localframes.h"

#include <proj.h>

#include <QMessageBox>
#include <QFileDialog>

namespace StereoVisionApp {

void viewTrajectory(Trajectory* traj, bool optimized) {

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (traj == nullptr) {
        return;
    }

    if (mw == nullptr) {
        return; //need main windows to display trajectory
    }

    StereoVisionApp::Editor* e = mw->openEditor(SparseAlignementEditor::staticMetaObject.className());

    SparseAlignementEditor* sae = qobject_cast<SparseAlignementEditor*>(e);

    if (sae == nullptr) {
        return;
    }

    QString traj_drawable_name = QString("TrajectoryView_%1").arg(traj->objectName());

    if (optimized) {
        traj_drawable_name = QString("TrajectoryViewOpt_%1").arg(traj->objectName());
    }

    OpenGlDrawable* drawable = sae->getDrawable(traj_drawable_name);
    OpenGlDrawableTrajectory* drawableTrajectory = qobject_cast<OpenGlDrawableTrajectory*>(drawable);

    if (drawable == nullptr) {
        drawableTrajectory = new OpenGlDrawableTrajectory();
        drawableTrajectory->setSceneScale(sae->sceneScale());
        drawableTrajectory->setHandleScale(sae->camScale());

        if (optimized) {
            drawableTrajectory->setBaseColor(QColor(180,200,10));

            drawableTrajectory->setOrientHandleColor(StereoVision::Geometry::Axis::X,
                                                     QColor(255, 0, 255));
            drawableTrajectory->setOrientHandleColor(StereoVision::Geometry::Axis::Y,
                                                     QColor(150, 255, 0));
            drawableTrajectory->setOrientHandleColor(StereoVision::Geometry::Axis::Z,
                                                     QColor(0, 255, 255));
        }

        QObject::connect(sae, &SparseAlignementEditor::sceneScaleChanged,
                         drawableTrajectory, &OpenGlDrawableTrajectory::setSceneScale);

        QObject::connect(sae, &SparseAlignementEditor::camScaleChanged,
                         drawableTrajectory, &OpenGlDrawableTrajectory::setHandleScale);

        sae->addDrawable(traj_drawable_name, drawableTrajectory);

    } else if (drawableTrajectory == nullptr) {
        //a drawable already exist with the name and is not a OpenGlDrawableTrajectory

        return; //error, this is not supposed to happen

    }

    StatusOptionalReturn<std::vector<StereoVision::Geometry::AffineTransform<float>>> trajDataOpt = traj->loadTrajectoryInProjectLocalFrame(optimized); //get the trajectory
    if (!trajDataOpt.isValid()) {
        QMessageBox::information(mw, QObject::tr("Error loading the trajectory"), trajDataOpt.errorMessage());
    }

    std::vector<StereoVision::Geometry::AffineTransform<float>>& trajData = trajDataOpt.value();

    constexpr int nOrientationHandles = 9;
    drawableTrajectory->setTrajectory(trajData, nOrientationHandles);

    QObject::connect(traj, &Trajectory::trajectoryDataChanged, drawableTrajectory, [drawableTrajectory, traj] () {
        drawableTrajectory->setTrajectory(traj);
    });
}

void setLeverArm(Trajectory* traj) {

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (traj == nullptr) {
        return;
    }

    if (mw == nullptr) {
        return; //need main windows to display trajectory
    }

}
void setAccelerometerMounting(Trajectory* traj) {

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (traj == nullptr) {
        return;
    }

    if (mw == nullptr) {
        return; //need main windows to display trajectory
    }

    auto mounting = traj->accelerometerMounting();

    auto transform = RigidBodyTransformInputDialog::inputAffineTransform(QObject::tr("Accelerometer mounting"), mw, mounting);

    if (!transform.has_value()) {
        return;
    }

    traj->setAccelerometerMounting(transform.value());

}
void setGyroMounting(Trajectory* traj) {

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (traj == nullptr) {
        return;
    }

    if (mw == nullptr) {
        return; //need main windows to display trajectory
    }

    auto mounting = traj->gyroMounting();

    auto transform = RigidBodyTransformInputDialog::inputAffineTransform(QObject::tr("Gyro mounting"), mw, mounting);

    if (!transform.has_value()) {
        return;
    }

    traj->setGyroMounting(transform.value());

}

void checkTrajectoryConsistency(Trajectory* traj) {

    QTextStream out(stdout);

    if (traj == nullptr) {
        out << "Analyzing trajectory nullptr (aborting)!" << Qt::endl;
        return;
    }

    out << "Analyzing trajectory " << traj->objectName() << ":" << Qt::endl;

    StatusOptionalReturn<Trajectory::TimeCartesianSequence> angularSpeedOpt = traj->loadAngularSpeedSequence();

    StatusOptionalReturn<Trajectory::TimeCartesianSequence> accelerometerOpt = traj->loadAccelerationSequence();

    StatusOptionalReturn<Trajectory::TimeTrajectorySequence> posesOpt = traj->loadTrajectoryProjectLocalFrameSequence();

    if (!posesOpt.isValid()) {
        out << "Could not load trajectory, aborting consistency analysis!" << Qt::endl;
        return;
    }

    if (!angularSpeedOpt.isValid()) {
        out << "Could not load angular speed, aborting consistency analysis!" << Qt::endl;
        return;
    }

    if (!accelerometerOpt.isValid()) {
        out << "Could not load accelerometer data, aborting consistency analysis!" << Qt::endl;
        return;
    }

    Trajectory::TimeCartesianSequence& angularSpeed = angularSpeedOpt.value();
    Trajectory::TimeCartesianSequence& accelerometer = accelerometerOpt.value();

    Trajectory::TimeTrajectorySequence& poses = posesOpt.value();

    out << "Data loaded!" << "\n";
    out << "\t" << "Poses: " << poses.nPoints() << " records loaded!" << "\n";
    out << "\t" << "Gyro: " << angularSpeed.nPoints() << " records loaded!" << "\n";
    out << "\t" << "Accelerometer: " << accelerometer.nPoints() << " records loaded!" << Qt::endl;

    if (poses.nPoints() < 2) {
        out << "Not enough samples in trajectory, aborting consistency analysis!" << Qt::endl;
        return;
    }

    if (angularSpeed.nPoints() < 2) {
        out << "Not enough samples in angular speed, aborting consistency analysis!" << Qt::endl;
        return;
    }

    if (accelerometer.nPoints() < 2) {
        out << "Not enough samples in accelerometer data, aborting consistency analysis!" << Qt::endl;
        return;
    }

    int trajRotNotFinitesRecords = 0;
    int trajPosNotFinitesRecords = 0;
    int gyroNotFiniteRecords = 0;
    int accelerometerNotFiniteRecords = 0;

    constexpr int nMaxNonFiniteIdxs = 10;

    std::vector<int> nonFiniteTrajRotIdx;
    std::vector<int> nonFiniteTrajPosIdx;
    std::vector<int> nonFiniteGyroIdx;
    std::vector<int> nonFiniteAccIdx;

    nonFiniteTrajRotIdx.reserve(nMaxNonFiniteIdxs);
    nonFiniteTrajPosIdx.reserve(nMaxNonFiniteIdxs);
    nonFiniteGyroIdx.reserve(nMaxNonFiniteIdxs);
    nonFiniteAccIdx.reserve(nMaxNonFiniteIdxs);

    for (int i = 0; i < poses.nPoints(); i++) {
        auto pose = poses[i];

        if (!pose.val.r.allFinite()) {
            trajRotNotFinitesRecords++;

            if (nonFiniteTrajRotIdx.size() < nMaxNonFiniteIdxs) {
                nonFiniteTrajRotIdx.push_back(i);
            }
        }

        if (!pose.val.t.allFinite()) {
            trajPosNotFinitesRecords++;

            if (nonFiniteTrajPosIdx.size() < nMaxNonFiniteIdxs) {
                nonFiniteTrajPosIdx.push_back(i);
            }
        }
    }

    for (int i = 0; i < angularSpeed.nPoints(); i++) {

        if (!angularSpeed[i].val.allFinite()) {
            gyroNotFiniteRecords++;

            if (nonFiniteGyroIdx.size() < nMaxNonFiniteIdxs) {
                nonFiniteGyroIdx.push_back(i);
            }
        }
    }

    for (int i = 0; i < accelerometer.nPoints(); i++) {

        if (!accelerometer[i].val.allFinite()) {
            accelerometerNotFiniteRecords++;

            if (nonFiniteAccIdx.size() < nMaxNonFiniteIdxs) {
                nonFiniteAccIdx.push_back(i);
            }
        }
    }

    if (trajPosNotFinitesRecords > 0 or trajRotNotFinitesRecords > 0 or gyroNotFiniteRecords > 0 or accelerometerNotFiniteRecords > 0) {
        out << "Non finite records detected!" << "\n";
        out << "\t" << "Position: " << trajPosNotFinitesRecords << " non finite records detected!" << "\n";

        if (nonFiniteTrajPosIdx.size() > 0) {
            out << "\t\t" << "First " << nonFiniteTrajPosIdx.size() << " non finite idxs:";
            for (int idx : nonFiniteTrajPosIdx) {
                out << " " << idx;
            }
            out << "\n";
        }

        out << "\t" << "Orientation: " << trajRotNotFinitesRecords << " non finite records detected!" << "\n";

        if (nonFiniteTrajRotIdx.size() > 0) {
            out << "\t\t" << "First " << nonFiniteTrajRotIdx.size() << " non finite idxs:";
            for (int idx : nonFiniteTrajRotIdx) {
                out << " " << idx;
            }
            out << "\n";
        }

        out << "\t" << "Gyro: " << gyroNotFiniteRecords << " non finite records detected!" << "\n";

        if (nonFiniteGyroIdx.size() > 0) {
            out << "\t\t" << "First " << nonFiniteGyroIdx.size() << " non finite idxs:";
            for (int idx : nonFiniteGyroIdx) {
                out << " " << idx;
            }
            out << "\n";
        }

        out << "\t" << "Accelerometer: " << accelerometerNotFiniteRecords << " non finite records detected!" << "\n";

        if (nonFiniteAccIdx.size() > 0) {
            out << "\t\t" << "First " << nonFiniteAccIdx.size() << " non finite idxs:";
            for (int idx : nonFiniteAccIdx) {
                out << " " << idx;
            }
            out << "\n";
        }

        out << "Aborting!" << Qt::endl;
        return;
    }

    double minIntegrationTime = 0.5; //min integration time of half a second

    double startTime = poses.sequenceStartTime();
    double endTime = poses.sequenceEndTime();

    int nExpectedErrors = std::ceil(1.1*(endTime - startTime)/minIntegrationTime);

    std::vector<Eigen::Vector3d> rErrors;
    std::vector<Eigen::Vector3d> aErrors;

    rErrors.reserve(nExpectedErrors);
    aErrors.reserve(nExpectedErrors);

    int deltai = 1;

    for (int i = 0; i < poses.nPoints(); i += deltai) {

        Trajectory::TimeTrajectorySequence::TimedElement body1_to_local = poses[i];

        deltai = 1;

        do {

            if (poses[i+deltai].time - body1_to_local.time >= minIntegrationTime) {
                break;
            }
            deltai++;
        } while (i+deltai < poses.nPoints());

        if (i+deltai >= poses.nPoints()) {
            break;
        }

        Trajectory::TimeTrajectorySequence::TimedElement body2_to_local = poses[i+deltai];

        StereoVision::Geometry::RigidBodyTransform<double> body1_to_body2 = body2_to_local.val.inverse()*body1_to_local.val;
        StereoVision::Geometry::AffineTransform<double> body1_to_body2Aff = body1_to_body2.toAffineTransform();

        Eigen::Matrix3d GyroR2to1 = PreIntegrateGyro(angularSpeed, body1_to_local.time, body2_to_local.time);

        Eigen::Matrix3d errorR = body1_to_body2Aff.R*GyroR2to1;

        rErrors.push_back(StereoVision::Geometry::inverseRodriguezFormula(errorR));

        if (i+2*deltai < poses.nPoints()) {
            Trajectory::TimeTrajectorySequence::TimedElement body3_to_local = poses[i+2*deltai];

            StereoVision::Geometry::RigidBodyTransform<double> body2_to_body1 = body1_to_body2.inverse();
            StereoVision::Geometry::RigidBodyTransform<double> body3_to_body1 = body1_to_local.val.inverse()*body3_to_local.val;

            Eigen::Vector3d speed1 = body2_to_body1.t / (body2_to_local.time - body1_to_local.time);
            Eigen::Vector3d speed2 = (body3_to_body1.t - body2_to_body1.t) / (body3_to_local.time - body2_to_local.time);

            double timespeed1 = (body2_to_local.time + body1_to_local.time)/2;
            double timespeed2 = (body3_to_local.time + body2_to_local.time)/2;

            //align with the initial point
            speed1 = StereoVision::Geometry::angleAxisRotate<double>(body1_to_body2.r*0.5, speed1);
            speed2 = StereoVision::Geometry::angleAxisRotate<double>(body1_to_body2.r*0.5, speed2);

            Eigen::Vector3d speedDeltaTrj = (speed2 - speed1);

            Eigen::Vector3d speedDeltaObs = PreIntegrateAccelerometer(accelerometer, angularSpeed, timespeed1, timespeed2);

            Eigen::Vector3d  accDelta = (speedDeltaObs - speedDeltaTrj)/(timespeed2 - timespeed1);

            aErrors.push_back(accDelta);

        }

    }

    Eigen::Vector3d meanRotError = Eigen::Vector3d::Zero();
    Eigen::Vector3d maxRotError = Eigen::Vector3d::Zero();
    double rootMeanSquaredRotNorm = 0;
    double maxRotNorm = 0;

    for (int i = 0; i < rErrors.size(); i++) {
        Eigen::Vector3d delta_r = rErrors[i];
        meanRotError += delta_r;

        for (int a = 0; a < 3; a++) {
            if (std::abs(delta_r[a]) > std::abs(maxRotError[a])) {
                maxRotError[a] = delta_r[a];
            }
        }

        double sqnorm = delta_r.squaredNorm();
        rootMeanSquaredRotNorm += sqnorm;

        if (sqnorm > maxRotNorm) {
            maxRotNorm = sqnorm;
        }
    }

    meanRotError /= rErrors.size();
    rootMeanSquaredRotNorm /= rErrors.size();

    rootMeanSquaredRotNorm = std::sqrt(rootMeanSquaredRotNorm);
    maxRotNorm = std::sqrt(maxRotNorm);

    out << "\t" << "Mean rotation speed error [axis angle]: " << meanRotError.x() << " " << meanRotError.y() << " " << meanRotError.z() << "\n";
    out << "\t" << "Max rotation speed error [axis angle]: " << maxRotError.x() << " " << maxRotError.y() << " " << maxRotError.z() << "\n";
    out << "\t" << "root mean squared rotation norm [axis angle]: " << rootMeanSquaredRotNorm << " and max rotation norm [axis angle]: " << maxRotNorm << Qt::endl;

    Eigen::Vector3d meanAccDelta = Eigen::Vector3d::Zero();

    for (int i = 0; i < aErrors.size(); i++) {
        meanAccDelta += aErrors[i];
    }

    meanAccDelta /= aErrors.size();

    out << "\t" << "Mean accelleration delta: " << meanAccDelta.x() << " " << meanAccDelta.y() << " " << meanAccDelta.z() << Qt::endl;

    Eigen::Vector3d stdAccDelta = Eigen::Vector3d::Zero();

    for (int i = 0; i < aErrors.size(); i++) {
        Eigen::Vector3d delta = aErrors[i] - meanAccDelta;
        delta[0] *= delta[0];
        delta[1] *= delta[1];
        delta[2] *= delta[2];
        stdAccDelta += delta;
    }

    stdAccDelta /= aErrors.size();

    out << "\t" << "Std accelleration delta: " << stdAccDelta.x() << " " << stdAccDelta.y() << " " << stdAccDelta.z() << Qt::endl;

}

void exportTrajectory(Trajectory* traj, QString filePath, bool exportOptimized) {

    if (traj == nullptr) {
        return;
    }

    Project* currentProject = traj->getProject();

    MainWindow* mw = MainWindow::getActiveMainWindow();

    QString outFilePath = filePath;

    if (outFilePath.isEmpty()) {

        if (mw == nullptr) {
            return; //need main windows to display a save file dialog
        }

        outFilePath = QFileDialog::getSaveFileName(mw, QObject::tr("Save trajectory to"));

        if (outFilePath.isEmpty()) {
            return;
        }
    }

    constexpr bool subsample = true;

    StatusOptionalReturn<Trajectory::TimeTrajectorySequence> exportTraj = 
        exportOptimized ? traj->optimizedTrajectory(subsample) : traj->loadTrajectorySequence();

    StereoVision::Geometry::AffineTransform<double> local2ecef(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

    if (exportOptimized) {        
        if (currentProject != nullptr) {
            StereoVision::Geometry::AffineTransform<double> ecef2local = currentProject->ecef2local().cast<double>();
            local2ecef.R = ecef2local.R.transpose();
            local2ecef.t = -ecef2local.R.transpose()*ecef2local.t;
        }
    }


    if (!exportTraj.isValid()) {
        if (mw != nullptr) {
            QMessageBox::warning(mw, QObject::tr("Error when exporting initial trajectory"), exportTraj.errorMessage());
        }
        return;
    }

    QFile outFile(outFilePath);

    bool ok = outFile.open(QFile::WriteOnly);

    if (!ok) {
        if (mw != nullptr) {
            QMessageBox::warning(mw,
                                 QObject::tr("Error when exporting optimized trajectory"),
                                 QObject::tr("Could not write to file %1").arg(outFilePath));
        }
        return;
    }

    QTextStream out(&outFile);

    out << "#trajectory is given in ECEF, pose represent platform2ecef transform, rotation is given as rotation axis in radian" << "\n";
    out << "time,x,y,z,rx,ry,rz" << "\n";

    Trajectory::TimeTrajectorySequence& trajSeq = exportTraj.value();

    for (int i = 0; i < trajSeq.nPoints(); i++) {
        double& time = trajSeq[i].time;
        StereoVision::Geometry::RigidBodyTransform<double> platform2ecef =
        exportOptimized ? StereoVision::Geometry::RigidBodyTransform<double>(local2ecef*trajSeq[i].val.toAffineTransform()) :
                          StereoVision::Geometry::RigidBodyTransform<double>(trajSeq[i].val.toAffineTransform());


        out << QString("%1").arg(time,0, 'f', 6) << ',';
        out << QString("%1").arg(platform2ecef.t.x(),0, 'f', 3) << ',';
        out << QString("%1").arg(platform2ecef.t.y(),0, 'f', 3) << ',';
        out << QString("%1").arg(platform2ecef.t.z(),0, 'f', 3) << ',';
        out << QString("%1").arg(platform2ecef.r.x(),0, 'f', 6) << ',';
        out << QString("%1").arg(platform2ecef.r.y(),0, 'f', 6) << ',';
        out << QString("%1").arg(platform2ecef.r.z(),0, 'f', 6);
        out << "\n";
    }

    out.flush();

}


void exportTrajectoryGeographic(Trajectory* traj, QString filePath, bool exportOptimized) {

    if (traj == nullptr) {
        return;
    }

    Project* currentProject = traj->getProject();

    MainWindow* mw = MainWindow::getActiveMainWindow();

    QString outFilePath = filePath;

    if (outFilePath.isEmpty()) {

        if (mw == nullptr) {
            return; //need main windows to display a save file dialog
        }

        outFilePath = QFileDialog::getSaveFileName(mw, QObject::tr("Save trajectory to"));

        if (outFilePath.isEmpty()) {
            return;
        }
    }

    constexpr bool subsample = true;

    StatusOptionalReturn<Trajectory::TimeTrajectorySequence> exportTraj =
        exportOptimized ? traj->optimizedTrajectory(subsample) : traj->loadTrajectorySequence();

    StereoVision::Geometry::AffineTransform<double> local2ecef(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

    if (exportOptimized) {
        if (currentProject != nullptr) {
            StereoVision::Geometry::AffineTransform<double> ecef2local = currentProject->ecef2local().cast<double>();
            local2ecef.R = ecef2local.R.transpose();
            local2ecef.t = -ecef2local.R.transpose()*ecef2local.t;
        }
    }


    if (!exportTraj.isValid()) {
        if (mw != nullptr) {
            QMessageBox::warning(mw, QObject::tr("Error when exporting trajectory"), exportTraj.errorMessage());
        }
        return;
    }

    const char* wgs84_latlonheight = "EPSG:4979";
    const char* wgs84_ecef = "EPSG:4978";

    Trajectory::TimeTrajectorySequence& trajSeq = exportTraj.value();

    PJ_CONTEXT* ctx = proj_context_create();

    if (ctx == nullptr) {
        if (mw != nullptr) {
            QMessageBox::warning(mw, QObject::tr("Proj error"), QObject::tr("Error when creating proj context to convert trajectory"));
        }
        return;
    }

    PJ* ecef2latlon = proj_create_crs_to_crs(ctx, wgs84_ecef, wgs84_latlonheight, nullptr);

    if (ecef2latlon == nullptr) {
        proj_context_destroy(ctx);
        if (mw != nullptr) {
            QMessageBox::warning(mw, QObject::tr("Proj error"), QObject::tr("Error when creating proj transform to convert trajectory"));
        }
        return;
    }

    Geo::TopocentricConvention topConv = Geo::TopocentricConvention::NED;

    Eigen::Array<double,3,Eigen::Dynamic> positions;
    positions.resize(3, trajSeq.nPoints());

    for (int i = 0; i < trajSeq.nPoints(); i++) {
        double& time = trajSeq[i].time;
        StereoVision::Geometry::RigidBodyTransform<double> platform2ecef =
        exportOptimized ? StereoVision::Geometry::RigidBodyTransform<double>(local2ecef*trajSeq[i].val.toAffineTransform()) :
                          StereoVision::Geometry::RigidBodyTransform<double>(trajSeq[i].val.toAffineTransform());

        PJ_COORD coordEcef;
        coordEcef.xyz.x = platform2ecef.t[0];
        coordEcef.xyz.y = platform2ecef.t[1];
        coordEcef.xyz.z = platform2ecef.t[2];

        PJ_COORD coordGeo = proj_trans(ecef2latlon, PJ_FWD, coordEcef);

        positions(0,i) = coordGeo.xyz.x;
        positions(1,i) = coordGeo.xyz.y;
        positions(2,i) = coordGeo.xyz.z;

    }

    proj_destroy(ecef2latlon);
    proj_context_destroy(ctx);

    auto ltpc2ecefopt = getLTPC2ECEF(positions,wgs84_latlonheight,topConv);

    if (!ltpc2ecefopt.has_value()) {
        if (mw != nullptr) {
            QMessageBox::warning(mw, QObject::tr("Proj error"), QObject::tr("Could not get transformations between ecef and local coordinate systems"));
        }
        return;
    }

    std::vector<StereoVision::Geometry::AffineTransform<double>>&
            local2ecefTransforms = ltpc2ecefopt.value();

    QFile outFile(outFilePath);

    bool ok = outFile.open(QFile::WriteOnly);

    if (!ok) {
        if (mw != nullptr) {
            QMessageBox::warning(mw,
                                 QObject::tr("Error when exporting trajectory"),
                                 QObject::tr("Could not write to file %1").arg(outFilePath));
        }
        return;
    }

    QTextStream out(&outFile);

    out << "#trajectory is given in WGG84 geographic coordinates (EPSG:4979), rotation is given as XYZ euler angles (degree) in local NED frame" << "\n";
    out << "time,lat,lon,height,roll,pitch,heading" << "\n";

    for (int i = 0; i < trajSeq.nPoints(); i++) {
        double& time = trajSeq[i].time;
        StereoVision::Geometry::RigidBodyTransform<double> platform2ecef =
        exportOptimized ? StereoVision::Geometry::RigidBodyTransform<double>(local2ecef*trajSeq[i].val.toAffineTransform()) :
                          StereoVision::Geometry::RigidBodyTransform<double>(trajSeq[i].val.toAffineTransform());

        StereoVision::Geometry::RigidBodyTransform<double> local2ecef =
                local2ecefTransforms[i];

        StereoVision::Geometry::AffineTransform<double> platform2local =
                local2ecef.inverse().toAffineTransform()*platform2ecef.toAffineTransform();

        Eigen::Matrix3d& rMat = platform2local.R;

        Eigen::Vector3d angles
                = StereoVision::Geometry::rMat2eulerRadzyx(rMat)/M_PI * 180;

        out << QString("%1").arg(time,0, 'f', 6) << ',';
        out << QString("%1").arg(positions(0,i),0, 'f', 14) << ',';
        out << QString("%1").arg(positions(1,i),0, 'f', 14) << ',';
        out << QString("%1").arg(positions(2,i),0, 'f', 6) << ',';
        out << QString("%1").arg(angles.x(),0, 'f', 6) << ',';
        out << QString("%1").arg(angles.y(),0, 'f', 6) << ',';
        out << QString("%1").arg(angles.z(),0, 'f', 6);
        out << "\n";
    }

    out.flush();



}

} // namespace StereoVisionApp
