#include "trajectoryalignementanalysiseditor.h"

#define QCUSTOMPLOT_USE_LIBRARY
#include "qcustomplot/qcustomplot.h"

#include "datablocks/trajectory.h"

#include "utils/statusoptionalreturn.h"

#include "vision/trajectoryImuPreIntegration.h"

#include <QVBoxLayout>

#include <QLabel>

namespace StereoVisionApp {

TrajectoryAlignementAnalysisEditor::TrajectoryAlignementAnalysisEditor(QWidget *parent)
    : Editor{parent},
      _trajectory(nullptr)
{

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setMargin(5);
    layout->setSpacing(0);

    QLabel* speedDeltasLabel = new QLabel(tr("Speed deltas:"), this);
    _speedDeltasPlot = new QCustomPlot(this);

    _speedDeltasPlot->legend->setVisible(true);
    _speedDeltasPlot->legend->setFont(QFont("Sans",9));

    _speedDeltasPlot->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    _speedDeltasPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    _speedDeltasPlot->addGraph(); // x imu
    _speedDeltasPlot->addGraph(); // y imu
    _speedDeltasPlot->addGraph(); // z imu
    _speedDeltasPlot->addGraph(); // x traj
    _speedDeltasPlot->addGraph(); // y traj
    _speedDeltasPlot->addGraph(); // z traj
    _speedDeltasPlot->xAxis->setLabel("time");
    _speedDeltasPlot->yAxis->setLabel("Speed delta");

    _speedDeltasPlot->graph(0)->setPen(QPen(QColor(255,0,0)));
    _speedDeltasPlot->graph(1)->setPen(QPen(QColor(0,255,0)));
    _speedDeltasPlot->graph(2)->setPen(QPen(QColor(0,0,255)));

    _speedDeltasPlot->graph(3)->setPen(QPen(QColor(255,0,255)));
    _speedDeltasPlot->graph(4)->setPen(QPen(QColor(200,255,0)));
    _speedDeltasPlot->graph(5)->setPen(QPen(QColor(0,255,255)));

    _speedDeltasPlot->graph(0)->setName("x acc");
    _speedDeltasPlot->graph(1)->setName("y acc");
    _speedDeltasPlot->graph(2)->setName("z acc");

    _speedDeltasPlot->graph(3)->setName("x traj");
    _speedDeltasPlot->graph(4)->setName("y traj");
    _speedDeltasPlot->graph(5)->setName("z traj");

    layout->addWidget(speedDeltasLabel);
    layout->addWidget(_speedDeltasPlot);

    layout->addSpacing(5);

    QLabel* orientationDeltasLabel = new QLabel(tr("Orientation deltas:"), this);
    _orientationDeltasPlot = new QCustomPlot(this);

    _orientationDeltasPlot->legend->setVisible(true);
    _orientationDeltasPlot->legend->setFont(QFont("Sans",9));

    _orientationDeltasPlot->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    _orientationDeltasPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    _orientationDeltasPlot->addGraph(); // x imu
    _orientationDeltasPlot->addGraph(); // y imu
    _orientationDeltasPlot->addGraph(); // z imu
    _orientationDeltasPlot->addGraph(); // x traj
    _orientationDeltasPlot->addGraph(); // y traj
    _orientationDeltasPlot->addGraph(); // z traj
    _orientationDeltasPlot->xAxis->setLabel("time");
    _orientationDeltasPlot->yAxis->setLabel("Orientation delta");

    _orientationDeltasPlot->graph(0)->setPen(QPen(QColor(255,0,0)));
    _orientationDeltasPlot->graph(1)->setPen(QPen(QColor(0,255,0)));
    _orientationDeltasPlot->graph(2)->setPen(QPen(QColor(0,0,255)));

    _orientationDeltasPlot->graph(3)->setPen(QPen(QColor(255,0,255)));
    _orientationDeltasPlot->graph(4)->setPen(QPen(QColor(200,255,0)));
    _orientationDeltasPlot->graph(5)->setPen(QPen(QColor(0,255,255)));

    _orientationDeltasPlot->graph(0)->setName("x gyro");
    _orientationDeltasPlot->graph(1)->setName("y gyro");
    _orientationDeltasPlot->graph(2)->setName("z gyro");

    _orientationDeltasPlot->graph(3)->setName("x traj");
    _orientationDeltasPlot->graph(4)->setName("y traj");
    _orientationDeltasPlot->graph(5)->setName("z traj");


    layout->addWidget(orientationDeltasLabel);
    layout->addWidget(_orientationDeltasPlot);

    //synchronize the ranges
    connect(_speedDeltasPlot->xAxis, static_cast<void(QCPAxis::*)(const QCPRange &)>(&QCPAxis::rangeChanged),
            _orientationDeltasPlot->xAxis, static_cast<void(QCPAxis::*)(const QCPRange &)>(&QCPAxis::setRange));
    connect(_orientationDeltasPlot->xAxis, static_cast<void(QCPAxis::*)(const QCPRange &)>(&QCPAxis::rangeChanged),
            _speedDeltasPlot->xAxis, static_cast<void(QCPAxis::*)(const QCPRange &)>(&QCPAxis::setRange));

}

void TrajectoryAlignementAnalysisEditor::setTrajectory(Trajectory* trj) {

    if (trj != _trajectory) {

        if (_trajectory != nullptr) {
            disconnect(_trajectory, nullptr, this, nullptr); //disconnect all;
        }

        _trajectory = trj;
        reconfigurePlots();

        connect(_trajectory, &Trajectory::trajectoryDataChanged,
                this, &TrajectoryAlignementAnalysisEditor::reconfigurePlots);
        connect(_trajectory, &Trajectory::optimizedTrajectoryDataChanged,
                this, &TrajectoryAlignementAnalysisEditor::reconfigurePlots);
    }

}

void TrajectoryAlignementAnalysisEditor::reconfigurePlots() {

    StatusOptionalReturn<Trajectory::TimeTrajectorySequence> trajOptional = StatusOptionalReturn<Trajectory::TimeTrajectorySequence>::error("");

    StatusOptionalReturn<Trajectory::TimeCartesianSequence> angularSpeedOpt = StatusOptionalReturn<Trajectory::TimeCartesianSequence>::error("");

    StatusOptionalReturn<Trajectory::TimeCartesianSequence> accelerometerOpt = StatusOptionalReturn<Trajectory::TimeCartesianSequence>::error("");

    bool valid_trajectory = true;

    if (_trajectory == nullptr) {
        valid_trajectory = false;
    } else {

        trajOptional = _trajectory->loadTrajectoryProjectLocalFrameSequence();

        angularSpeedOpt = _trajectory->loadAngularSpeedSequence();

        accelerometerOpt = _trajectory->loadAccelerationSequence();

        if (!trajOptional.isValid()) {
            valid_trajectory = false;
        }

        if (!angularSpeedOpt.isValid()) {
            valid_trajectory = false;
        }

        if (!accelerometerOpt.isValid()) {
            valid_trajectory = false;
        }
    }

    if (!valid_trajectory) {

        for (int i = 0; i < 6; i++) {
            _speedDeltasPlot->graph(i)->setData(QVector<double>(), QVector<double>());
            _orientationDeltasPlot->graph(i)->setData(QVector<double>(), QVector<double>());
        }

        _speedDeltasPlot->replot();
        _orientationDeltasPlot->replot();

        return;
    }

    Trajectory::TimeTrajectorySequence& traj = trajOptional.value();
    Trajectory::TimeCartesianSequence& gyro = angularSpeedOpt.value();
    Trajectory::TimeCartesianSequence& accelerometer = accelerometerOpt.value();


    double minIntegrationTime = 2; //min integration time of half a second

    double startTime = traj.sequenceStartTime();
    double endTime = traj.sequenceEndTime();

    int nExpectedErrors = std::ceil(1.1*(endTime - startTime)/minIntegrationTime);

    constexpr int maxSamples = 10000;

    int nExpectedSamples = std::min(nExpectedErrors, maxSamples);

    QVector<double> timesAccelerometer;
    QVector<double> timesGyro;

    QVector<double> speedXdeltaImu;
    QVector<double> speedYdeltaImu;
    QVector<double> speedZdeltaImu;

    QVector<double> speedXdeltaTraj;
    QVector<double> speedYdeltaTraj;
    QVector<double> speedZdeltaTraj;

    QVector<double> rotXdeltaGyro;
    QVector<double> rotYdeltaGyro;
    QVector<double> rotZdeltaGyro;

    QVector<double> rotXdeltaTraj;
    QVector<double> rotYdeltaTraj;
    QVector<double> rotZdeltaTraj;

    timesAccelerometer.reserve(nExpectedSamples);
    timesGyro.reserve(nExpectedSamples);

    speedXdeltaImu.reserve(nExpectedSamples);
    speedYdeltaImu.reserve(nExpectedSamples);
    speedZdeltaImu.reserve(nExpectedSamples);

    speedXdeltaTraj.reserve(nExpectedSamples);
    speedYdeltaTraj.reserve(nExpectedSamples);
    speedZdeltaTraj.reserve(nExpectedSamples);

    rotXdeltaGyro.reserve(nExpectedSamples);
    rotYdeltaGyro.reserve(nExpectedSamples);
    rotZdeltaGyro.reserve(nExpectedSamples);

    rotXdeltaTraj.reserve(nExpectedSamples);
    rotYdeltaTraj.reserve(nExpectedSamples);
    rotZdeltaTraj.reserve(nExpectedSamples);

    int deltai = 1;
    int expectedDeltai = std::ceil(traj.nPoints()/double(nExpectedSamples));

    double minTime = traj[0].time;
    double maxTime = traj[traj.nPoints()-1].time;

    double maxAbsPosDelta = 0;
    double maxAbsRotDelta = 0;

    for (int i = 0; i < traj.nPoints(); i += deltai) {

        Trajectory::TimeTrajectorySequence::TimedElement body1_to_local = traj[i];

        deltai = 1;

        do {

            if (traj[i+deltai].time - body1_to_local.time >= minIntegrationTime) {
                break;
            }
            deltai++;
        } while (i+deltai < traj.nPoints());

        if (i+deltai >= traj.nPoints()) {
            break;
        }

        Trajectory::TimeTrajectorySequence::TimedElement body2_to_local = traj[i+deltai];

        StereoVision::Geometry::RigidBodyTransform<double> body1_to_body2 = body2_to_local.val.inverse()*body1_to_local.val;
        StereoVision::Geometry::AffineTransform<double> body1_to_body2Aff = body1_to_body2.toAffineTransform();

        Eigen::Matrix3d GyroR2to1 = PreIntegrateGyro(gyro, body1_to_local.time, body2_to_local.time);

        Eigen::Vector3d imuAngularSpeed = StereoVision::Geometry::inverseRodriguezFormula(GyroR2to1);
        Eigen::Vector3d observedAngularSpeed = StereoVision::Geometry::inverseRodriguezFormula(body1_to_body2Aff.R); //we convert to matrix and back to normalize

        timesGyro.push_back(body1_to_local.time);

        rotXdeltaGyro.push_back(imuAngularSpeed.x());
        rotYdeltaGyro.push_back(imuAngularSpeed.y());
        rotZdeltaGyro.push_back(imuAngularSpeed.z());

        if (std::abs(rotXdeltaGyro.last()) > maxAbsRotDelta) {
            maxAbsRotDelta = std::abs(rotXdeltaGyro.last());
        }

        if (std::abs(rotYdeltaGyro.last()) > maxAbsRotDelta) {
            maxAbsRotDelta = std::abs(rotYdeltaGyro.last());
        }

        if (std::abs(rotZdeltaGyro.last()) > maxAbsRotDelta) {
            maxAbsRotDelta = std::abs(rotZdeltaGyro.last());
        }

        rotXdeltaTraj.push_back(observedAngularSpeed.x());
        rotYdeltaTraj.push_back(observedAngularSpeed.y());
        rotZdeltaTraj.push_back(observedAngularSpeed.z());

        if (std::abs(rotXdeltaTraj.last()) > maxAbsRotDelta) {
            maxAbsRotDelta = std::abs(rotXdeltaTraj.last());
        }

        if (std::abs(rotYdeltaTraj.last()) > maxAbsRotDelta) {
            maxAbsRotDelta = std::abs(rotYdeltaTraj.last());
        }

        if (std::abs(rotZdeltaTraj.last()) > maxAbsRotDelta) {
            maxAbsRotDelta = std::abs(rotZdeltaTraj.last());
        }

        if (i+2*deltai < traj.nPoints()) {
            Trajectory::TimeTrajectorySequence::TimedElement body3_to_local = traj[i+2*deltai];

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

            Eigen::Vector3d speedDeltaObs = PreIntegrateAccelerometer(accelerometer, gyro, timespeed1, timespeed2);

            speedDeltaObs /= (timespeed2 - timespeed1);
            speedDeltaTrj /= (timespeed2 - timespeed1);

            timesAccelerometer.push_back(timespeed1);

            speedXdeltaImu.push_back(speedDeltaObs.x());
            speedYdeltaImu.push_back(speedDeltaObs.y());
            speedZdeltaImu.push_back(speedDeltaObs.z());

            if (std::abs(speedXdeltaImu.last()) > maxAbsPosDelta) {
                maxAbsPosDelta = std::abs(speedXdeltaImu.last());
            }

            if (std::abs(speedYdeltaImu.last()) > maxAbsPosDelta) {
                maxAbsPosDelta = std::abs(speedYdeltaImu.last());
            }

            if (std::abs(speedZdeltaImu.last()) > maxAbsPosDelta) {
                maxAbsPosDelta = std::abs(speedZdeltaImu.last());
            }

            speedXdeltaTraj.push_back(speedDeltaTrj.x());
            speedYdeltaTraj.push_back(speedDeltaTrj.y());
            speedZdeltaTraj.push_back(speedDeltaTrj.z());

            if (std::abs(speedXdeltaTraj.last()) > maxAbsPosDelta) {
                maxAbsPosDelta = std::abs(speedXdeltaTraj.last());
            }

            if (std::abs(speedYdeltaTraj.last()) > maxAbsPosDelta) {
                maxAbsPosDelta = std::abs(speedYdeltaTraj.last());
            }

            if (std::abs(speedZdeltaTraj.last()) > maxAbsPosDelta) {
                maxAbsPosDelta = std::abs(speedZdeltaTraj.last());
            }
        }

        deltai = std::max(deltai, expectedDeltai);

    }

    _speedDeltasPlot->graph(0)->setData(timesAccelerometer, speedXdeltaImu);
    _speedDeltasPlot->graph(1)->setData(timesAccelerometer, speedYdeltaImu);
    _speedDeltasPlot->graph(2)->setData(timesAccelerometer, speedZdeltaImu);


    _speedDeltasPlot->graph(3)->setData(timesAccelerometer, speedXdeltaTraj);
    _speedDeltasPlot->graph(4)->setData(timesAccelerometer, speedYdeltaTraj);
    _speedDeltasPlot->graph(5)->setData(timesAccelerometer, speedZdeltaTraj);

    _speedDeltasPlot->xAxis->setRange(minTime, maxTime);
    _speedDeltasPlot->yAxis->setRange(-maxAbsPosDelta, maxAbsPosDelta);

    _speedDeltasPlot->replot();

    _orientationDeltasPlot->graph(0)->setData(timesGyro, rotXdeltaGyro);
    _orientationDeltasPlot->graph(1)->setData(timesGyro, rotYdeltaGyro);
    _orientationDeltasPlot->graph(2)->setData(timesGyro, rotZdeltaGyro);

    _orientationDeltasPlot->graph(3)->setData(timesGyro, rotXdeltaTraj);
    _orientationDeltasPlot->graph(4)->setData(timesGyro, rotYdeltaTraj);
    _orientationDeltasPlot->graph(5)->setData(timesGyro, rotZdeltaTraj);

    _orientationDeltasPlot->yAxis->setRange(-maxAbsRotDelta, maxAbsRotDelta);

    _orientationDeltasPlot->replot();



}

TrajectoryAlignementAnalysisEditorFactory::TrajectoryAlignementAnalysisEditorFactory(QObject *parent) :
    EditorFactory(parent)
{

}

QString TrajectoryAlignementAnalysisEditorFactory::TypeDescrName() const {
    return tr("Trajectory Alignement Analysis");
}
QString TrajectoryAlignementAnalysisEditorFactory::itemClassName() const {
    return TrajectoryAlignementAnalysisEditor::staticMetaObject.className();
}
Editor* TrajectoryAlignementAnalysisEditorFactory::factorizeEditor(QWidget* parent) const {
    return new TrajectoryAlignementAnalysisEditor(parent);
}

} // namespace StereoVisionApp
