#include "trajectorysequencevieweditor.h"

#define QCUSTOMPLOT_USE_LIBRARY
#include "qcustomplot/qcustomplot.h"

#include "datablocks/trajectory.h"

#include <QVBoxLayout>
#include <QLabel>

namespace StereoVisionApp {

TrajectorySequenceViewEditor::TrajectorySequenceViewEditor(QWidget *parent)
    : Editor{parent},
      _trajectory(nullptr)
{

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setMargin(5);
    layout->setSpacing(0);

    QLabel* positionLabel = new QLabel(tr("Position:"), this);
    _positionPlot = new QCustomPlot(this);

    _positionPlot->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    _positionPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    _positionPlot->addGraph(); // x
    _positionPlot->addGraph(); // y
    _positionPlot->addGraph(); // z
    _positionPlot->xAxis->setLabel("time");
    _positionPlot->yAxis->setLabel("position");

    _positionPlot->graph(0)->setPen(QPen(Qt::red));
    _positionPlot->graph(1)->setPen(QPen(Qt::green));
    _positionPlot->graph(2)->setPen(QPen(Qt::blue));

    layout->addWidget(positionLabel);
    layout->addWidget(_positionPlot);

    layout->addSpacing(5);

    QLabel* orientationLabel = new QLabel(tr("Orientation:"), this);
    _orientationPlot = new QCustomPlot(this);

    _orientationPlot->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    _orientationPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    _orientationPlot->addGraph(); // x
    _orientationPlot->addGraph(); // y
    _orientationPlot->addGraph(); // z
    _orientationPlot->xAxis->setLabel("time");
    _orientationPlot->yAxis->setLabel("orientation");

    _orientationPlot->graph(0)->setPen(QPen(Qt::red));
    _orientationPlot->graph(1)->setPen(QPen(Qt::green));
    _orientationPlot->graph(2)->setPen(QPen(Qt::blue));


    layout->addWidget(orientationLabel);
    layout->addWidget(_orientationPlot);

    //synchronize the ranges
    connect(_positionPlot->xAxis, static_cast<void(QCPAxis::*)(const QCPRange &)>(&QCPAxis::rangeChanged),
            _orientationPlot->xAxis, static_cast<void(QCPAxis::*)(const QCPRange &)>(&QCPAxis::setRange));
    connect(_positionPlot->xAxis, static_cast<void(QCPAxis::*)(const QCPRange &)>(&QCPAxis::rangeChanged),
            _orientationPlot->xAxis, static_cast<void(QCPAxis::*)(const QCPRange &)>(&QCPAxis::setRange));

}

void TrajectorySequenceViewEditor::setTrajectory(Trajectory* trj) {

    if (trj != _trajectory) {

        if (_trajectory != nullptr) {
            disconnect(_trajectory, nullptr, this, nullptr); //disconnect all;
        }

        _trajectory = trj;
        reconfigurePlots();

        connect(_trajectory, &Trajectory::trajectoryDataChanged,
                this, &TrajectorySequenceViewEditor::reconfigurePlots);
        connect(_trajectory, &Trajectory::optimizedTrajectoryDataChanged,
                this, &TrajectorySequenceViewEditor::reconfigurePlots);
    }

}

void TrajectorySequenceViewEditor::reconfigurePlots() {

    QTextStream out(stdout);

    StatusOptionalReturn<Trajectory::TimeTrajectorySequence> trajOptional = StatusOptionalReturn<Trajectory::TimeTrajectorySequence>::error("");

    bool valid_trajectory = true;

    if (_trajectory == nullptr) {
        QMessageBox::warning(this, tr("Could not load data"), tr("Null trajectory provided"));
        valid_trajectory = false;
    } else {

        trajOptional = _trajectory->loadTrajectoryProjectLocalFrameSequence();

        if (!trajOptional.isValid()) {
            QMessageBox::warning(this, tr("Invalid trajectory data"), trajOptional.errorMessage());
            valid_trajectory = false;
        }
    }

    if (!valid_trajectory) {

        for (int i = 0; i < 3; i++) {
            _positionPlot->graph(i)->setData(QVector<double>(), QVector<double>());
            _orientationPlot->graph(i)->setData(QVector<double>(), QVector<double>());
        }

        _positionPlot->replot();
        _orientationPlot->replot();

        return;
    }

    Trajectory::TimeTrajectorySequence& traj = trajOptional.value();

    if (traj.nPoints() == 0) {
        QMessageBox::warning(this, tr("Empty trajectory data"), tr("Check the trajectory configuration"));
    }

    out << tr("Loaded and display trajectory with %1 nodes").arg(traj.nPoints()) << Qt::endl;

    constexpr int maxSamples = 50000;

    int nExpectedSamples = std::min(traj.nPoints(), maxSamples);

    int deltai = traj.nPoints()/nExpectedSamples;

    out << "nExpectedSamples = " << nExpectedSamples << " deltai = " << deltai << Qt::endl;

    int maxNSamples = traj.nPoints()/deltai + 1;

    QVector<double> times;

    QVector<double> posX;
    QVector<double> posY;
    QVector<double> posZ;

    QVector<double> rotX;
    QVector<double> rotY;
    QVector<double> rotZ;

    times.reserve(maxNSamples);

    posX.reserve(maxNSamples);
    posY.reserve(maxNSamples);
    posZ.reserve(maxNSamples);

    rotX.reserve(maxNSamples);
    rotY.reserve(maxNSamples);
    rotZ.reserve(maxNSamples);

    double maxPos = -std::numeric_limits<double>::infinity();
    double minPos = std::numeric_limits<double>::infinity();

    double maxRot = -std::numeric_limits<double>::infinity();
    double minRot = std::numeric_limits<double>::infinity();

    for (int i = 0; i < traj.nPoints(); i += deltai) {

        Trajectory::TimeTrajectorySequence::TimedElement pose = traj[i];

        deltai = 1;//we convert to matrix and back to normalize

        times.push_back(pose.time);

        /*if (!pose.val.t.allFinite() or !pose.val.r.allFinite()) {
            out << "Non finite pose at time = << " << pose.time << " index: " << i
                << " t = [" << pose.val.t.x() << " " << pose.val.t.y() << " " << pose.val.t.z()
                << "] r = [" << pose.val.r.x() << " " << pose.val.r.y() << " " << pose.val.r.z()
                << "]\n";
        }*/

        posX.push_back(pose.val.t.x());
        posY.push_back(pose.val.t.y());
        posZ.push_back(pose.val.t.z());

        for (int a = 0; a < 3; a++) {

            if (pose.val.t[a] < minPos) {
                minPos = pose.val.t[a];
            }

            if (pose.val.t[a] > maxPos) {
                maxPos = pose.val.t[a];
            }
        }

        rotX.push_back(pose.val.r.x());
        rotY.push_back(pose.val.r.y());
        rotZ.push_back(pose.val.r.z());

        for (int a = 0; a < 3; a++) {

            if (pose.val.r[a] < minRot) {
                minRot = pose.val.r[a];
            }

            if (pose.val.r[a] > maxRot) {
                maxRot = pose.val.r[a];
            }
        }

    }

    double minTime = times.first();
    double maxTime = times.last();

    out << "minTime = " << minTime << " maxTime = " << maxTime
        << " minPos = " << minPos << " maxPos = " << maxPos
        << " minRot = " << minRot << " maxRot = " << maxRot << Qt::endl;

    _positionPlot->graph(0)->setData(times, posX);
    _positionPlot->graph(1)->setData(times, posY);
    _positionPlot->graph(2)->setData(times, posZ);

    _positionPlot->xAxis->setRange(minTime, maxTime);
    _positionPlot->yAxis->setRange(minPos*1.05, maxPos*1.05);

    _positionPlot->replot();

    _orientationPlot->graph(0)->setData(times, rotX);
    _orientationPlot->graph(1)->setData(times, rotY);
    _orientationPlot->graph(2)->setData(times, rotZ);

    _orientationPlot->xAxis->setRange(minTime, maxTime);
    _orientationPlot->yAxis->setRange(minRot*1.05, maxRot*1.05);

    _orientationPlot->replot();

}

TrajectorySequenceViewEditorFactory::TrajectorySequenceViewEditorFactory(QObject *parent) :
    EditorFactory(parent)
{

}


QString TrajectorySequenceViewEditorFactory::TypeDescrName() const {
    return tr("Trajectory Sequence View");
}
QString TrajectorySequenceViewEditorFactory::itemClassName() const {
    return TrajectorySequenceViewEditor::staticMetaObject.className();
}
Editor* TrajectorySequenceViewEditorFactory::factorizeEditor(QWidget* parent) const {
    return new TrajectorySequenceViewEditor(parent);
}

} // namespace StereoVisionApp
