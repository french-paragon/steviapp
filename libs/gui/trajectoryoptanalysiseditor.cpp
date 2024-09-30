#include "trajectoryoptanalysiseditor.h"

#define QCUSTOMPLOT_USE_LIBRARY
#include "qcustomplot/qcustomplot.h"

#include "datablocks/trajectory.h"

#include <QVBoxLayout>
#include <QLabel>

namespace StereoVisionApp {

TrajectoryOptAnalysisEditor::TrajectoryOptAnalysisEditor(QWidget *parent) :
    Editor(parent),
    _trajectory(nullptr)
{

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setMargin(5);
    layout->setSpacing(0);

    QLabel* positionDeltasLabel = new QLabel(tr("Position residuals:"), this);
    _positionDeltasPlot = new QCustomPlot(this);

    _positionDeltasPlot->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    _positionDeltasPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    _positionDeltasPlot->addGraph(); // x
    _positionDeltasPlot->addGraph(); // y
    _positionDeltasPlot->addGraph(); // z
    _positionDeltasPlot->xAxis->setLabel("time");
    _positionDeltasPlot->yAxis->setLabel("position error");

    _positionDeltasPlot->graph(0)->setPen(QPen(Qt::red));
    _positionDeltasPlot->graph(1)->setPen(QPen(Qt::green));
    _positionDeltasPlot->graph(2)->setPen(QPen(Qt::blue));

    layout->addWidget(positionDeltasLabel);
    layout->addWidget(_positionDeltasPlot);

    layout->addSpacing(5);

    QLabel* orientationDeltasLabel = new QLabel(tr("Orientation residuals:"), this);
    _orientationDeltasPlot = new QCustomPlot(this);

    _orientationDeltasPlot->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    _orientationDeltasPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    _orientationDeltasPlot->addGraph(); // x
    _orientationDeltasPlot->addGraph(); // y
    _orientationDeltasPlot->addGraph(); // z
    _orientationDeltasPlot->xAxis->setLabel("time");
    _orientationDeltasPlot->yAxis->setLabel("orientation error");

    _orientationDeltasPlot->graph(0)->setPen(QPen(Qt::red));
    _orientationDeltasPlot->graph(1)->setPen(QPen(Qt::green));
    _orientationDeltasPlot->graph(2)->setPen(QPen(Qt::blue));


    layout->addWidget(orientationDeltasLabel);
    layout->addWidget(_orientationDeltasPlot);

    //synchronize the ranges
    connect(_positionDeltasPlot->xAxis, static_cast<void(QCPAxis::*)(const QCPRange &)>(&QCPAxis::rangeChanged),
            _orientationDeltasPlot->xAxis, static_cast<void(QCPAxis::*)(const QCPRange &)>(&QCPAxis::setRange));
    connect(_orientationDeltasPlot->xAxis, static_cast<void(QCPAxis::*)(const QCPRange &)>(&QCPAxis::rangeChanged),
            _positionDeltasPlot->xAxis, static_cast<void(QCPAxis::*)(const QCPRange &)>(&QCPAxis::setRange));

}

void TrajectoryOptAnalysisEditor::setTrajectory(Trajectory* trj) {
    if (trj != _trajectory) {

        if (_trajectory != nullptr) {
            disconnect(_trajectory, nullptr, this, nullptr); //disconnect all;
        }

        _trajectory = trj;
        reconfigurePlots();

        connect(_trajectory, &Trajectory::trajectoryDataChanged,
                this, &TrajectoryOptAnalysisEditor::reconfigurePlots);
        connect(_trajectory, &Trajectory::optimizedTrajectoryDataChanged,
                this, &TrajectoryOptAnalysisEditor::reconfigurePlots);
    }
}

void TrajectoryOptAnalysisEditor::reconfigurePlots() {

    Trajectory::TimeTrajectorySequence traj;
    Trajectory::TimeTrajectorySequence optTraj;

    bool valid_trajectory = true;
    if (_trajectory == nullptr) {
        valid_trajectory = false;
    } else {

        std::optional<Trajectory::TimeTrajectorySequence> trajOptional = _trajectory->loadTrajectoryProjectLocalFrameSequence();

        if (!trajOptional.has_value()) {
            valid_trajectory = false;
        } else {
            traj = trajOptional.value();
        }

        std::optional<Trajectory::TimeTrajectorySequence> optTrajOptional = _trajectory->optimizedTrajectory();

       if (!optTrajOptional.has_value()) {
           valid_trajectory = false;
       } else {
           optTraj = optTrajOptional.value();
       }
    }

    if (!valid_trajectory) {

        _positionDeltasPlot->graph(0)->setData(QVector<double>(), QVector<double>());
        _positionDeltasPlot->graph(1)->setData(QVector<double>(), QVector<double>());
        _positionDeltasPlot->graph(2)->setData(QVector<double>(), QVector<double>());

        _positionDeltasPlot->replot();

        _orientationDeltasPlot->graph(0)->setData(QVector<double>(), QVector<double>());
        _orientationDeltasPlot->graph(1)->setData(QVector<double>(), QVector<double>());
        _orientationDeltasPlot->graph(2)->setData(QVector<double>(), QVector<double>());

        _orientationDeltasPlot->replot();

        return;
    }

    constexpr int maxSamples = 10000;

    int nSamples = std::min(optTraj.nPoints(), maxSamples);

    QVector<double> times(nSamples);

    QVector<double> posXerrors(nSamples);
    QVector<double> posYerrors(nSamples);
    QVector<double> posZerrors(nSamples);

    QVector<double> rotXerrors(nSamples);
    QVector<double> rotYerrors(nSamples);
    QVector<double> rotZerrors(nSamples);

    double minTime = optTraj[0].time;
    double maxTime = optTraj[optTraj.nPoints()-1].time;

    double maxAbsPosError = 0;
    double maxAbsRotError = 0;

    for (int i = 0; i < nSamples; i++) {

        double time = minTime + i*((maxTime - minTime)/(nSamples-1));

        times[i] = time;

        auto interpTraj = traj.getValueAtTime(time);

        StereoVision::Geometry::RigidBodyTransform<double> initial =
                interpTraj.weigthLower*interpTraj.valLower + interpTraj.weigthUpper*interpTraj.valUpper;

        auto interpOptTraj = optTraj.getValueAtTime(time);

        StereoVision::Geometry::RigidBodyTransform<double> opt =
                interpOptTraj.weigthLower*interpOptTraj.valLower + interpOptTraj.weigthUpper*interpOptTraj.valUpper;

        posXerrors[i] = initial.t.x() - opt.t.x();
        posYerrors[i] = initial.t.y() - opt.t.y();
        posZerrors[i] = initial.t.z() - opt.t.z();

        if (std::abs(posXerrors[i]) > maxAbsPosError) {
            maxAbsPosError = std::abs(posXerrors[i]);
        }

        if (std::abs(posYerrors[i]) > maxAbsPosError) {
            maxAbsPosError = std::abs(posYerrors[i]);
        }

        if (std::abs(posZerrors[i]) > maxAbsPosError) {
            maxAbsPosError = std::abs(posZerrors[i]);
        }

        rotXerrors[i] = initial.r.x() - opt.r.x();
        rotYerrors[i] = initial.r.y() - opt.r.y();
        rotZerrors[i] = initial.r.z() - opt.r.z();

        if (std::abs(rotXerrors[i]) > maxAbsRotError) {
            maxAbsRotError = std::abs(rotXerrors[i]);
        }

        if (std::abs(rotYerrors[i]) > maxAbsRotError) {
            maxAbsRotError = std::abs(rotYerrors[i]);
        }

        if (std::abs(rotZerrors[i]) > maxAbsRotError) {
            maxAbsRotError = std::abs(rotZerrors[i]);
        }
    }

    _positionDeltasPlot->graph(0)->setData(times, posXerrors);
    _positionDeltasPlot->graph(1)->setData(times, posYerrors);
    _positionDeltasPlot->graph(2)->setData(times, posZerrors);

    _positionDeltasPlot->xAxis->setRange(minTime, maxTime);
    _positionDeltasPlot->yAxis->setRange(-maxAbsPosError, maxAbsPosError);

    _positionDeltasPlot->replot();

    _orientationDeltasPlot->graph(0)->setData(times, rotXerrors);
    _orientationDeltasPlot->graph(1)->setData(times, rotYerrors);
    _orientationDeltasPlot->graph(2)->setData(times, rotZerrors);

    _orientationDeltasPlot->yAxis->setRange(-maxAbsRotError, maxAbsRotError);

    _orientationDeltasPlot->replot();

}

TrajectoryOptAnalysisEditorFactory::TrajectoryOptAnalysisEditorFactory(QObject *parent) :
    EditorFactory(parent)
{

}


QString TrajectoryOptAnalysisEditorFactory::TypeDescrName() const {
    return tr("Optimized trajectory analyzer");
}

QString TrajectoryOptAnalysisEditorFactory::itemClassName() const {
    return TrajectoryOptAnalysisEditor::staticMetaObject.className();
}

Editor* TrajectoryOptAnalysisEditorFactory::factorizeEditor(QWidget* parent) const {
    return new TrajectoryOptAnalysisEditor(parent);
}

} // namespace StereoVisionApp
