#ifndef STEREOVISIONAPP_TRAJECTORYOPTANALYSISEDITOR_H
#define STEREOVISIONAPP_TRAJECTORYOPTANALYSISEDITOR_H

#include <QWidget>
#include <QString>
#include <QMap>

#include "./editor.h"

class QCustomPlot;

namespace StereoVisionApp {

class Trajectory;

class TrajectoryOptAnalysisEditor: public Editor
{
    Q_OBJECT
public:
    TrajectoryOptAnalysisEditor(QWidget *parent = nullptr);

    void setTrajectory(Trajectory* trj);

protected:

    void reconfigurePlots();

    Trajectory* _trajectory;

    QCustomPlot* _absolutePositionPlot;
    QCustomPlot* _positionDeltasPlot;

    QCustomPlot* _absoluteOrientationPlot;
    QCustomPlot* _orientationDeltasPlot;

};


class TrajectoryOptAnalysisEditorFactory : public EditorFactory
{
    Q_OBJECT
public:
    explicit TrajectoryOptAnalysisEditorFactory(QObject *parent = nullptr);


    virtual QString TypeDescrName() const;
    virtual QString itemClassName() const;
    virtual Editor* factorizeEditor(QWidget* parent) const;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYOPTANALYSISEDITOR_H
