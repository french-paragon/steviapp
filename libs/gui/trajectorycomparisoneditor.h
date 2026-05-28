#ifndef STEREOVISIONAPP_TRAJECTORYOPTANALYSISEDITOR_H
#define STEREOVISIONAPP_TRAJECTORYOPTANALYSISEDITOR_H

#include <QWidget>
#include <QString>
#include <QMap>

#include "./editor.h"

class QCustomPlot;

namespace StereoVisionApp {

class Trajectory;

class TrajectoryComparisonEditor: public Editor
{
    Q_OBJECT
public:
    TrajectoryComparisonEditor(QWidget *parent = nullptr);

    void setTrajectory(Trajectory* trj);

    inline void setTrajectories(Trajectory* trj1, Trajectory* trj2, bool compareOptimized = false) {
        setTrajectories(trj1, trj2, compareOptimized, compareOptimized);
    }
    void setTrajectories(Trajectory* trj1, Trajectory* trj2, bool compareOptimized1, bool compareOptimized2);

protected:

    void reconfigurePlots();

    Trajectory* _trajectory;
    Trajectory* _trajectory2;
    bool _useOptimizedInMulti1;
    bool _useOptimizedInMulti2;
    bool _forwardComparison;

    QCustomPlot* _absolutePositionPlot;
    QCustomPlot* _positionDeltasPlot;

    QCustomPlot* _absoluteOrientationPlot;
    QCustomPlot* _orientationDeltasPlot;

};


class TrajectoryComparisonEditorFactory : public EditorFactory
{
    Q_OBJECT
public:
    explicit TrajectoryComparisonEditorFactory(QObject *parent = nullptr);


    virtual QString TypeDescrName() const;
    virtual QString itemClassName() const;
    virtual Editor* factorizeEditor(QWidget* parent) const;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYOPTANALYSISEDITOR_H
