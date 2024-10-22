#ifndef STEREOVISIONAPP_TRAJECTORYSEQUENCEVIEWEDITOR_H
#define STEREOVISIONAPP_TRAJECTORYSEQUENCEVIEWEDITOR_H

#include <QWidget>
#include <QString>
#include <QMap>

#include "./editor.h"

class QCustomPlot;

namespace StereoVisionApp {

class Trajectory;

class TrajectorySequenceViewEditor : public Editor
{
    Q_OBJECT
public:
    explicit TrajectorySequenceViewEditor(QWidget *parent = nullptr);

    void setTrajectory(Trajectory* trj);

Q_SIGNALS:

protected:

    void reconfigurePlots();

    Trajectory* _trajectory;

    QCustomPlot* _positionPlot;

    QCustomPlot* _orientationPlot;

};


class TrajectorySequenceViewEditorFactory : public EditorFactory
{
    Q_OBJECT
public:
    explicit TrajectorySequenceViewEditorFactory(QObject *parent = nullptr);


    virtual QString TypeDescrName() const;
    virtual QString itemClassName() const;
    virtual Editor* factorizeEditor(QWidget* parent) const;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYSEQUENCEVIEWEDITOR_H
