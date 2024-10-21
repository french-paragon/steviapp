#ifndef STEREOVISIONAPP_TRAJECTORYALIGNEMENTANALYSISEDITOR_H
#define STEREOVISIONAPP_TRAJECTORYALIGNEMENTANALYSISEDITOR_H

#include <QWidget>
#include <QString>
#include <QMap>

#include "./editor.h"

class QCustomPlot;

namespace StereoVisionApp {

class Trajectory;

class TrajectoryAlignementAnalysisEditor : public Editor
{
    Q_OBJECT
public:
    explicit TrajectoryAlignementAnalysisEditor(QWidget *parent = nullptr);

    void setTrajectory(Trajectory* trj);

Q_SIGNALS:

protected:

    void reconfigurePlots();

    Trajectory* _trajectory;

    QCustomPlot* _speedDeltasPlot;

    QCustomPlot* _orientationDeltasPlot;

};


class TrajectoryAlignementAnalysisEditorFactory : public EditorFactory
{
    Q_OBJECT
public:
    explicit TrajectoryAlignementAnalysisEditorFactory(QObject *parent = nullptr);


    virtual QString TypeDescrName() const;
    virtual QString itemClassName() const;
    virtual Editor* factorizeEditor(QWidget* parent) const;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYALIGNEMENTANALYSISEDITOR_H
