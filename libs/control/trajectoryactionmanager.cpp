#include "trajectoryactionmanager.h"

#include "datablocks/trajectory.h"

#include "gui/dialogs/trajectoryeditpositionoptionsdialog.h"
#include "gui/dialogs/trajectoryeditorientationoptionsdialog.h"

#include "gui/dialogs/trajectoryeditaccelerometeroptionsdialog.h"
#include "gui/dialogs/trajectoryeditgyrooptionsdialog.h"

#include "gui/sparsealignementeditor.h"
#include "gui/trajectorysequencevieweditor.h"
#include "gui/trajectoryoptanalysiseditor.h"
#include "gui/trajectoryalignementanalysiseditor.h"

#include "gui/openGlDrawables/opengldrawabletrajectory.h"

#include "control/mainwindow.h"

#include "trajectoryactions.h"

#include <QAction>

namespace StereoVisionApp {

TrajectoryActionManager::TrajectoryActionManager(QObject *parent) : DatablockActionManager(parent)
{

}

QList<QAction*> TrajectoryActionManager::factorizeClassContextActions(QObject* parent, Project* p) const {
    return DatablockActionManager::factorizeClassContextActions(parent, p);
}

QList<QAction*> TrajectoryActionManager::factorizeItemContextActions(QObject* parent, DataBlock* item) const {

    QList<QAction*> actions = DatablockActionManager::factorizeItemContextActions(parent, item);

    Trajectory* traj = qobject_cast<Trajectory*>(item);

    if (traj == nullptr) {
        return actions;
    }

    QAction* setPositionData = new QAction(tr("set position data"), parent);
    connect(setPositionData, &QAction::triggered, traj, [traj] () {

        MainWindow* mw = MainWindow::getActiveMainWindow();

        TrajectoryEditPositionOptionsDialog::ConfigureTrajectoryPositionOptions(traj, mw);

    });
    actions.append(setPositionData);

    QAction* setOrientationData = new QAction(tr("set orientation data"), parent);
    connect(setOrientationData, &QAction::triggered, traj, [traj] () {

        MainWindow* mw = MainWindow::getActiveMainWindow();

        TrajectoryEditOrientationOptionsDialog::ConfigureTrajectoryOrientationOptions(traj, mw);

    });
    actions.append(setOrientationData);

    QAction* setAccelerometerData = new QAction(tr("set accelerometer data"), parent);
    connect(setAccelerometerData, &QAction::triggered, traj, [traj] () {

        MainWindow* mw = MainWindow::getActiveMainWindow();

        TrajectoryEditAccelerometerOptionsDialog::ConfigureTrajectoryAccelerometerOptions(traj, mw);

    });
    actions.append(setAccelerometerData);

    QAction* accMountingAction = new QAction(tr("Set accelerometer mounting"), parent);
    connect(accMountingAction, &QAction::triggered, traj, [traj] () {
        setAccelerometerMounting(traj);
    });
    actions.append(accMountingAction);

    QAction* setGyroData = new QAction(tr("set gyro data"), parent);
    connect(setGyroData, &QAction::triggered, traj, [traj] () {

        MainWindow* mw = MainWindow::getActiveMainWindow();

        TrajectoryEditGyroOptionsDialog::ConfigureTrajectoryGyroOptions(traj, mw);

    });
    actions.append(setGyroData);

    QAction* gyroMountingAction = new QAction(tr("Set gyro mounting"), parent);
    connect(gyroMountingAction, &QAction::triggered, traj, [traj] () {
        setGyroMounting(traj);
    });
    actions.append(gyroMountingAction);

    QAction* viewTrajectoryAction = new QAction(tr("view trajectory"), parent);
    connect(viewTrajectoryAction, &QAction::triggered, traj, [traj] () {
        viewTrajectory(traj,false);
    });
    actions.append(viewTrajectoryAction);

    QAction* analyzeTrajectoryAction = new QAction(tr("view trajectory sequence"), parent);
    connect(analyzeTrajectoryAction, &QAction::triggered, traj, [traj] () {

        MainWindow* mw = MainWindow::getActiveMainWindow();

        Editor* editor = mw->openEditor(TrajectorySequenceViewEditor::staticMetaObject.className());

        TrajectorySequenceViewEditor* tsve = qobject_cast<TrajectorySequenceViewEditor*>(editor);

        if (tsve == nullptr) {
            return;
        }

        tsve->setTrajectory(traj);
    });
    actions.append(analyzeTrajectoryAction);

    if (traj->hasOptimizedTrajectory()) {

        QAction* viewOptTrajectoryAction = new QAction(tr("view optimized trajectory"), parent);
        connect(viewOptTrajectoryAction, &QAction::triggered, traj, [traj] () {
            viewTrajectory(traj,true);
        });
        actions.append(viewOptTrajectoryAction);

        QAction* exportOptTrajectoryAction = new QAction(tr("export optimized trajectory"), parent);
        connect(exportOptTrajectoryAction, &QAction::triggered, traj, [traj] () {
            exportOptimizedTrajectory(traj,"");
        });
        actions.append(exportOptTrajectoryAction);

        QAction* analyzeOptTrajectoryAction = new QAction(tr("analyze optimized trajectory"), parent);
        connect(analyzeOptTrajectoryAction, &QAction::triggered, traj, [traj] () {

            MainWindow* mw = MainWindow::getActiveMainWindow();

            Editor* editor = mw->openEditor(TrajectoryOptAnalysisEditor::staticMetaObject.className());

            TrajectoryOptAnalysisEditor* toae = qobject_cast<TrajectoryOptAnalysisEditor*>(editor);

            if (toae == nullptr) {
                return;
            }

            toae->setTrajectory(traj);
        });
        actions.append(analyzeOptTrajectoryAction);

    }

    QAction* analyzeTrajectoryAlignmentAction = new QAction(tr("analyze trajectory alignment"), parent);
    connect(analyzeTrajectoryAlignmentAction, &QAction::triggered, traj, [traj] () {

        MainWindow* mw = MainWindow::getActiveMainWindow();

        Editor* editor = mw->openEditor(TrajectoryAlignementAnalysisEditor::staticMetaObject.className());

        TrajectoryAlignementAnalysisEditor* taae = qobject_cast<TrajectoryAlignementAnalysisEditor*>(editor);

        if (taae == nullptr) {
            return;
        }

        taae->setTrajectory(traj);
    });
    actions.append(analyzeTrajectoryAlignmentAction);

    return actions;
}

QString TrajectoryActionManager::ActionManagerClassName() const {
    return TrajectoryActionManager::staticMetaObject.className();
}
QString TrajectoryActionManager::itemClassName() const {
    return Trajectory::staticMetaObject.className();
}

} // namespace StereoVisionApp
