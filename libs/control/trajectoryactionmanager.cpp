#include "trajectoryactionmanager.h"

#include "datablocks/trajectory.h"

#include "gui/dialogs/trajectoryeditpositionoptionsdialog.h"
#include "gui/dialogs/trajectoryeditorientationoptionsdialog.h"

#include "gui/dialogs/trajectoryeditaccelerometeroptionsdialog.h"
#include "gui/dialogs/trajectoryeditgyrooptionsdialog.h"

#include "gui/sparsealignementeditor.h"

#include "gui/openGlDrawables/opengldrawabletrajectory.h"

#include "control/mainwindow.h"

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

    QAction* setGyroData = new QAction(tr("set gyro data"), parent);
    connect(setGyroData, &QAction::triggered, traj, [traj] () {

        MainWindow* mw = MainWindow::getActiveMainWindow();

        TrajectoryEditGyroOptionsDialog::ConfigureTrajectoryGyroOptions(traj, mw);

    });
    actions.append(setGyroData);

    QAction* viewTrajectory = new QAction(tr("view trajectory"), parent);
    connect(viewTrajectory, &QAction::triggered, traj, [traj] () {

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

        OpenGlDrawable* drawable = sae->getDrawable(traj_drawable_name);
        OpenGlDrawableTrajectory* drawableTrajectory = qobject_cast<OpenGlDrawableTrajectory*>(drawable);

        if (drawable == nullptr) {
            drawableTrajectory = new OpenGlDrawableTrajectory();
            sae->addDrawable(traj_drawable_name, drawableTrajectory);

        } else if (drawableTrajectory == nullptr) {
            //a drawable already exist with the name and is not a OpenGlDrawableTrajectory

            return; //error, this is not supposed to happen

        }

        drawableTrajectory->setTrajectory(traj);

        QObject::connect(traj, &Trajectory::trajectoryDataChanged, drawableTrajectory, [drawableTrajectory, traj] () {
            drawableTrajectory->setTrajectory(traj);
        });
    });
    actions.append(viewTrajectory);

    return actions;
}

QString TrajectoryActionManager::ActionManagerClassName() const {
    return TrajectoryActionManager::staticMetaObject.className();
}
QString TrajectoryActionManager::itemClassName() const {
    return Trajectory::staticMetaObject.className();
}

} // namespace StereoVisionApp
