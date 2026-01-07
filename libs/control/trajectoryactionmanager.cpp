#include "trajectoryactionmanager.h"

#include "datablocks/trajectory.h"
#include "datablocks/mounting.h"

#include "gui/dialogs/trajectoryeditpositionoptionsdialog.h"
#include "gui/dialogs/trajectoryeditgpsoptionsdialog.h"
#include "gui/dialogs/trajectoryeditorientationoptionsdialog.h"

#include "gui/dialogs/trajectoryeditaccelerometeroptionsdialog.h"
#include "gui/dialogs/trajectoryeditgyrooptionsdialog.h"

#include "gui/sparsealignementeditor.h"
#include "gui/trajectorysequencevieweditor.h"
#include "gui/trajectorycomparisoneditor.h"
#include "gui/trajectoryalignementanalysiseditor.h"

#include "gui/openGlDrawables/opengldrawabletrajectory.h"

#include "control/mainwindow.h"

#include "trajectoryactions.h"

#include <QAction>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QMenu>

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

    QAction* setGPSData = new QAction(tr("set GPS data"), parent);
    connect(setGPSData, &QAction::triggered, traj, [traj] () {

        MainWindow* mw = MainWindow::getActiveMainWindow();

        TrajectoryEditGpsOptionsDialog::ConfigureTrajectoryGpsOptions(traj, mw);

    });
    actions.append(setGPSData);

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

    QAction* accMountingAction = new QAction(tr("Set accelerometer transform"), parent);
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

    QAction* gyroMountingAction = new QAction(tr("Set gyro transform"), parent);
    connect(gyroMountingAction, &QAction::triggered, traj, [traj] () {
        setGyroMounting(traj);
    });
    actions.append(gyroMountingAction);

    QAction* configureInsStochasticProcesses = new QAction(tr("configure stochastic processes"), parent);
    QMenu* configureInsStochasticProcessesMenu = new QMenu();

    configureInsStochasticProcesses->setMenu(configureInsStochasticProcessesMenu);

    QAction* gyroBiasProcesses = new QAction(tr("gyro bias"), parent);
    connect(gyroBiasProcesses, &QAction::triggered, traj, [traj] () {
        editTrajectoryINSStochasticProcesses(traj, TrajectoryInsStochasticProcessType::GyroBias);
    });
    configureInsStochasticProcessesMenu->addAction(gyroBiasProcesses);

    QAction* gyroScaleProcesses = new QAction(tr("gyro scale"), parent);
    connect(gyroScaleProcesses, &QAction::triggered, traj, [traj] () {
        editTrajectoryINSStochasticProcesses(traj, TrajectoryInsStochasticProcessType::GyroScale);
    });
    configureInsStochasticProcessesMenu->addAction(gyroScaleProcesses);

    QAction* accBiasProcesses = new QAction(tr("acc bias"), parent);
    connect(accBiasProcesses, &QAction::triggered, traj, [traj] () {
        editTrajectoryINSStochasticProcesses(traj, TrajectoryInsStochasticProcessType::AccBias);
    });
    configureInsStochasticProcessesMenu->addAction(accBiasProcesses);

    QAction* accScaleProcesses = new QAction(tr("acc scale"), parent);
    connect(accScaleProcesses, &QAction::triggered, traj, [traj] () {
        editTrajectoryINSStochasticProcesses(traj, TrajectoryInsStochasticProcessType::AccScale);
    });
    configureInsStochasticProcessesMenu->addAction(accScaleProcesses);

    actions.append(configureInsStochasticProcesses);

    QAction* assignGpsMounting = createAssignToMountingAction(parent, traj->getProject(), {traj}, AssignMountMode::GPS);
    QAction* assignInsMounting = createAssignToMountingAction(parent, traj->getProject(), {traj}, AssignMountMode::INS);

    actions.append(assignGpsMounting);
    actions.append(assignInsMounting);

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

    QAction* exportTrajectoryAction = new QAction(tr("export trajectory"), parent);
    connect(exportTrajectoryAction, &QAction::triggered, traj, [traj] () {
        exportTrajectory(traj);
    });
    actions.append(exportTrajectoryAction);

    if (traj->hasOptimizedTrajectory()) {

        QAction* viewOptTrajectoryAction = new QAction(tr("view optimized trajectory"), parent);
        connect(viewOptTrajectoryAction, &QAction::triggered, traj, [traj] () {
            viewTrajectory(traj,true);
        });
        actions.append(viewOptTrajectoryAction);

        QAction* analyzeOptTrajectoryAction = new QAction(tr("analyze optimized trajectory"), parent);
        connect(analyzeOptTrajectoryAction, &QAction::triggered, traj, [traj] () {

            MainWindow* mw = MainWindow::getActiveMainWindow();

            Editor* editor = mw->openEditor(TrajectoryComparisonEditor::staticMetaObject.className());

            TrajectoryComparisonEditor* toae = qobject_cast<TrajectoryComparisonEditor*>(editor);

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

QList<QAction*> TrajectoryActionManager::factorizeMultiItemsContextActions(QObject* parent, Project* p, QModelIndexList const& projectIndex) const {

    QVector<Trajectory*> trajs;
    QVector<qint64> trajIds;
    trajs.reserve(projectIndex.count());
    trajIds.reserve(projectIndex.count());

    for (QModelIndex const& id : projectIndex) {
        qint64 trajid = p->data(id, Project::IdRole).toInt();
        Trajectory* traj = qobject_cast<Trajectory*>(p->getById(trajid));

        if (traj != nullptr) {
            trajs.push_back(traj);
            trajIds.push_back(trajid);
        }
    }

    QList<QAction*> lst;

    if (trajs.size() == 2) {

        QAction* compareTrajectories = new QAction(tr("Compare trajectories"), parent);
        connect(compareTrajectories, &QAction::triggered, trajs[0], [trajs] () {

            MainWindow* mw = MainWindow::getActiveMainWindow();

            Editor* editor = mw->openEditor(TrajectoryComparisonEditor::staticMetaObject.className());

            TrajectoryComparisonEditor* toae = qobject_cast<TrajectoryComparisonEditor*>(editor);

            if (toae == nullptr) {
                return;
            }

            toae->setTrajectories(trajs[0], trajs[1]);

        });
        lst.append(compareTrajectories);

    }

    return lst;
}

QString TrajectoryActionManager::ActionManagerClassName() const {
    return TrajectoryActionManager::staticMetaObject.className();
}
QString TrajectoryActionManager::itemClassName() const {
    return Trajectory::staticMetaObject.className();
}

QAction* TrajectoryActionManager::createAssignToMountingAction(QObject* parent,
                                      StereoVisionApp::Project* p,
                                      QVector<Trajectory*> const& trajs,
                                      AssignMountMode mode) const {

    QString actionText = (mode == GPS) ? tr("Assign gps mounting") : tr("Assign ins mounting");
    QAction* assignToMounting = new QAction(actionText, parent);
    QMenu* mountingMenu = new QMenu();
    connect(assignToMounting, &QObject::destroyed, mountingMenu, &QObject::deleteLater);

    QVector<qint64> mountingIds = p->getIdsByClass(StereoVisionApp::Mounting::staticMetaObject.className());

    for(qint64 mountingId : mountingIds) {

        StereoVisionApp::Mounting* m = qobject_cast<StereoVisionApp::Mounting*>(p->getById(mountingId));

        if (m != nullptr) {
            QAction* toMounting = new QAction(m->objectName(), assignToMounting);
            if (mode == GPS) {
                connect(toMounting, &QAction::triggered, [mountingId, trajs] () {
                    for (Trajectory* traj : trajs) {
                        traj->assignGpsMounting(mountingId);
                    }
                });
            } else {
                connect(toMounting, &QAction::triggered, [mountingId, trajs] () {
                    for (Trajectory* traj : trajs) {
                        traj->assignInsMounting(mountingId);
                    }
                });
            }
            mountingMenu->addAction(toMounting);
        }
    }

    QAction* clearMounting = new QAction(tr("Clear"), assignToMounting);

    if (mode == GPS) {
        connect(clearMounting, &QAction::triggered, [trajs] () {
            for (Trajectory* traj : trajs) {
                traj->assignGpsMounting(-1);
            }
        });
    } else {
        connect(clearMounting, &QAction::triggered, [trajs] () {
            for (Trajectory* traj : trajs) {
                traj->assignInsMounting(-1);
            }
        });
    }

    mountingMenu->addAction(clearMounting);

    assignToMounting->setMenu(mountingMenu);

    return assignToMounting;
}

} // namespace StereoVisionApp
