#include "trajectoryactionmanager.h"

#include "datablocks/trajectory.h"

#include "gui/dialogs/trajectoryeditpositionoptionsdialog.h"
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

    QAction* exportTrajectoryAction = new QAction(tr("export trajectory"), parent);
    connect(exportTrajectoryAction, &QAction::triggered, traj, [traj] () {

        MainWindow* mw = MainWindow::getActiveMainWindow();

        if (mw == nullptr) {
            return;
        }

        constexpr int Optimized = 0;
        constexpr int NotOptimized = 1;

        constexpr int ECEF = 0;
        constexpr int Geographic = 1;

        QDialog exportOptionDialog(mw);
        QVBoxLayout layout(&exportOptionDialog);
        QFormLayout formLayout;
        QComboBox optimizedOptionBox;
        QComboBox geoBox;
        QDialogButtonBox buttonBox(QDialogButtonBox::Ok|QDialogButtonBox::Cancel);

        if (traj->hasOptimizedTrajectory()) {
            optimizedOptionBox.addItem(QObject::tr("Optimized"), Optimized);
        }
        optimizedOptionBox.addItem(QObject::tr("Initial"), NotOptimized);

        geoBox.addItem(QObject::tr("Export in Geographic coordinates"), Geographic);
        geoBox.addItem(QObject::tr("Export in ECEF coordinates"), ECEF);

        connect(&buttonBox, &QDialogButtonBox::accepted, &exportOptionDialog, &QDialog::accept);
        connect(&buttonBox, &QDialogButtonBox::rejected, &exportOptionDialog, &QDialog::reject);

        formLayout.addRow(QObject::tr("Optimized:"), &optimizedOptionBox);
        formLayout.addRow(QObject::tr("Representation:"), &geoBox);
        layout.addLayout(&formLayout);
        layout.addWidget(&buttonBox);

        int code = exportOptionDialog.exec();

        if (code != QDialog::Accepted) {
            return;
        }

        bool exportOptimized = optimizedOptionBox.currentData().toInt() == Optimized;
        int exportFrame = geoBox.currentData().toInt();

        if (exportFrame == ECEF) {
            exportTrajectory(traj,"", exportOptimized);
        } else if (exportFrame == Geographic) {
            exportTrajectoryGeographic(traj,"", exportOptimized);
        }
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

} // namespace StereoVisionApp
