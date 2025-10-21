#include "localcoordinatesystembaseactionmanager.h"

#include <QWidget>
#include <QAction>
#include <QMenu>

#include "mainwindow.h"
#include "datablocks/localcoordinatesystem.h"
#include "datablocks/trajectory.h"
#include "datablocks/mounting.h"

#include "localcoordinatesystemactions.h"

#include "gui/localcoordinatesystempointdetailseditor.h"

namespace StereoVisionApp {

LocalCoordinateSystemBaseActionManager::LocalCoordinateSystemBaseActionManager(QObject *parent) :
	DatablockActionManager(parent)
{

}

QString LocalCoordinateSystemBaseActionManager::ActionManagerClassName() const {
	return LocalCoordinateSystemBaseActionManager::staticMetaObject.className();
}
QString LocalCoordinateSystemBaseActionManager::itemClassName() const {
	return LocalCoordinateSystem::staticMetaObject.className();
}

QList<QAction*> LocalCoordinateSystemBaseActionManager::factorizeItemContextActions(QObject* parent, DataBlock* p) const {

	LocalCoordinateSystem* lcs = qobject_cast<LocalCoordinateSystem*>(p);

	if (lcs == nullptr) {
		return {};
	}

	QWidget* w = qobject_cast<QWidget*>(parent);

	if (w != nullptr) {
		w = w->window();
	}

	MainWindow* mw = qobject_cast<MainWindow*>(w);

	QList<QAction*> lst;

	if (mw != nullptr) {
		QAction* edit = new QAction(tr("Point details"), parent);
		connect(edit, &QAction::triggered, [mw, lcs] () {

			Editor* e = mw->openEditor(LocalCoordinateSystemPointDetailsEditor::staticMetaObject.className());
			LocalCoordinateSystemPointDetailsEditor* le = qobject_cast<LocalCoordinateSystemPointDetailsEditor*>(e);

			le->setLocalCoordinateSystem(lcs);

		});

		lst << edit;
	}

	QAction* alignToPoints = new QAction(tr("Align to points"), parent);
	connect(alignToPoints, &QAction::triggered, [lcs] () {
		alignLocalCoordinateSystemToPoints({lcs->internalId()}, lcs->getProject());
	});
	lst.append(alignToPoints);

    QAction* assignToTrajectory = createAssignToTrajectoryAction(parent, lcs->getProject(), {lcs});

    lst.append(assignToTrajectory);

    QAction* assignToMounting = createAssignToMountingAction(parent, lcs->getProject(), {lcs});

    lst.append(assignToMounting);

	QAction* clearOptimized = new QAction(tr("Clear optimized"), parent);
	connect(clearOptimized, &QAction::triggered, [lcs] () {
		lcs->clearOptimized();
	});
	lst.append(clearOptimized);

	QAction* remove = new QAction(tr("Remove"), parent);
	connect(remove, &QAction::triggered, [lcs] () {
		Project* p = lcs->getProject();

		if (p != nullptr) {
			p->clearById(lcs->internalId());
		}
	});
	lst.append(remove);

	return lst;
}

QList<QAction*> LocalCoordinateSystemBaseActionManager::factorizeMultiItemsContextActions(QObject* parent, Project* p, QModelIndexList const& projectIndex) const {

	QVector<LocalCoordinateSystem*> lcss;
	QVector<qint64> lcsids;
	lcss.reserve(projectIndex.count());
	lcsids.reserve(projectIndex.count());

	for (QModelIndex const& id : projectIndex) {
		LocalCoordinateSystem* lm = qobject_cast<LocalCoordinateSystem*>(p->getById(p->data(id, Project::IdRole).toInt()));

		if (lm != nullptr) {
			lcss.push_back(lm);
			lcsids.push_back(lm->internalId());
		}
	}

	QWidget* w = qobject_cast<QWidget*>(parent);

	if (w != nullptr) {
		w = w->window();
	}

	QList<QAction*> lst;

    if (lcss.isEmpty()) {
        return lst;
    }

    QAction* assignToTrajectory = createAssignToTrajectoryAction(parent, lcss[0]->getProject(), lcss);

    lst.append(assignToTrajectory);

    QAction* assignToMounting = createAssignToMountingAction(parent, lcss[0]->getProject(), lcss);

    lst.append(assignToMounting);

	QAction* clearOptimized = new QAction(tr("Clear optimized"), parent);
	connect(clearOptimized, &QAction::triggered, [lcss] () {
		for (LocalCoordinateSystem* lcs : lcss) {
			lcs->clearOptimized();
		}
	});
	lst.append(clearOptimized);

	return lst;

}

QAction* LocalCoordinateSystemBaseActionManager::createAssignToTrajectoryAction(QObject* parent, Project* p, const QVector<LocalCoordinateSystem *> &lcss) const {

    QAction* assignToTrajectory = new QAction(tr("Assign to trajectory"), parent);
    QMenu* trajMenu = new QMenu();
    connect(assignToTrajectory, &QObject::destroyed, trajMenu, &QObject::deleteLater);

    QVector<qint64> trajIds = p->getIdsByClass(Trajectory::staticMetaObject.className());

    for(qint64 trajId : trajIds) {

        Trajectory* traj = p->getDataBlock<Trajectory>(trajId);

        if (traj != nullptr) {
            QAction* toTraj = new QAction(traj->objectName(), assignToTrajectory);
            connect(toTraj, &QAction::triggered, [trajId, lcss] () {
                for (LocalCoordinateSystem* lcs : lcss) {
                    lcs->assignTrajectory(trajId);
                }
            });
            trajMenu->addAction(toTraj);
        }
    }

    QAction* clearTraj = new QAction(tr("Clear trajectory"), assignToTrajectory);
    connect(clearTraj, &QAction::triggered, [lcss] () {
        for (LocalCoordinateSystem* lcs : lcss) {
            lcs->assignTrajectory(-1);
        }
    });
    trajMenu->addAction(clearTraj);



    assignToTrajectory->setMenu(trajMenu);

    return assignToTrajectory;
}
QAction* LocalCoordinateSystemBaseActionManager::createAssignToMountingAction(QObject* parent, Project* p, const QVector<LocalCoordinateSystem *> &lcss) const {
    QAction* assignToMounting = new QAction(tr("Assign to mounting"), parent);
    QMenu* mountingMenu = new QMenu();
    connect(assignToMounting, &QObject::destroyed, mountingMenu, &QObject::deleteLater);

    QVector<qint64> mountingIds = p->getIdsByClass(Mounting::staticMetaObject.className());

    for(qint64 mountingId : mountingIds) {

        Mounting* mounting = p->getDataBlock<Mounting>(mountingId);

        if (mounting != nullptr) {
            QAction* toTraj = new QAction(mounting->objectName(), assignToMounting);
            connect(toTraj, &QAction::triggered, [mountingId, lcss] () {
                for (LocalCoordinateSystem* lcs : lcss) {
                    lcs->assignMounting(mountingId);
                }
            });
            mountingMenu->addAction(toTraj);
        }
    }

    QAction* clearMounting = new QAction(tr("Clear mounting"), assignToMounting);
    connect(clearMounting, &QAction::triggered, [lcss] () {
        for (LocalCoordinateSystem* lcs : lcss) {
            lcs->assignMounting(-1);
        }
    });
    mountingMenu->addAction(clearMounting);



    assignToMounting->setMenu(mountingMenu);

    return assignToMounting;
}

} // namespace StereoVisionApp
