#include "landmarkbaseactionmanager.h"

#include "landmarkbasedactions.h"

#include "datablocks/landmark.h"
#include "datablocks/angleconstrain.h"
#include "datablocks/distanceconstrain.h"
#include "datablocks/localcoordinatesystem.h"

#include "gui/landmarkpointdetailseditor.h"

#include "mainwindow.h"

#include <QWidget>
#include <QAction>
#include <QMenu>
#include <QFileDialog>

namespace StereoVisionApp {

LandmarkBaseActionManager::LandmarkBaseActionManager(QObject *parent) :
	DatablockActionManager(parent)
{

}

QString LandmarkBaseActionManager::ActionManagerClassName() const {
	return "StereoVisionApp::LandmarkBaseActionManager";
}
QString LandmarkBaseActionManager::itemClassName() const {
	return LandmarkFactory::landmarkClassName();
}

QList<QAction*> LandmarkBaseActionManager::factorizeClassContextActions(QObject* parent, Project* p) const {

    if (p == nullptr) {
        return {};
    }

    QWidget* w = qobject_cast<QWidget*>(parent);

    if (w != nullptr) {
        w = w->window();
    }

    MainWindow* mw = qobject_cast<MainWindow*>(w);

    QList<QAction*> lst = DatablockActionManager::factorizeClassContextActions(parent, p); //basic actions like adding an item

    QVector<qint64> lmids = p->getIdsByClass(Landmark::staticMetaObject.className());

    if (lmids.size() > 0) {
        QAction* exportLandmarksToCSVAction = new QAction("export to csv", parent);

        connect(exportLandmarksToCSVAction, &QAction::triggered, p, [w, p, lmids] () {
            QString f = QFileDialog::getSaveFileName(w, tr("Export landmarks to csv"), QString(), "csv files (*.csv)");
            if (!f.isEmpty()) {
                exportLandmarksToCsv(p, lmids, f);
            }
        });

        lst.append(exportLandmarksToCSVAction);
    }

    return lst;
}
QList<QAction*> LandmarkBaseActionManager::factorizeItemContextActions(QObject* parent, DataBlock* p) const {

	Landmark* lm = qobject_cast<Landmark*>(p);

	if (lm == nullptr) {
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
		connect(edit, &QAction::triggered, [mw, lm] () {

			Editor* e = mw->openEditor(LandmarkPointDetailsEditor::staticMetaObject.className());
			LandmarkPointDetailsEditor* le = qobject_cast<LandmarkPointDetailsEditor*>(e);

			le->setLandmark(lm);

		});

		lst << edit;
	}

	QAction* addToLocalCoordinateSystem = createAddToLocalCoordinateSystemAction(parent, lm->getProject(), {lm});

	lst.append(addToLocalCoordinateSystem);

	QAction* clearOptimized = new QAction(tr("Clear optimized"), parent);
	connect(clearOptimized, &QAction::triggered, [lm] () {
		lm->clearOptimized();
	});
	lst.append(clearOptimized);

	QAction* remove = new QAction(tr("Remove"), parent);
	connect(remove, &QAction::triggered, [lm] () {
		Project* p = lm->getProject();

		if (p != nullptr) {
			p->clearById(lm->internalId());
		}
	});
	lst.append(remove);

	return lst;
}

QList<QAction*> LandmarkBaseActionManager::factorizeMultiItemsContextActions(QObject* parent, Project* p, QModelIndexList const& projectIndex) const {

	QVector<Landmark*> lms;
	QVector<qint64> lmids;
	lms.reserve(projectIndex.count());
	lmids.reserve(projectIndex.count());

	for (QModelIndex const& id : projectIndex) {
		Landmark* lm = qobject_cast<Landmark*>(p->getById(p->data(id, Project::IdRole).toInt()));

		if (lm != nullptr) {
			lms.push_back(lm);
			lmids.push_back(lm->internalId());
		}
	}

	QWidget* w = qobject_cast<QWidget*>(parent);

	if (w != nullptr) {
		w = w->window();
	}

	QList<QAction*> lst;

	QAction* assignToDistanceConstrain = createAssignToDistanceConstrainAction(parent, p, lms);

	if (assignToDistanceConstrain != nullptr) {
		lst.append(assignToDistanceConstrain);
	}

	QAction* assignToAngleConstrain = createAssignToAngleConstrainAction(parent, p, lms);

	if (assignToAngleConstrain != nullptr) {
		lst.append(assignToAngleConstrain);
	}

	QAction* exportLandmarksToCSVAction = new QAction("export to csv", parent);

	connect(exportLandmarksToCSVAction, &QAction::triggered, p, [w, p, lmids] () {
		QString f = QFileDialog::getSaveFileName(w, tr("Export landmarks to csv"), QString(), "csv files (*.csv)");
		if (!f.isEmpty()) {
			exportLandmarksToCsv(p, lmids, f);
		}
	});

	lst.append(exportLandmarksToCSVAction);

	QAction* clearOptimized = new QAction(tr("Clear optimized"), parent);
	connect(clearOptimized, &QAction::triggered, [lms] () {
		for (Landmark* lm : lms) {
			lm->clearOptimized();
		}
	});
	lst.append(clearOptimized);

	return lst;

}

QAction* LandmarkBaseActionManager::createAssignToDistanceConstrainAction(QObject* parent, Project* p, const QVector<Landmark *> &lms) const {

	bool ok = true;

	QAction* assignToDistanceConstrain = new QAction(tr("Assign to distance constrain"), parent);

	if (lms.size() != 2) {
		assignToDistanceConstrain->deleteLater();
		return nullptr;
	}

	QMenu* constaintsMenu = new QMenu();
	connect(assignToDistanceConstrain, &QObject::destroyed, constaintsMenu, &QObject::deleteLater);

	QVector<qint64> constraintsIds = p->getIdsByClass(DistanceConstrain::staticMetaObject.className());

	for (qint64 constrainId : constraintsIds) {

		DistanceConstrain* constraint = qobject_cast<DistanceConstrain*>(p->getById(constrainId));

		if (constraint != nullptr) {

			QVector<qint64> pairs = constraint->listTypedSubDataBlocks(DistanceLandmarksPair::staticMetaObject.className());

			for (qint64 id_p : pairs) {
				DistanceLandmarksPair* p = constraint->getLandmarksPair(id_p);

				if (p != nullptr) {
					if ((p->getNthLandmarkId(0) == lms[0]->internalId() and
							p->getNthLandmarkId(1) == lms[1]->internalId()) or
						(p->getNthLandmarkId(0) == lms[1]->internalId() and
							p->getNthLandmarkId(1) == lms[0]->internalId())) {
						ok = false;
						break;
					}
				}
			}

			QAction* toConstrain = new QAction(constraint->objectName(), assignToDistanceConstrain);
			connect(toConstrain, &QAction::triggered, constraint, [constraint, lms] () {
				constraint->insertLandmarksPair(lms[0]->internalId(), lms[1]->internalId());
			});
			constaintsMenu->addAction(toConstrain);
		}

	}

	if (ok) {
		assignToDistanceConstrain->setMenu(constaintsMenu);
		return assignToDistanceConstrain;
	} else {
		assignToDistanceConstrain->deleteLater();
		return nullptr;
	}

	return nullptr;
}

QAction* LandmarkBaseActionManager::createAssignToAngleConstrainAction(QObject* parent, Project* p, const QVector<Landmark *> &lms) const {

	bool ok = true;

	QAction* assignToAngleConstrain = new QAction(tr("Assign to angle constrain"), parent);

	if (lms.size() != 3) {
		assignToAngleConstrain->deleteLater();
		return nullptr;
	}

	QMenu* rigMenu = new QMenu();
	connect(assignToAngleConstrain, &QObject::destroyed, rigMenu, &QObject::deleteLater);

	QVector<qint64> constraintsIds = p->getIdsByClass(AngleConstrain::staticMetaObject.className());

	for (qint64 constrainId : constraintsIds) {

		AngleConstrain* constraint = qobject_cast<AngleConstrain*>(p->getById(constrainId));

		if (constraint != nullptr) {

			QAction* toConstrain = new QAction(constraint->objectName(), assignToAngleConstrain);
			connect(toConstrain, &QAction::triggered, constraint, [constraint, lms] () {
				constraint->insertLandmarksTriplet(lms[0]->internalId(), lms[1]->internalId(), lms[2]->internalId());
			});
			rigMenu->addAction(toConstrain);
		}

	}

	if (ok) {
		assignToAngleConstrain->setMenu(rigMenu);
		return assignToAngleConstrain;
	} else {
		assignToAngleConstrain->deleteLater();
		return nullptr;
	}

	return nullptr;
}

QAction* LandmarkBaseActionManager::createAddToLocalCoordinateSystemAction(QObject* parent, Project* p, const QVector<Landmark *> &lms) const {


	bool ok = true;

	QAction* assignToLocalReferenceSystem = new QAction(tr("Assign to local coordinate system"), parent);

	if (lms.isEmpty()) {
		assignToLocalReferenceSystem->deleteLater();
		return nullptr;
	}

	QMenu* assignMenu = new QMenu();
	connect(assignToLocalReferenceSystem, &QObject::destroyed, assignMenu, &QObject::deleteLater);

	QVector<qint64> lmIds;
	lmIds.reserve(lms.size());

	for (Landmark* lm : lms) {
		lmIds.push_back(lm->internalId());
	}

	QVector<qint64> constraintsIds = p->getIdsByClass(LocalCoordinateSystem::staticMetaObject.className());

	for (qint64 constrainId : constraintsIds) {

		LocalCoordinateSystem* constraint = qobject_cast<LocalCoordinateSystem*>(p->getById(constrainId));

		if (constraint != nullptr) {

			QAction* toConstrain = new QAction(constraint->objectName(), assignToLocalReferenceSystem);
			connect(toConstrain, &QAction::triggered, constraint, [constrainId, lmIds, p] () {
				attachLandmarkToLocalCoordinateSystem(p, lmIds, constrainId);
			});
			assignMenu->addAction(toConstrain);
		}

	}

	if (ok) {
		assignToLocalReferenceSystem->setMenu(assignMenu);
		return assignToLocalReferenceSystem;
	} else {
		assignToLocalReferenceSystem->deleteLater();
		return nullptr;
	}

	return nullptr;

}

} // namespace StereoVisionApp
