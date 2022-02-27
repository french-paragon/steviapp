#include "landmarkbaseactionmanager.h"

#include "datablocks/landmark.h"
#include "datablocks/angleconstrain.h"
#include "datablocks/distanceconstrain.h"

#include <QWidget>
#include <QAction>
#include <QMenu>

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

QList<QAction*> LandmarkBaseActionManager::factorizeMultiItemsContextActions(QObject* parent, Project* p, QModelIndexList const& projectIndex) const {

	QVector<Landmark*> lms;
	lms.reserve(projectIndex.count());

	for (QModelIndex const& id : projectIndex) {
		Landmark* lm = qobject_cast<Landmark*>(p->getById(p->data(id, Project::IdRole).toInt()));

		if (lm != nullptr) {
			lms.push_back(lm);
		}
	}

	QWidget* w = qobject_cast<QWidget*>(parent);

	if (w != nullptr) {
		w = w->window();
	}

	QString cn = itemClassName();

	QList<QAction*> lst;

	QAction* assignToDistanceConstrain = createAssignToDistanceConstrainAction(parent, p, lms);

	if (assignToDistanceConstrain != nullptr) {
		lst.append(assignToDistanceConstrain);
	}

	QAction* assignToAngleConstrain = createAssignToAngleConstrainAction(parent, p, lms);

	if (assignToAngleConstrain != nullptr) {
		lst.append(assignToAngleConstrain);
	}

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
			connect(toConstrain, &QAction::triggered, [constraint, lms] () {
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
			connect(toConstrain, &QAction::triggered, [constraint, lms] () {
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

} // namespace StereoVisionApp
