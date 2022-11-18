#include "distanceconstrainactionmanager.h"

#include "datablocks/distanceconstrain.h"

#include <QAction>

namespace StereoVisionApp {

DistanceConstrainActionManager::DistanceConstrainActionManager(QObject *parent) :
	DatablockActionManager(parent)
{

}

QString DistanceConstrainActionManager::ActionManagerClassName() const {
	return DistanceConstrainActionManager::staticMetaObject.className();
}
QString DistanceConstrainActionManager::itemClassName() const {
	return DistanceConstrain::staticMetaObject.className();
}

QList<QAction*> DistanceConstrainActionManager::factorizeItemContextActions(QObject* parent, DataBlock* p) const {

	DistanceConstrain* dc = qobject_cast<DistanceConstrain*>(p);

	if (dc == nullptr) {
		return QList<QAction*>();
	}

	QList<QAction*> lst;

	QAction* remove = new QAction(tr("Remove"), parent);
	connect(remove, &QAction::triggered, [dc] () {
		Project* p = dc->getProject();

		if (p != nullptr) {
			p->clearById(dc->internalId());
		}
	});
	lst.append(remove);

	return lst;

}

} // namespace StereoVisionApp
