#include "angleconstrainactionmanager.h"

#include "datablocks/angleconstrain.h"

#include <QAction>

namespace StereoVisionApp {

AngleConstrainActionManager::AngleConstrainActionManager(QObject* parent) :
	DatablockActionManager(parent)
{

}

QString AngleConstrainActionManager::ActionManagerClassName() const {
	return AngleConstrainActionManager::staticMetaObject.className();
}
QString AngleConstrainActionManager::itemClassName() const {
	return AngleConstrain::staticMetaObject.className();
}

QList<QAction*> AngleConstrainActionManager::factorizeItemContextActions(QObject* parent, DataBlock* p) const {


	AngleConstrain* ac = qobject_cast<AngleConstrain*>(p);

	if (ac == nullptr) {
		return QList<QAction*>();
	}

	QList<QAction*> lst;

	QAction* remove = new QAction(tr("Remove"), parent);
	connect(remove, &QAction::triggered, [ac] () {
		Project* p = ac->getProject();

		if (p != nullptr) {
			p->clearById(ac->internalId());
		}
	});
	lst.append(remove);

	return lst;

}

} // namespace StereoVisionApp
