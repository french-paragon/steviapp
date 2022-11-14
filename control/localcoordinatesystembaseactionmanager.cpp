#include "localcoordinatesystembaseactionmanager.h"

#include <QWidget>
#include <QAction>

#include "mainwindow.h"
#include "datablocks/localcoordinatesystem.h"

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

	LocalCoordinateSystem* lm = qobject_cast<LocalCoordinateSystem*>(p);

	if (lm == nullptr) {
		return {};
	}

	QWidget* w = qobject_cast<QWidget*>(parent);

	if (w != nullptr) {
		w = w->window();
	}

	MainWindow* mw = qobject_cast<MainWindow*>(w);

	QList<QAction*> lst;

	QAction* clearOptimized = new QAction(tr("Clear optimized"), parent);
	connect(clearOptimized, &QAction::triggered, [lm] () {
		lm->clearOptimized();
	});
	lst.append(clearOptimized);

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

	QAction* clearOptimized = new QAction(tr("Clear optimized"), parent);
	connect(clearOptimized, &QAction::triggered, [lcss] () {
		for (LocalCoordinateSystem* lm : lcss) {
			lm->clearOptimized();
		}
	});
	lst.append(clearOptimized);

	return lst;

}

} // namespace StereoVisionApp
