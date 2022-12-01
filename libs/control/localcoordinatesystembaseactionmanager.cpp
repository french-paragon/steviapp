#include "localcoordinatesystembaseactionmanager.h"

#include <QWidget>
#include <QAction>

#include "mainwindow.h"
#include "datablocks/localcoordinatesystem.h"

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

	QAction* clearOptimized = new QAction(tr("Clear optimized"), parent);
	connect(clearOptimized, &QAction::triggered, [lcss] () {
		for (LocalCoordinateSystem* lcs : lcss) {
			lcs->clearOptimized();
		}
	});
	lst.append(clearOptimized);

	return lst;

}

} // namespace StereoVisionApp
