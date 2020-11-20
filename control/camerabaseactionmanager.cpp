#include "camerabaseactionmanager.h"

#include "datablocks/camera.h"

#include "mainwindow.h"
#include "gui/lenseditor.h"

#include <QAction>

namespace StereoVisionApp {

CameraBaseActionManager::CameraBaseActionManager(QObject *parent) :
	DatablockActionManager(parent)
{

}


QString CameraBaseActionManager::ActionManagerClassName() const {
	return "StereoVisionApp::CameraBaseActionManager";
}
QString CameraBaseActionManager::itemClassName() const {
	return CameraFactory::cameraClassName();
}

QList<QAction*> CameraBaseActionManager::factorizeClassContextActions(QObject* parent, Project* p) const {
	Q_UNUSED(parent);
	Q_UNUSED(p);
	return {};
}
QList<QAction*> CameraBaseActionManager::factorizeItemContextActions(QObject* parent, DataBlock* item) const {

	Camera* cam = qobject_cast<Camera*>(item);

	if (cam == nullptr) {
		return {};
	}

	QWidget* w = qobject_cast<QWidget*>(parent);

	if (w != nullptr) {
		w = w->window();
	}

	MainWindow* mw = qobject_cast<MainWindow*>(w);

	QList<QAction*> lst;

	if (mw != nullptr) {
		QAction* edit = new QAction(tr("Edit"), parent);
		connect(edit, &QAction::triggered, [mw, cam] () {

			Editor* e = mw->openEditor(LensEditor::LensEditorClassName);
			LensEditor* le = qobject_cast<LensEditor*>(e);

			le->setCamera(cam);

		});
		lst << edit;
	}

	QAction* remove = new QAction(tr("Remove"), parent);
	connect(remove, &QAction::triggered, [cam] () {
		Project* p = cam->getProject();

		if (p != nullptr) {
			p->clearById(cam->internalId());
		}
	});
	lst.append(remove);

	return lst;

}

} // namespace StereoVisionApp
