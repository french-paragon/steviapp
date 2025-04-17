#include "cameracalibrationactionmanager.h"

#include "datablocks/camera.h"
#include "datablocks/cameracalibration.h"

#include "gui/editor.h"
#include "gui/cameracalibrationeditor.h"

#include "mainwindow.h"

#include "cameracalibrationactions.h"

#include <QAction>

namespace StereoVisionApp {

CameraCalibrationActionManager::CameraCalibrationActionManager(QObject *parent):
	DatablockActionManager(parent)
{

}

QString CameraCalibrationActionManager::ActionManagerClassName() const {
	return "StereoVisionApp::CameraCalibrationActionManager";
}
QString CameraCalibrationActionManager::itemClassName() const {
	return CameraCalibrationFactory::cameraCalibrationClassName();
}

QList<QAction*> CameraCalibrationActionManager::factorizeClassContextActions(QObject* parent, Project* p) const {

    Q_UNUSED(parent);

	QString classname = itemClassName();

	QAction* add = new QAction(tr("Add a new Calibration"), parent);
	connect(add, &QAction::triggered, [classname, p] () {
		qint64 block_id = p->createDataBlock(classname.toStdString().c_str());
		if (block_id > 0) {
			DataBlock* b = p->getById(block_id);
			b->setObjectName(tr("new calibration"));
		}
	});

	return {add};
}

QList<QAction*> CameraCalibrationActionManager::factorizeItemContextActions(QObject* parent, DataBlock* p) const {

	CameraCalibration* calib = qobject_cast<CameraCalibration*>(p);

	if (calib == nullptr) {
		return {};
	}

	QWidget* w = qobject_cast<QWidget*>(parent);

	if (w != nullptr) {
		w = w->window();
	}

	MainWindow* mw = qobject_cast<MainWindow*>(w);

	QList<QAction*> lst;

	if (mw != nullptr) {
        /*QAction* edit = new QAction(tr("Edit"), parent);
		connect(edit, &QAction::triggered, [mw, calib] () {

			Editor* e = mw->openEditor(CameraCalibrationEditor::staticMetaObject.className());
			CameraCalibrationEditor* le = qobject_cast<CameraCalibrationEditor*>(e);

			QObject::connect(le, &CameraCalibrationEditor::optimizeCalibrationTriggered, runCameraCalibration);
			le->setCalibration(calib);

		});
        lst << edit;*/
	}

	QAction* remove = new QAction(tr("Remove"), parent);
	connect(remove, &QAction::triggered, [calib] () {
		Project* p = calib->getProject();

		if (p != nullptr) {
			p->clearById(calib->internalId());
		}
	});
	lst.append(remove);

	return lst;
}

} // namespace StereoVisionApp
