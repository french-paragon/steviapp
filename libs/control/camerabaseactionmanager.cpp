#include "camerabaseactionmanager.h"

#include "datablocks/camera.h"
#include "datablocks/cameras/pushbroompinholecamera.h"

#include "mainwindow.h"
#include "gui/lenseditor.h"

#include <QAction>
#include <QFileDialog>
#include <QJsonDocument>

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

    QString classname = itemClassName();

    QAction* add = new QAction(tr("Add a new Camera"), parent);
    connect(add, &QAction::triggered, [classname, p] () {
        qint64 block_id = p->createDataBlock(classname.toStdString().c_str());
        if (block_id > 0) {
            DataBlock* b = p->getById(block_id);
            b->setObjectName(tr("new camera"));
        }
    });

    QAction* addPushBroom = new QAction(tr("Add a new Push Broom Camera"), parent);
    connect(addPushBroom, &QAction::triggered, [p] () {
        QString classname = PushBroomPinholeCamera::staticMetaObject.className();
        qint64 block_id = p->createDataBlock(classname.toStdString().c_str());
        if (block_id > 0) {
            DataBlock* b = p->getById(block_id);
            b->setObjectName(tr("new push broom camera"));
        }
    });

	QAction* import = new QAction(tr("Import a Camera"), parent);
	connect(import, &QAction::triggered, [classname, p] () {

		MainWindow* mw = MainWindow::getActiveMainWindow();

		if (mw == nullptr) {
			return;
		}

		QString cameraFile = QFileDialog::getOpenFileName(mw, tr("Import a camera"), QString(), tr("Steviap camera files (*.stevcam)"));

		if (cameraFile.isEmpty()) {
			return;
		}

		QFile infile(cameraFile);
		bool opened = infile.open(QIODevice::ReadOnly | QIODevice::Text);

		if (!opened) {
			return;
		}

		QByteArray data = infile.readAll();
		infile.close();

		QJsonDocument doc = QJsonDocument::fromJson(data);
		QJsonObject camRep = doc.object();

		qint64 block_id = p->createDataBlock(classname.toStdString().c_str());

		if (block_id >= 0) {
			Camera* cam = p->getDataBlock<Camera>(block_id);
			if (cam != nullptr) {
				cam->setObjectName(tr("imported camera"));
				cam->setParametersFromJsonRepresentation(camRep);

				if (camRep.contains("exportedCameraName")) {
					QJsonValue exportedName = camRep.value("exportedCameraName");

					if (exportedName.isString()) {
						cam->setObjectName(tr("imported camera (%1)").arg(exportedName.toString()));
					}
				}
			}
		}
	});

    return {add, import, addPushBroom};
}
QList<QAction*> CameraBaseActionManager::factorizeItemContextActions(QObject* parent, DataBlock* item) const {

    if (item == nullptr) {
        return {};
    }

    Camera* cam = qobject_cast<Camera*>(item);

	QWidget* w = qobject_cast<QWidget*>(parent);

	if (w != nullptr) {
		w = w->window();
	}

	MainWindow* mw = qobject_cast<MainWindow*>(w);

	QList<QAction*> lst;

	if (mw != nullptr) {
        if (cam != nullptr) {
            QAction* edit = new QAction(tr("Edit"), parent);
            connect(edit, &QAction::triggered, [mw, cam] () {

                Editor* e = mw->openEditor(LensEditor::LensEditorClassName);
                LensEditor* le = qobject_cast<LensEditor*>(e);

                le->setCamera(cam);

            });
            lst << edit;
        }
	}

    if (cam != nullptr) {
        QAction* exportCam = new QAction(tr("Export"), parent);
        connect(exportCam, &QAction::triggered, [cam] () {

            MainWindow* mw = MainWindow::getActiveMainWindow();

            if (mw == nullptr) {
                return;
            }

            QString cameraFile = QFileDialog::getSaveFileName(mw, tr("Export the camera"), QString(), tr("Steviap camera files (*.stevcam)"));

            if (cameraFile.isEmpty()) {
                return;
            }

            if (!cameraFile.endsWith(".stevcam")) {
                cameraFile += ".stevcam";
            }

            QJsonObject camRep = cam->getJsonRepresentation();

            camRep.insert("exportedCameraName", cam->objectName());

            QJsonDocument doc(camRep);

            QByteArray data = doc.toJson();

            QFile outfile(cameraFile);
            bool opened = outfile.open(QIODevice::WriteOnly | QIODevice::Text);

            if (!opened) {
                return;
            }

            outfile.write(data);
            outfile.close();


        });
        lst.append(exportCam);

        QAction* importCam = new QAction(tr("Import parameters"), parent);
        connect(importCam, &QAction::triggered, [cam] () {

            if (cam == nullptr) {
                return ;
            }

            MainWindow* mw = MainWindow::getActiveMainWindow();

            if (mw == nullptr) {
                return;
            }

            QString cameraFile = QFileDialog::getOpenFileName(mw, tr("Import a camera"), QString(), tr("Steviap camera files (*.stevcam)"));

            if (cameraFile.isEmpty()) {
                return;
            }

            QFile infile(cameraFile);
            bool opened = infile.open(QIODevice::ReadOnly | QIODevice::Text);

            if (!opened) {
                return;
            }

            QByteArray data = infile.readAll();
            infile.close();

            QJsonDocument doc = QJsonDocument::fromJson(data);
            QJsonObject camRep = doc.object();

            cam->setParametersFromJsonRepresentation(camRep);

        });
        lst.append(importCam);
    }

    QAction* clearOptimized = new QAction(tr("Clear optimized"), parent);
    connect(clearOptimized, &QAction::triggered, [item] () {
        item->clearOptimized();
    });
    lst.append(clearOptimized);

	QAction* remove = new QAction(tr("Remove"), parent);
    connect(remove, &QAction::triggered, [item] () {
        Project* p = item->getProject();

		if (p != nullptr) {
            p->clearById(item->internalId());
		}
	});
	lst.append(remove);

	return lst;

}

} // namespace StereoVisionApp
