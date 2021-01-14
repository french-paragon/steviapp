#include "imagebaseactionmanager.h"

#include "mainwindow.h"
#include "gui/imageeditor.h"

#include "datablocks/image.h"
#include "datablocks/camera.h"

#include <QAction>
#include <QStandardPaths>
#include <QFileDialog>
#include <QMenu>
#include <QMessageBox>

#include "imagebaseactions.h"

namespace StereoVisionApp {

ImageBaseActionManager::ImageBaseActionManager(QObject *parent) :
	DatablockActionManager(parent)
{

}

QString ImageBaseActionManager::ActionManagerClassName() const {
	return "StereoVisionApp::ImageBaseActionManager";
}
QString ImageBaseActionManager::itemClassName() const {
	return ImageFactory::imageClassName();
}

QList<QAction*> ImageBaseActionManager::factorizeClassContextActions(QObject* parent, Project* p) const {

	QWidget* w = qobject_cast<QWidget*>(parent);

	if (w != nullptr) {
		w = w->window();
	}

	QString cn = itemClassName();

	QAction* add = new QAction(tr("Add Images"), parent);
	connect(add, &QAction::triggered, [w, p] () {
		QString filter = "Images(*.jpg *.jpeg *.png *.bmp *.tiff *.tif)";
		QString folder = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
		QStringList images = QFileDialog::getOpenFileNames(w, tr("Open Images"), folder, filter);

		QStringList failedImgs = addImages(images, p);

		if (!failedImgs.isEmpty()) {
			QString errorList = failedImgs.join("\n");
			QMessageBox::warning(w, tr("Failed to load some images"), errorList);
		}
	});

	QAction* exportRectified = new QAction(tr("Export rectified images"), parent);
	connect(exportRectified, &QAction::triggered, [w, p, cn] () {
		QList<qint64> imageIds = p->getIdsByClass(cn).toList();
		exportRectifiedImages(imageIds, p, w);
	});

	if (p->getIdsByClass(cn).size() == 0) {
		exportRectified->setEnabled(false);
	}

	return {add, exportRectified};

}
QList<QAction*> ImageBaseActionManager::factorizeItemContextActions(QObject* parent, DataBlock* item) const {

	Image* im = qobject_cast<Image*>(item);

	if (im == nullptr) {
		return {};
	}

	QWidget* w = qobject_cast<QWidget*>(parent);

	if (w != nullptr) {
		w = w->window();
	}

	MainWindow* mw = qobject_cast<MainWindow*>(w);

	QString cn = itemClassName();

	QList<QAction*> lst;

	if (mw != nullptr) {
		QAction* edit = new QAction(tr("Edit"), parent);
		connect(edit, &QAction::triggered, [mw, im] () {

			Editor* e = mw->openEditor(ImageEditor::ImageEditorClassName);
			ImageEditor* ie = qobject_cast<ImageEditor*>(e);

			ie->setImage(im);

		});
		lst << edit;
	}

	QAction* remove = new QAction(tr("Remove"), parent);
	connect(remove, &QAction::triggered, [im] () {
		Project* p = im->getProject();

		if (p != nullptr) {
			p->clearById(im->internalId());
		}
	});
	lst.append(remove);

	QAction* assignToCamera = createAssignToCameraAction(parent, im->getProject(), {im});

	lst.append(assignToCamera);

	QAction* exportRectified = new QAction(tr("Export rectified image"), parent);
	connect(exportRectified, &QAction::triggered, [mw, im] () {
		exportRectifiedImages({im->internalId()}, im->getProject(), mw);
	});
	lst.append(exportRectified);

	return lst;

}
QList<QAction*> ImageBaseActionManager::factorizeMultiItemsContextActions(QObject* parent, Project* p, QModelIndexList const& projectIndex) const {

	QVector<Image*> ims;
	ims.reserve(projectIndex.count());

	for (QModelIndex const& id : projectIndex) {
		Image* im = qobject_cast<Image*>(p->getById(p->data(id, Project::IdRole).toInt()));

		if (im != nullptr) {
			ims.push_back(im);
		}
	}

	QWidget* w = qobject_cast<QWidget*>(parent);

	if (w != nullptr) {
		w = w->window();
	}

	QString cn = itemClassName();

	QList<QAction*> lst;

	QAction* assignToCamera = createAssignToCameraAction(parent, p, ims);

	lst.append(assignToCamera);


	QAction* exportRectified = new QAction(tr("Export rectified images"), parent);
	connect(exportRectified, &QAction::triggered, [w, p, ims] () {
		QList<qint64> imageIds;
		imageIds.reserve(ims.size());

		for (Image* im : ims) {
			imageIds.push_back(im->internalId());
		}

		exportRectifiedImages(imageIds, p, w);
	});

	lst.append(exportRectified);

	return lst;

}

QAction* ImageBaseActionManager::createAssignToCameraAction(QObject* parent, Project* p, QVector<Image*> const& ims) const {

	QAction* assignToCamera = new QAction(tr("Assign to camera"), parent);
	QMenu* camMenu = new QMenu();
	connect(assignToCamera, &QObject::destroyed, camMenu, &QObject::deleteLater);

	QVector<qint64> camsIds = p->getIdsByClass(CameraFactory::cameraClassName());

	for(qint64 camId : camsIds) {

		Camera* c = qobject_cast<Camera*>(p->getById(camId));

		if (c != nullptr) {
			QAction* toCam = new QAction(c->objectName(), assignToCamera);
			connect(toCam, &QAction::triggered, [camId, ims] () {
				for (Image* im : ims) {
					im->assignCamera(camId);
				}
			});
			camMenu->addAction(toCam);
		}
	}
	assignToCamera->setMenu(camMenu);

	return assignToCamera;

}

} // namespace StereoVisionApp
