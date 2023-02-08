#include "stereorigactionmanager.h"

#include "datablocks/stereorig.h"
#include "datablocks/image.h"

#include "mainwindow.h"

#include "stereorigactions.h"

#include <QWidget>
#include <QFileDialog>
#include <QAction>
#include <QMenu>

#include <QFile>
#include <QJsonDocument>

namespace StereoVisionApp {

StereoRigActionManager::StereoRigActionManager(QObject* parent) :
	DatablockActionManager(parent)
{

}

QList<QAction*> StereoRigActionManager::factorizeClassContextActions(QObject* parent, Project* p) const {

	QList<QAction*> actions = DatablockActionManager::factorizeClassContextActions(parent, p);

	QString classname = itemClassName();

	QAction* import = new QAction(tr("Import a Stereo Rig"), parent);
	connect(import, &QAction::triggered, [classname, p] () {

		MainWindow* mw = MainWindow::getActiveMainWindow();

		if (mw == nullptr) {
			return;
		}

		QString rigFile = QFileDialog::getOpenFileName(mw, tr("Import a Stereo Rig"), QString(), tr("Steviap rig files (*.stevrig)"));

		if (rigFile.isEmpty()) {
			return;
		}

		QFile infile(rigFile);
		bool opened = infile.open(QIODevice::ReadOnly | QIODevice::Text);

		if (!opened) {
			return;
		}

		QByteArray data = infile.readAll();
		infile.close();

		QJsonDocument doc = QJsonDocument::fromJson(data);
		QJsonObject rigRep = doc.object();

		qint64 block_id = p->createDataBlock(classname.toStdString().c_str());

		if (block_id >= 0) {
			StereoRig* rig = p->getDataBlock<StereoRig>(block_id);
			if (rig != nullptr) {
				rig->setObjectName(tr("imported stereo rig"));
				rig->setParametersFromJsonRepresentation(rigRep);

				if (rigRep.contains("exportedStereoRigName")) {
					QJsonValue exportedName = rigRep.value("exportedStereoRigName");

					if (exportedName.isString()) {
						rig->setObjectName(tr("imported stereo rig (%1)").arg(exportedName.toString()));
					}
				}
			}
		}
	});

	actions.push_back(import);

	return actions;

}

QList<QAction*> StereoRigActionManager::factorizeItemContextActions(QObject* parent, DataBlock* item) const {

	QList<QAction*> actions = DatablockActionManager::factorizeItemContextActions(parent, item);

	StereoRig* rig = qobject_cast<StereoRig*>(item);

	if (rig == nullptr) {
		return actions;
	}

	QAction* exportRig = new QAction(tr("Export"), parent);
	connect(exportRig, &QAction::triggered, [rig] () {

		MainWindow* mw = MainWindow::getActiveMainWindow();

		if (mw == nullptr) {
			return;
		}

		QString rigFile = QFileDialog::getSaveFileName(mw, tr("Export the Stereo Rig"), QString(), tr("Steviap Stereo Rig files (*.stevrig)"));

		if (rigFile.isEmpty()) {
			return;
		}

		if (!rigFile.endsWith(".stevrig")) {
			rigFile += ".stevrig";
		}

		QJsonObject rigRep = rig->getJsonRepresentation();

		rigRep.insert("exportedStereoRigName", rig->objectName());

		QJsonDocument doc(rigRep);

		QByteArray data = doc.toJson();

		QFile outfile(rigFile);
		bool opened = outfile.open(QIODevice::WriteOnly | QIODevice::Text);

		if (!opened) {
			return;
		}

		outfile.write(data);
		outfile.close();


	});
	actions.append(exportRig);

	QAction* importRig = new QAction(tr("Import parameters"), parent);
	connect(importRig, &QAction::triggered, [rig] () {

		if (rig == nullptr) {
			return;
		}

		MainWindow* mw = MainWindow::getActiveMainWindow();

		if (mw == nullptr) {
			return;
		}

		QString rigFile = QFileDialog::getOpenFileName(mw, tr("Import a Stereo Rig"), QString(), tr("Steviap rig files (*.stevrig)"));

		if (rigFile.isEmpty()) {
			return;
		}

		QFile infile(rigFile);
		bool opened = infile.open(QIODevice::ReadOnly | QIODevice::Text);

		if (!opened) {
			return;
		}

		QByteArray data = infile.readAll();
		infile.close();

		QJsonDocument doc = QJsonDocument::fromJson(data);
		QJsonObject rigRep = doc.object();

		rig->setParametersFromJsonRepresentation(rigRep);
	});

	actions.push_back(importRig);

	QAction* alignImage = createAlignImageActions(parent, rig->getProject(), rig);

	if (alignImage != nullptr) {
		actions.push_back(alignImage);
	}

	QAction* exportRectifiedImages = createExportImageActions(parent, rig->getProject(), rig);

	if (alignImage != nullptr) {
		actions.push_back(exportRectifiedImages);
	}

	QAction* remove = new QAction(tr("Remove"), parent);
	connect(remove, &QAction::triggered, [rig] () {
		Project* p = rig->getProject();

		if (p != nullptr) {
			p->clearById(rig->internalId());
		}
	});
	actions.append(remove);

	return actions;

}

QString StereoRigActionManager::ActionManagerClassName() const {
	return "StereoVisionApp::StereoRigActionManager";
}
QString StereoRigActionManager::itemClassName() const {
	return StereoRigFactory::StereoRigClassName();
}


QAction* StereoRigActionManager::createAlignImageActions(QObject* parent, Project* p, StereoRig* rig) const {


	bool ok = true;
	int countActions = 0;

	if (rig == nullptr) {
		return nullptr;
	}

	qint64 rig_id = rig->internalId();

	QAction* alignImage = new QAction(tr("Align image for pair"), parent);

	QMenu* pairsMenu = new QMenu();
	connect(alignImage, &QObject::destroyed, pairsMenu, &QObject::deleteLater);

	QVector<qint64> pairsidxs = rig->listTypedSubDataBlocks(ImagePair::staticMetaObject.className());

	for (qint64 id : pairsidxs) {

		ImagePair* pair = rig->getImagePair(id);

		if (pair == nullptr) {
			continue;
		}

		Image* img1 = p->getDataBlock<Image>(pair->idImgCam1());
		Image* img2 = p->getDataBlock<Image>(pair->idImgCam2());

		if (img1 == nullptr or img2 == nullptr) {
			continue;
		}

		qint64 img_1_id = img1->internalId();
		qint64 img_2_id = img2->internalId();

		QAction* forPair = pairsMenu->addAction(QString("Rig %1 - %2").arg(pair->idImgCam1()).arg(pair->idImgCam2()));
		QMenu* imgsMenu = new QMenu();
		connect(forPair, &QObject::destroyed, imgsMenu, &QObject::deleteLater);

		QAction* img1Action = imgsMenu->addAction(img1->objectName());

		connect(img1Action, &QAction::triggered, p, [p, rig_id, img_1_id, img_2_id] () {
			alignImagesInRig(p, rig_id, img_2_id, img_1_id);
		});

		QAction* img2Action = imgsMenu->addAction(img2->objectName());

		connect(img2Action, &QAction::triggered, p, [p, rig_id, img_1_id, img_2_id] () {
			alignImagesInRig(p, rig_id, img_1_id, img_2_id);
		});

		countActions++;

		forPair->setMenu(imgsMenu);

	}

	alignImage->setMenu(pairsMenu);

	if (countActions <= 0) {
		alignImage->deleteLater();
		return nullptr;
	}

	return alignImage;

}


QAction* StereoRigActionManager::createExportImageActions(QObject* parent, Project* p, StereoRig* rig) const {


	int countActions = 0;


	qint64 rig_id = rig->internalId();

	QAction* exportRectifiedImagesAction = new QAction(tr("Export rectified images for pair"), parent);

	QMenu* pairsMenu = new QMenu();
	connect(exportRectifiedImagesAction, &QObject::destroyed, pairsMenu, &QObject::deleteLater);

	QVector<qint64> pairsidxs = rig->listTypedSubDataBlocks(ImagePair::staticMetaObject.className());

	for (qint64 id : pairsidxs) {

		ImagePair* pair = rig->getImagePair(id);

		if (pair == nullptr) {
			continue;
		}

		QAction* forPair = pairsMenu->addAction(QString("Rig %1 - %2").arg(pair->idImgCam1()).arg(pair->idImgCam2()));

		connect(forPair, &QAction::triggered, p, [p, rig_id, id] () {
			exportRectifiedImages(p, rig_id, id);
		});

		countActions++;

	}

	exportRectifiedImagesAction->setMenu(pairsMenu);

	if (countActions <= 0) {
		exportRectifiedImagesAction->deleteLater();
		return nullptr;
	}

	return exportRectifiedImagesAction;
}

} // namespace StereoVisionApp
