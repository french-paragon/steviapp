#include "stereorigactionmanager.h"

#include "datablocks/stereorig.h"

#include "mainwindow.h"

#include <QWidget>
#include <QFileDialog>
#include <QAction>

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

} // namespace StereoVisionApp
