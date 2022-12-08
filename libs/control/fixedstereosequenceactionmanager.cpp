#include "fixedstereosequenceactionmanager.h"

#include "datablocks/fixedcolorstereosequence.h"
#include "datablocks/fixedstereopluscolorsequence.h"

#include "gui/fixedstereosequenceeditor.h"

#include "mainwindow.h"

#include "fixedstereosequenceactions.h"

#include <QAction>

namespace StereoVisionApp {

FixedStereoPlusColorSequenceActionManager::FixedStereoPlusColorSequenceActionManager(QObject* parent) :
	DatablockActionManager(parent)
{

}

QString FixedStereoPlusColorSequenceActionManager::ActionManagerClassName() const {
	return FixedStereoPlusColorSequenceActionManager::staticMetaObject.className();
}
QString FixedStereoPlusColorSequenceActionManager::itemClassName() const {
	return FixedStereoPlusColorSequence::staticMetaObject.className();
}

QList<QAction*> FixedStereoPlusColorSequenceActionManager::factorizeItemContextActions(QObject* parent, DataBlock* p) const {

	FixedStereoPlusColorSequence* seq = qobject_cast<FixedStereoPlusColorSequence*>(p);

	if (seq == nullptr) {
		return {};
	}

	MainWindow* mw = MainWindow::getActiveMainWindow();

	QList<QAction*> lst;

	if (mw != nullptr) {
		QAction* edit = new QAction(tr("Edit"), parent);
		connect(edit, &QAction::triggered, [mw, seq] () {

			Editor* e = mw->openEditor(FixedStereoSequenceEditor::staticMetaObject.className());
			FixedStereoSequenceEditor* se = qobject_cast<FixedStereoSequenceEditor*>(e);

			QObject::connect(se, &FixedStereoSequenceEditor::rgbImagesExportTriggered, exportColoredStereoImagesPointCloud);
			QObject::connect(se, &FixedStereoSequenceEditor::imagesWithRGBExportTriggered, exportStereoImagesPlusColorPointCloud);

			se->setSequence(seq);

		});
		lst << edit;
	}

	return lst;

}

FixedColorStereoSequenceActionManager::FixedColorStereoSequenceActionManager(QObject* parent) :
	DatablockActionManager(parent)
{

}

QString FixedColorStereoSequenceActionManager::ActionManagerClassName() const {
	return FixedStereoPlusColorSequenceActionManager::staticMetaObject.className();
}
QString FixedColorStereoSequenceActionManager::itemClassName() const {
	return FixedColorStereoSequence::staticMetaObject.className();
}

QList<QAction*> FixedColorStereoSequenceActionManager::factorizeItemContextActions(QObject* parent, DataBlock* p) const {

	FixedColorStereoSequence* seq = qobject_cast<FixedColorStereoSequence*>(p);

	if (seq == nullptr) {
		return {};
	}

	MainWindow* mw = MainWindow::getActiveMainWindow();

	QList<QAction*> lst;

	if (mw != nullptr) {
		QAction* edit = new QAction(tr("Edit"), parent);
		connect(edit, &QAction::triggered, [mw, seq] () {

			Editor* e = mw->openEditor(FixedStereoSequenceEditor::staticMetaObject.className());
			FixedStereoSequenceEditor* se = qobject_cast<FixedStereoSequenceEditor*>(e);

			se->setSequence(seq);

		});
		lst << edit;
	}

	return lst;

}

} // namespace StereoVisionApp
