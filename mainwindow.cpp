#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include "gui/imagewidget.h"
#include "gui/editor.h"

#include "datablocks/project.h"
#include "datablocks/itemdatamodel.h"

#include <QDebug>
#include <QTabWidget>
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>

namespace StereoVisionApp {

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, ui(new Ui::MainWindow),
	  _activeProject(nullptr)
{
	ui->setupUi(this);

	ui->projectView->setHeaderHidden(true);
	ui->projectView->setContextMenuPolicy(Qt::CustomContextMenu);

	ui->editorPanel->setTabsClosable(true);
	connect(ui->editorPanel, &QTabWidget::tabCloseRequested, this, &MainWindow::closeEditor);

	connect(ui->actionnew_Project, &QAction::triggered, this, &MainWindow::newEmptyProject);
	connect(ui->actionsave_Project, &QAction::triggered, this, &MainWindow::saveProject);
	connect(ui->actionopen_Project, &QAction::triggered, this, &MainWindow::openProject);

	connect(ui->projectView, &QTreeView::customContextMenuRequested, this, &MainWindow::projectContextMenu);
	connect(ui->projectView, &QTreeView::clicked, this, &MainWindow::onProjectSelectionChanged);
}

MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::newEmptyProject() {

	resetProject();

}
void MainWindow::saveProject() {
	saveProjectAs();
}

void MainWindow::saveProjectAs() {

	if (_activeProject == nullptr) {
		return;
	}

	QString filter = "Projects files (*." + Project::PROJECT_FILE_EXT + ")";
	QString folder = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);

	QString fName = QFileDialog::getSaveFileName(this, tr("Save project"), folder, filter);

	if (!fName.isEmpty()) {
		bool status = _activeProject->save(fName);

		if (!status) {
			QMessageBox::warning(this, tr("Error while saving project"), tr("Is the location correct and writtable ?"));
		}
	}

}
void MainWindow::openProject() {

	QString filter = "Projects files (*" + Project::PROJECT_FILE_EXT + ")";
	QString folder = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);


	QString fName = QFileDialog::getOpenFileName(this, tr("Save project"), folder, filter);

	if (!fName.isEmpty()) {
		resetProject();
		bool status = _activeProject->load(fName);

		if (!status) {
			QMessageBox::warning(this, tr("Error while opening project"), tr("Is the location correct and readable ?"));
		}
	}
}

void MainWindow::resetProject() {

	if (_activeProject != nullptr) {
		_activeProject->deleteLater();
	}

	_activeProject = StereoVisionApp::ProjectFactory::defaultProjectFactory().createProject(this);
	ui->projectView->setModel(_activeProject);

	for (Editor* e : _openedEditors) {
		e->setProject(_activeProject);
	}
}

Editor* MainWindow::openEditor(QString editorClassName) {
	Editor* e;

	if (!_openedEditors.contains(editorClassName)) {

		if (!_installedEditors.contains(editorClassName)) {
			return nullptr;
		}

		e = _installedEditors[editorClassName]->factorizeEditor(this);

		if (_activeProject != nullptr) {
			e->setProject(_activeProject);
		}
		_openedEditors[editorClassName] = e;

	} else {
		e = _openedEditors.value(editorClassName);
	}

	int id = ui->editorPanel->indexOf(e);

	if (id < 0) {
		id = ui->editorPanel->count();
		ui->editorPanel->insertTab(id, e, _installedEditors.value(editorClassName)->TypeDescrName());
	}
	ui->editorPanel->setCurrentIndex(id);

	return e;

}
bool MainWindow::isEditorAvailable(QString editorClassName) const {
	return _installedEditors.contains(editorClassName);
}

void MainWindow::installEditor(EditorFactory* factory) {

	if (!_installedEditors.contains(factory->itemClassName())) {
		_installedEditors.insert(factory->itemClassName(), factory);
	}

}

void MainWindow::closeEditor(int index) {
	if (index >= 0 and index < ui->editorPanel->count()) {
		QWidget* w = ui->editorPanel->widget(index);
		Editor* e = qobject_cast<Editor*>(w);

		if (e != nullptr) {
			if (_openedEditors.contains(e->metaObject()->className())) {
				_openedEditors.remove(e->metaObject()->className());
			}
			e->deleteLater();
		}
		ui->editorPanel->removeTab(index);
	}
}

void MainWindow::projectContextMenu(QPoint const& pt) {

	if (_activeProject == nullptr) {
		return;
	}

	QMenu m(ui->projectView);

	QModelIndex id = ui->projectView->indexAt(pt);

	if (!id.isValid()) {
		return;
	}

	QList<QAction*> acts;

	if (id.parent() == QModelIndex()) { //Class item.
		QString dataBlockClass = id.data(Project::ClassRole).toString();
		DataBlockFactory* f = _activeProject->getFactoryForClass(dataBlockClass);

		acts = f->factorizeClassContextActions(ui->projectView, _activeProject);

		for (QAction* a : acts) {
			m.addAction(a);
		}
	} else if (id.parent().parent() == QModelIndex()) { //Datablock item.
		QString dataBlockClass = id.data(Project::ClassRole).toString();
		DataBlockFactory* f = _activeProject->getFactoryForClass(dataBlockClass);

		qint64 item_id = id.data(Project::IdRole).toInt();

		DataBlock* dataBlock = _activeProject->getById(item_id);

		if (dataBlock == nullptr) {
			return;
		}

		acts = f->factorizeItemContextActions(ui->projectView, dataBlock);

		for (QAction* a : acts) {
			m.addAction(a);
		}
	}

	if (acts.count() > 0) {
		m.exec(ui->projectView->mapToGlobal(pt));
	}

	for (QAction* a : acts) {
		a->deleteLater();
	}
}

void MainWindow::onProjectSelectionChanged() {

	QItemSelectionModel* m = ui->projectView->selectionModel();
	QModelIndexList idxs = m->selectedIndexes();

	if (idxs.size() == 1) {

		QVariant v = idxs[0].data(Project::IdRole);

		if (v.isValid()) {
			qint64 id = v.toInt();

			DataBlock* b = _activeProject->getById(id);

			if (b != nullptr) {
				ui->dataBlockView->setModel(b->getDataModel());
				return;
			}
		}

	}

	ui->dataBlockView->setModel(nullptr);

}

} // namespace StereoVisionApp
