#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include "control/actionmanager.h"
#include "control/solversactions.h"
#include "control/exportactions.h"

#include "gui/imagewidget.h"
#include "gui/editor.h"
#include "gui/sparsealignementeditor.h"

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
	ui->dataBlockView->setContextMenuPolicy(Qt::CustomContextMenu);

	ui->editorPanel->setTabsClosable(true);
	connect(ui->editorPanel, &QTabWidget::tabCloseRequested, this, &MainWindow::closeEditor);

	connect(ui->actionnew_Project, &QAction::triggered, this, &MainWindow::newEmptyProject);
	connect(ui->actionsave_Project, &QAction::triggered, this, &MainWindow::saveProject);
	connect(ui->actionsave_project_as, &QAction::triggered, this, &MainWindow::saveProjectAs);
	connect(ui->actionopen_Project, &QAction::triggered, this, static_cast<void(MainWindow::*)()>(&MainWindow::openProject));
	connect(ui->actionexport_optimized_to_collada, &QAction::triggered, [this] () { exportCollada(_activeProject, this); });

	connect(ui->projectView, &QTreeView::customContextMenuRequested, this, &MainWindow::projectContextMenu);
	connect(ui->projectView, &QTreeView::clicked, this, &MainWindow::onProjectSelectionChanged);

	connect(ui->dataBlockView, &QTreeView::customContextMenuRequested, this, &MainWindow::datablockContextMenu);

	connect(ui->actionclear_solution, &QAction::triggered, this, &MainWindow::clearOptimSolution);
	connect(ui->actionsolve_coarse, &QAction::triggered, this, &MainWindow::runCoarseOptim);
	connect(ui->actionsolve_sparse, &QAction::triggered, this, &MainWindow::runSparseOptim);
	connect(ui->actionsolve_stereo_rig, &QAction::triggered, [this] () {solveSparseStereoRig(_activeProject, this); });
	connect(ui->actionOpenSparseAlignEditor, &QAction::triggered, [this] () {openSparseViewer(); });
	connect(ui->actionFindInitialSolution, &QAction::triggered, [this] () {if (_activeProject != nullptr) {initSolution(_activeProject, this); } });
	connect(ui->actionInit_stereo_rig, &QAction::triggered, [this] () {if (_activeProject != nullptr) {initMonoStereoRigSolution(_activeProject, this); } });
	connect(ui->actionopen_Sparse_solution_editor, &QAction::triggered, this, [this] () {openSparseViewer(); });

}

MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::newEmptyProject() {

	resetProject();

}
void MainWindow::saveProject() {

	if (_activeProject->source().isEmpty()) {
		saveProjectAs();
		return;
	}

	bool status = _activeProject->save();

	if (!status) {
		QMessageBox::warning(this, tr("Error while saving project"), tr("Is the location correct and writtable ?"));
	}
}

void MainWindow::saveProjectAs() {

	if (_activeProject == nullptr) {
		return;
	}

	QString filter = "Projects files (*" + Project::PROJECT_FILE_EXT + ")";
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

	openProject(fName);
}

void MainWindow::openProject(QString const& fName) {

	if (!fName.isEmpty()) {
		resetProject();
		bool status = _activeProject->load(fName);

		if (!status) {
			QMessageBox::warning(this, tr("Error while opening project"), tr("Is the location correct and readable ?"));
		}
	}
}

void MainWindow::displayInfoMessage(QString msg) {
	ui->statusbar->showMessage(msg);
}

void MainWindow::resetProject() {

	if (_activeProject != nullptr) {
		_activeProject->deleteLater();
	}

	_activeProject = StereoVisionApp::ProjectFactory::defaultProjectFactory().createProject(this);
	ui->projectView->setModel(_activeProject);

	configureProjectWindowsMenu();
	ui->menusolve->setEnabled(true);

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

		// pass messages
		connect(e, &StereoVisionApp::Editor::sendStatusMessage,
				this, &MainWindow::displayInfoMessage);

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
			disconnect(e, &StereoVisionApp::Editor::sendStatusMessage,
					   this, &MainWindow::displayInfoMessage);
			e->deleteLater();
		}
		ui->editorPanel->removeTab(index);
	}
}

void MainWindow::openSparseViewer() {

	Editor* e = openEditor(SparseAlignementEditor::SparseAlignementEditorClassName);
	SparseAlignementEditor* sae = qobject_cast<SparseAlignementEditor*>(e);
	if (sae != nullptr) {
		sae->reloadLandmarks();
	}
}

void MainWindow::configureProjectWindowsMenu() {

	ui->menuproject->clear();
	ui->menuproject->setEnabled(false);

	if (_activeProject == nullptr) {
		return;
	}

	QVector<QString> datablocksTypes = StereoVisionApp::ProjectFactory::defaultProjectFactory().installedTypes();
	QVector<QString> descrs;
	QVector<int> sortingKey;
	descrs.reserve(datablocksTypes.size());
	sortingKey.reserve(datablocksTypes.size());

	int i = 0;
	for (QString t : datablocksTypes) {
		descrs.push_back(StereoVisionApp::ProjectFactory::defaultProjectFactory().typeDescr(t));
		sortingKey.push_back(i++);
	}

	std::sort(sortingKey.begin(), sortingKey.end(), [descrs] (int f, int s) { return descrs[f] < descrs[s]; });

	for (int index : sortingKey) {

		QMenu* m = ui->menuproject->addMenu(descrs[index]);

		QList<QAction*> acts = ActionManagersLibrary::defaultActionManagersLibrary().factorizeDatablockClassContextActions(datablocksTypes[index], m, _activeProject);

		if (acts.size() > 0) {
			m->addActions(acts);
		} else {
			ui->menuproject->removeAction(m->menuAction());
			m->deleteLater();
		}
	}

	ui->menuproject->setEnabled(true);

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

	QModelIndexList s = ui->projectView->selectionModel()->selectedRows();
	bool singleClassMultiSelection = false;
	QString mSelClass;
	if (s.count() > 1 and s.contains(id)) {
		singleClassMultiSelection = true;

		QModelIndex first = s.first();

		mSelClass = _activeProject->data(first, Project::ClassRole).toString();

		for (int i = 1; i < s.count(); i++) {
			if (_activeProject->data(s[i], Project::ClassRole).toString() != mSelClass) {
				singleClassMultiSelection = false;
				break;
			}
		}
	}

	if (singleClassMultiSelection) {

		acts = ActionManagersLibrary::defaultActionManagersLibrary().factorizeDatablockMultiItemsContextActions(mSelClass, ui->projectView, _activeProject, s);

		for (QAction* a : acts) {
			m.addAction(a);
		}

	} else if (id.parent() == QModelIndex()) { //Class item.
		QString dataBlockClass = id.data(Project::ClassRole).toString();

		acts = ActionManagersLibrary::defaultActionManagersLibrary().factorizeDatablockClassContextActions(dataBlockClass, ui->projectView, _activeProject);

		for (QAction* a : acts) {
			m.addAction(a);
		}
	} else if (id.parent().parent() == QModelIndex()) { //Datablock item.
		QString dataBlockClass = id.data(Project::ClassRole).toString();

		qint64 item_id = id.data(Project::IdRole).toInt();

		DataBlock* dataBlock = _activeProject->getById(item_id);

		if (dataBlock == nullptr) {
			return;
		}

		acts = ActionManagersLibrary::defaultActionManagersLibrary().factorizeDatablockItemContextActions(dataBlockClass, ui->projectView, dataBlock);

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
				ui->dataBlockView->expandToDepth(1);
				return;
			}
		}

	}

	ui->dataBlockView->setModel(nullptr);

}

void MainWindow::datablockContextMenu(QPoint const& pt) {

	if (_activeProject == nullptr) {
		return;
	}

	QMenu m(ui->dataBlockView);

	QModelIndex id = ui->dataBlockView->indexAt(pt);
	ItemDataModel* model = qobject_cast<ItemDataModel*>(ui->dataBlockView->model());

	if (!id.isValid() or model == nullptr or ui->dataBlockView->model() != id.model()) {
		return;
	}

	QPersistentModelIndex perId(id);

	QList<QAction*> acts;

	if (id.data(ItemDataModel::HasDeleterRole).toBool()) {
		QAction* del = new QAction("delete");
		connect(del, &QAction::triggered, [perId, model] () {
			if (perId.isValid()) {
				model->deleteAtIndex(perId);
			}
		});
		acts.push_back(del);

	}

	for (QAction* a : acts) {
		m.addAction(a);
	}

	if (acts.count() > 0) {
		m.exec(ui->dataBlockView->mapToGlobal(pt));
	}

	for (QAction* a : acts) {
		a->deleteLater();
	}
}


void MainWindow::clearOptimSolution() {

	if (_activeProject != nullptr) {
		resetSolution(_activeProject, this);
	} else {
		QMessageBox::warning(this, tr("Impossible to clear solution !"), tr("No project set"));
	}
}

void MainWindow::runCoarseOptim() {

	if (_activeProject != nullptr) {
		solveCoarse(_activeProject, this);
	} else {
		QMessageBox::warning(this, tr("Impossible to optimize !"), tr("No project set"));
	}
}

void MainWindow::runSparseOptim() {

	if (_activeProject != nullptr) {
		solveSparse(_activeProject, this);
	} else {
		QMessageBox::warning(this, tr("Impossible to optimize !"), tr("No project set"));
	}

}

} // namespace StereoVisionApp
