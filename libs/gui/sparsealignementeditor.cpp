#include "sparsealignementeditor.h"
#include "ui_sparsealignementeditor.h"

namespace StereoVisionApp {

const QString SparseAlignementEditor::SparseAlignementEditorClassName = "StereoVisionApp::SparseAlignementEditor";

SparseAlignementEditor::SparseAlignementEditor(QWidget *parent) :
	Editor(parent),
	ui(new Ui::SparseAlignementEditor)
{
	ui->setupUi(this);

	//pass messages
	connect(ui->widget, &SparseAlignementViewer::sendStatusMessage,
			this, &SparseAlignementEditor::sendStatusMessage);
}

SparseAlignementEditor::~SparseAlignementEditor()
{
	delete ui;
}

void SparseAlignementEditor::beforeProjectChange(Project* np) {
	Editor::beforeProjectChange(np);
	ui->widget->setProject(np);
}

void SparseAlignementEditor::reloadLandmarks() {
	ui->widget->reloadLandmarks();
}
void SparseAlignementEditor::clearLandmarks() {
	ui->widget->clearLandmarks();
}


SparseAlignementEditorFactory::SparseAlignementEditorFactory(QObject* parent) :
	EditorFactory(parent)
{

}
QString SparseAlignementEditorFactory::TypeDescrName() const {
	return tr("Sparse Alignement Viewer");
}
QString SparseAlignementEditorFactory::itemClassName() const {
	return SparseAlignementEditor::SparseAlignementEditorClassName;
}
Editor* SparseAlignementEditorFactory::factorizeEditor(QWidget* parent) const {
	return new SparseAlignementEditor(parent);
}

}//namespace StereoVisionApp
