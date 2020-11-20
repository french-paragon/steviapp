#include "sparsealignementeditor.h"
#include "ui_sparsealignementeditor.h"

namespace StereoVisionApp {

const QString SparseAlignementEditor::SparseAlignementEditorClassName = "StereoVisionApp::SparseAlignementEditor";

SparseAlignementEditor::SparseAlignementEditor(QWidget *parent) :
	Editor(parent),
	ui(new Ui::SparseAlignementEditor)
{
	ui->setupUi(this);
}

SparseAlignementEditor::~SparseAlignementEditor()
{
	delete ui;
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
