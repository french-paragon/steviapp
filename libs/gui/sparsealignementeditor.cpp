#include "sparsealignementeditor.h"
#include "ui_sparsealignementeditor.h"

namespace StereoVisionApp {

const QString SparseAlignementEditor::SparseAlignementEditorClassName = "StereoVisionApp::SparseAlignementEditor";

SparseAlignementEditor::SparseAlignementEditor(QWidget *parent) :
	Editor(parent),
	ui(new Ui::SparseAlignementEditor)
{
	ui->setupUi(this);

	_viewerInterface = new ProjectSparseAlignementDataInterface(this);
	ui->widget->setInterface(_viewerInterface);

	//pass messages
	connect(_viewerInterface, &ProjectSparseAlignementDataInterface::sendStatusMessage,
			this, &SparseAlignementEditor::sendStatusMessage);

	connect(ui->widget, &SparseAlignementViewer::sendStatusMessage,
			this, &SparseAlignementEditor::sendStatusMessage);
}

SparseAlignementEditor::~SparseAlignementEditor()
{
	delete ui;
}

void SparseAlignementEditor::beforeProjectChange(Project* np) {
	Editor::beforeProjectChange(np);
	_viewerInterface->setProject(np);
}

void SparseAlignementEditor::reloadLandmarks() {
	ui->widget->reloadLandmarks();
}
void SparseAlignementEditor::clearLandmarks() {
	ui->widget->clearLandmarks();
}

bool SparseAlignementEditor::addDrawable(QString const& name, OpenGlDrawable* drawable) {

    if (drawable == nullptr) {
        return false;
    }

    drawable->setParent(this);

    if (_additionalDrawables.contains(name)) {
        drawable->deleteLater();
        return false;
    }

    _additionalDrawables.insert(name, drawable);
    ui->widget->addDrawable(drawable);

    return true;
}

OpenGlDrawable* SparseAlignementEditor::getDrawable(QString const& name) {

    return _additionalDrawables.value(name, nullptr);

}

void SparseAlignementEditor::removeDrawable(QString const& name) {

    if (_additionalDrawables.contains(name)) {

        OpenGlDrawable* drawable = _additionalDrawables.value(name);

        _additionalDrawables.remove(name);
        ui->widget->removeDrawable(drawable);
    }
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
