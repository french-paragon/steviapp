#include "imagepointdetailseditor.h"
#include "ui_imagepointdetailseditor.h"

#include "sparsesolver/imagepointssolutionmodel.h"

#include <QSortFilterProxyModel>

StereoVisionApp::ImagePointDetailsEditor::ImagePointDetailsEditor(QWidget *parent) :
	Editor(parent),
	ui(new Ui::ImagePointDetailsEditor)
{
	ui->setupUi(this);
	_model = new ImagePointsSolutionModel(this);

	QSortFilterProxyModel* sortable_model = new QSortFilterProxyModel(this);

	sortable_model->setSourceModel(_model);

	ui->dataTreeView->setModel(sortable_model);
	ui->dataTreeView->setSortingEnabled(true);
}

StereoVisionApp::ImagePointDetailsEditor::~ImagePointDetailsEditor()
{
	delete ui;
}

void StereoVisionApp::ImagePointDetailsEditor::setImage(Image* img) {
	_model->setImage(img);
}

StereoVisionApp::ImagePointDetailsEditorFactory::ImagePointDetailsEditorFactory(QObject* parent):
	EditorFactory(parent)
{

}
QString StereoVisionApp::ImagePointDetailsEditorFactory::TypeDescrName() const {
	return tr("Image Points Details Editor");
}
QString StereoVisionApp::ImagePointDetailsEditorFactory::itemClassName() const {
	return ImagePointDetailsEditor::staticMetaObject.className();
}
StereoVisionApp::Editor* StereoVisionApp::ImagePointDetailsEditorFactory::factorizeEditor(QWidget* parent) const {
	return new ImagePointDetailsEditor(parent);
}
