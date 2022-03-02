#include "landmarkpointdetailseditor.h"
#include "ui_landmarkpointdetailseditor.h"

#include "sparsesolver/landmarkpointssolutionmodel.h"

#include <QSortFilterProxyModel>

StereoVisionApp::LandmarkPointDetailsEditor::LandmarkPointDetailsEditor(QWidget *parent) :
	Editor(parent),
	ui(new Ui::LandmarkPointDetailsEditor)
{
	ui->setupUi(this);
	_model = new LandmarkPointsSolutionModel(this);

	QSortFilterProxyModel* sortable_model = new QSortFilterProxyModel(this);

	sortable_model->setSourceModel(_model);

	ui->dataTreeView->setModel(sortable_model);
	ui->dataTreeView->setSortingEnabled(true);
}

StereoVisionApp::LandmarkPointDetailsEditor::~LandmarkPointDetailsEditor()
{
	delete ui;
}

void StereoVisionApp::LandmarkPointDetailsEditor::setLandmark(Landmark* lm) {
	_model->setLandmark(lm);
}

StereoVisionApp::LandmarkPointDetailsEditorFactory::LandmarkPointDetailsEditorFactory(QObject* parent):
	EditorFactory(parent)
{

}
QString StereoVisionApp::LandmarkPointDetailsEditorFactory::TypeDescrName() const {
	return tr("Image Points Details Editor");
}
QString StereoVisionApp::LandmarkPointDetailsEditorFactory::itemClassName() const {
	return LandmarkPointDetailsEditor::staticMetaObject.className();
}
StereoVisionApp::Editor* StereoVisionApp::LandmarkPointDetailsEditorFactory::factorizeEditor(QWidget* parent) const {
	return new LandmarkPointDetailsEditor(parent);
}
