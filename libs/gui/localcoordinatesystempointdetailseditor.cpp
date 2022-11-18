#include "localcoordinatesystempointdetailseditor.h"
#include "ui_localcoordinatesystempointdetailseditor.h"

#include "datablocks/localcoordinatesystem.h"

#include "sparsesolver/localcoordinatespointssolutionmodel.h"

#include <QSortFilterProxyModel>

namespace StereoVisionApp {

LocalCoordinateSystemPointDetailsEditor::LocalCoordinateSystemPointDetailsEditor(QWidget *parent) :
	Editor(parent),
	ui(new Ui::LocalCoordinateSystemPointDetailsEditor)
{
	ui->setupUi(this);
	_model = new LocalCoordinatesPointsSolutionModel(this);

	QSortFilterProxyModel* sortable_model = new QSortFilterProxyModel(this);

	sortable_model->setSourceModel(_model);

	ui->dataTreeView->setModel(sortable_model);
	ui->dataTreeView->setSortingEnabled(true);
}

LocalCoordinateSystemPointDetailsEditor::~LocalCoordinateSystemPointDetailsEditor()
{
	delete ui;
}

void LocalCoordinateSystemPointDetailsEditor::setLocalCoordinateSystem(LocalCoordinateSystem* lcs) {
	_model->setLocalCoordinateSystem(lcs);
}


LocalCoordinateSystemPointDetailsEditorFactory::LocalCoordinateSystemPointDetailsEditorFactory(QObject* parent) :
	EditorFactory(parent)
{

}

QString LocalCoordinateSystemPointDetailsEditorFactory::TypeDescrName() const {
	return tr("Local Frame Points Details Editor");;
}
QString LocalCoordinateSystemPointDetailsEditorFactory::itemClassName() const {
	return LocalCoordinateSystemPointDetailsEditor::staticMetaObject.className();
}

Editor* LocalCoordinateSystemPointDetailsEditorFactory::factorizeEditor(QWidget* parent) const {
	return new LocalCoordinateSystemPointDetailsEditor(parent);
}

} //namespace StereoVisionApp
