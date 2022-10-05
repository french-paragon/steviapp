#include "cameracalibrationsparsealignementeditor.h"
#include "ui_cameracalibrationsparsealignementeditor.h"

#include "datablocks/cameracalibration.h"
#include "./cameracalibrationsparsealignementviewerinterface.h"

namespace StereoVisionApp {

CameraCalibrationSparseAlignementEditor::CameraCalibrationSparseAlignementEditor(QWidget *parent) :
	Editor(parent),
	ui(new Ui::CameraCalibrationSparseAlignementEditor)
{
	ui->setupUi(this);

	_viewerInterface = new CameraCalibrationSparseAlignementViewerInterface(1.0, this);
	ui->widget->setInterface(_viewerInterface);

	//pass messages
	connect(_viewerInterface, &ProjectSparseAlignementDataInterface::sendStatusMessage,
			this, &CameraCalibrationSparseAlignementEditor::sendStatusMessage);

	connect(ui->widget, &SparseAlignementViewer::sendStatusMessage,
			this, &CameraCalibrationSparseAlignementEditor::sendStatusMessage);
}

CameraCalibrationSparseAlignementEditor::~CameraCalibrationSparseAlignementEditor()
{
	delete ui;
}

void CameraCalibrationSparseAlignementEditor::setCalibration(CameraCalibration* c) {
	_viewerInterface->setCalibration(c);
}
void CameraCalibrationSparseAlignementEditor::clearCalibration() {
	_viewerInterface->clearCalibration();
}

float CameraCalibrationSparseAlignementEditor::getGridSize() const {
	return _viewerInterface->getGridSize();
}
void CameraCalibrationSparseAlignementEditor::setGridSize(float grid_size) {
	_viewerInterface->setGridSize(grid_size);
}

CameraCalibrationSparseAlignementEditorFactory::CameraCalibrationSparseAlignementEditorFactory(QObject* parent) :
	EditorFactory(parent)
{

}

QString CameraCalibrationSparseAlignementEditorFactory::TypeDescrName() const {
	return tr("Calibration Alignement Viewer");
}
QString CameraCalibrationSparseAlignementEditorFactory::itemClassName() const {
	return CameraCalibrationSparseAlignementEditor::staticMetaObject.className();
}
Editor* CameraCalibrationSparseAlignementEditorFactory::factorizeEditor(QWidget* parent) const {
	return new CameraCalibrationSparseAlignementEditor(parent);
}

} // namespace StereoVisionApp
