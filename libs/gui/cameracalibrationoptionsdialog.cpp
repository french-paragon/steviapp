#include "cameracalibrationoptionsdialog.h"
#include "ui_cameracalibrationoptionsdialog.h"

namespace StereoVisionApp {

CameraCalibrationOptionsDialog::CameraCalibrationOptionsDialog(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::CameraCalibrationOptionsDialog)
{
	ui->setupUi(this);
}

CameraCalibrationOptionsDialog::~CameraCalibrationOptionsDialog()
{
	delete ui;
}

float CameraCalibrationOptionsDialog::checkboardCornerSize() const {
	return ui->gridSquareSizeSpinBox->value();
}
void CameraCalibrationOptionsDialog::setCheckboardCornerSize(float width) {
	ui->gridSquareSizeSpinBox->setValue(width);
}

int CameraCalibrationOptionsDialog::nIterations() const {
	return ui->optStepsSpinBox->value();
}
void CameraCalibrationOptionsDialog::setNIterations(int nIter) {
	ui->optStepsSpinBox->setValue(nIter);
}

} // namespace StereoVisionApp
