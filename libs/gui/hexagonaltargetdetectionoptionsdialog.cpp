#include "hexagonaltargetdetectionoptionsdialog.h"
#include "ui_hexagonaltargetdetectionoptionsdialog.h"

namespace StereoVisionApp {

HexagonalTargetDetectionOptionsDialog::HexagonalTargetDetectionOptionsDialog(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::HexagonalTargetDetectionOptionsDialog)
{
	ui->setupUi(this);
}

HexagonalTargetDetectionOptionsDialog::~HexagonalTargetDetectionOptionsDialog()
{
	delete ui;
}

double HexagonalTargetDetectionOptionsDialog::minThreshold() const {
	return ui->minGrayThresholdSpinBox->value();
}
void HexagonalTargetDetectionOptionsDialog::setMinThreshold(double threshold) {
	ui->minGrayThresholdSpinBox->setValue(threshold);
}

double HexagonalTargetDetectionOptionsDialog::diffThreshold() const {
	return ui->diffGrayThresholdSpinBox->value();
}
void HexagonalTargetDetectionOptionsDialog::setDiffThreshold(double threshold) {
	ui->diffGrayThresholdSpinBox->setValue(threshold);
}

int HexagonalTargetDetectionOptionsDialog::minArea() const {
	return ui->minAreaSpinBox->value();
}
void HexagonalTargetDetectionOptionsDialog::setMinArea(int minArea) {
	ui->minAreaSpinBox->setValue(minArea);
}

int HexagonalTargetDetectionOptionsDialog::maxArea() const {
	return ui->maxAreaSpinBox->value();
}
void HexagonalTargetDetectionOptionsDialog::setMaxArea(int maxArea) {
	ui->maxAreaSpinBox->setValue(maxArea);
}

double HexagonalTargetDetectionOptionsDialog::minToMaxAxisRatioThreshold() const {
	return ui->minorToMajorAxisRatioSpinBox->value();
}
void HexagonalTargetDetectionOptionsDialog::setMinToMaxAxisRatioThreshold(double threshold) {
	ui->minorToMajorAxisRatioSpinBox->setValue(threshold);
}

double HexagonalTargetDetectionOptionsDialog::hexagonMaxRelDiameter() const {
	return ui->hexRelDiameterSpinBox->value();
}
void HexagonalTargetDetectionOptionsDialog::setHexagonMaxRelDiameter(double threshold) {
	ui->hexRelDiameterSpinBox->setValue(threshold);
}

double HexagonalTargetDetectionOptionsDialog::redGain() const {
	return ui->redGainSpinBox->value();
}
void HexagonalTargetDetectionOptionsDialog::setRedGain(double gain) {
	ui->redGainSpinBox->setValue(gain);
}

double HexagonalTargetDetectionOptionsDialog::greenGain() const {
	return ui->greenGainSpinBox->value();
}
void HexagonalTargetDetectionOptionsDialog::setGreenGain(double gain) {
	ui->greenGainSpinBox->setValue(gain);
}

double HexagonalTargetDetectionOptionsDialog::blueGain() const {
	return ui->blueGainSpinBox->value();
}
void HexagonalTargetDetectionOptionsDialog::setBlueGain(double gain) {
	ui->blueGainSpinBox->setValue(gain);
}

bool HexagonalTargetDetectionOptionsDialog::replaceOld() const {
	return ui->removePreviouslyDetectedCheckBox->isChecked();
}
void HexagonalTargetDetectionOptionsDialog::setReplaceOld(bool replace) {
	ui->removePreviouslyDetectedCheckBox->setChecked(replace);
}


} // namespace StereoVisionApp
