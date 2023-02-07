#include "stereosequenceimageexportoptiondialog.h"
#include "ui_stereosequenceimageexportoptiondialog.h"

#include <QFileDialog>

namespace StereoVisionApp {

StereoSequenceImageExportOptionDialog::StereoSequenceImageExportOptionDialog(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::StereoSequenceImageExportOptionDialog)
{
	ui->setupUi(this);

	connect(ui->exportDirSelectButton, &QPushButton::clicked, this, &StereoSequenceImageExportOptionDialog::openExportDirectory);
	connect(ui->backgroundImageLeftOpenButton, &QPushButton::clicked, this, &StereoSequenceImageExportOptionDialog::openLeftBgImage);
	connect(ui->backgroundImageRightOpenButton, &QPushButton::clicked, this, &StereoSequenceImageExportOptionDialog::openRightBgImage);
}

StereoSequenceImageExportOptionDialog::~StereoSequenceImageExportOptionDialog()
{
	delete ui;
}

QString StereoSequenceImageExportOptionDialog::exportDir() const {
	return ui->exportDirLineEdit->text();
}
void StereoSequenceImageExportOptionDialog::setExportDir(QString const& dir) {
	ui->exportDirLineEdit->setText(dir);
}

QString StereoSequenceImageExportOptionDialog::leftBackgroundImage() const {
	return ui->backgroundImageLeftLineEdit->text();
}
void StereoSequenceImageExportOptionDialog::setLeftBackgroundImage(QString const& dir) {
	ui->backgroundImageLeftLineEdit->setText(dir);
}

QString StereoSequenceImageExportOptionDialog::rightBackgroundImage() const {
	return ui->backgroundImageRightLineEdit->text();
}
void StereoSequenceImageExportOptionDialog::setRightBackgroundImage(QString const& dir) {
	ui->backgroundImageRightLineEdit->setText(dir);
}

int StereoSequenceImageExportOptionDialog::searchWidth() const {
	return ui->searchWidthSpinBox->value();
}
void StereoSequenceImageExportOptionDialog::setSearchWidth(int width) {
	ui->searchWidthSpinBox->setValue(width);
}

int StereoSequenceImageExportOptionDialog::searchRadius() const {
	return ui->searchRadiusSpinBox->value();
}
void StereoSequenceImageExportOptionDialog::setSearchRadius(int radius) {
	ui->searchRadiusSpinBox->setValue(radius);
}

int StereoSequenceImageExportOptionDialog::hiearchicalLevel() const {
	return ui->hierarchicalLevelSpinBox->value();
}
void StereoSequenceImageExportOptionDialog::setHiearchicalLevel(int level) {
	ui->hierarchicalLevelSpinBox->setValue(level);
}

int StereoSequenceImageExportOptionDialog::transitionCostWeight() const {
	return ui->transitionWeightSpinBox->value();
}
void StereoSequenceImageExportOptionDialog::setTransitionCostWeight(int weight) {
	ui->transitionWeightSpinBox->setValue(weight);
}

int StereoSequenceImageExportOptionDialog::visualWeight() const {
	return ui->visualWeightSpinBox->value();
}
void StereoSequenceImageExportOptionDialog::setVisualWeight(int weight) {
	ui->visualWeightSpinBox->setValue(weight);
}

int StereoSequenceImageExportOptionDialog::visualPatchRadius() const {
	return ui->visualPatchRadiusSpinBox->value();
}
void StereoSequenceImageExportOptionDialog::setVisualPatchRadius(int radius) {
	ui->visualPatchRadiusSpinBox->setValue(radius);
}

float StereoSequenceImageExportOptionDialog::visualThreshold() const {
	return ui->visualThresholdSpinBox->value();
}
void StereoSequenceImageExportOptionDialog::setVisualThreshold(float threshold) {
	ui->visualThresholdSpinBox->setValue(threshold);
}

int StereoSequenceImageExportOptionDialog::depthWeight() const {
	return ui->depthWeightSpinBox->value();
}
void StereoSequenceImageExportOptionDialog::setDepthWeight(int weight) {
	ui->depthWeightSpinBox->setValue(weight);
}

int StereoSequenceImageExportOptionDialog::depthThreshold() const {
	return ui->depthDisparityThresholdSpinBox->value();
}
void StereoSequenceImageExportOptionDialog::setDepthThreshold(int threshold) {
	ui->depthDisparityThresholdSpinBox->setValue(threshold);
}

void StereoSequenceImageExportOptionDialog::openExportDirectory() {

	QString outFolder = QFileDialog::getExistingDirectory(this, QObject::tr("Export directory"));

	if (!outFolder.isEmpty()) {
		setExportDir(outFolder);
	}

}

void StereoSequenceImageExportOptionDialog::openLeftBgImage() {

	QString file = openImage();

	if (!file.isEmpty()) {
		setLeftBackgroundImage(file);
	}

}
void StereoSequenceImageExportOptionDialog::openRightBgImage() {

	QString file = openImage();

	if (!file.isEmpty()) {
		setLeftBackgroundImage(file);
	}
}

QString StereoSequenceImageExportOptionDialog::openImage() {

	QString file = QFileDialog::getSaveFileName(this, QObject::tr("Export directory"), QString(), QObject::tr("images(*.jpg,*.png,*.tiff,*.stevimg)"));

	return file;

}

} // namespace StereoVisionApp
