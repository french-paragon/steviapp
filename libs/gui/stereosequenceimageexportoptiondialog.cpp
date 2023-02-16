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


int StereoSequenceImageExportOptionDialog::erosionRadius() const {
	return ui->erosionRadiusSpinBox->value();
}
void StereoSequenceImageExportOptionDialog::setErosionRadius(int radius) {
	ui->erosionRadiusSpinBox->setValue(radius);
}

int StereoSequenceImageExportOptionDialog::dilationRadius() const {
	return ui->dilationRadiusSpinBox->value();
}
void StereoSequenceImageExportOptionDialog::setDilationRadius(int radius) {
	ui->dilationRadiusSpinBox->setValue(radius);
}

int StereoSequenceImageExportOptionDialog::extensionRadius() const {
	return ui->extensionRadiusSpinBox->value();
}
void StereoSequenceImageExportOptionDialog::setExtensionRadius(int radius) {
	ui->extensionRadiusSpinBox->setValue(radius);
}

int StereoSequenceImageExportOptionDialog::nHistogramBins() const {
	return ui->nHistogramBinsSpinBox->value();
}
void StereoSequenceImageExportOptionDialog::setNHistogramBins(int nBins) {
	ui->nHistogramBinsSpinBox->setValue(nBins);
}

int StereoSequenceImageExportOptionDialog::cutoffHistogramBin() const {
	return ui->histogramBinThresholdSpinBox->value();
}
void StereoSequenceImageExportOptionDialog::setCutoffHistogramBins(int cutoff) {
	ui->histogramBinThresholdSpinBox->setValue(cutoff);
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
