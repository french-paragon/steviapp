#include "stereosequenceexportoptiondialog.h"
#include "ui_stereosequenceexportoptiondialog.h"

#include <QFileDialog>

namespace StereoVisionApp {

StereoSequenceExportOptionDialog::StereoSequenceExportOptionDialog(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::StereoSequenceExportOptionDialog)
{
	ui->setupUi(this);

	connect(ui->exportDirButton, &QPushButton::clicked, this, &StereoSequenceExportOptionDialog::onOpenDirButtonClicked);
}

StereoSequenceExportOptionDialog::~StereoSequenceExportOptionDialog()
{
	delete ui;
}

QString StereoSequenceExportOptionDialog::exportDir() const {
	return ui->exportDirLine->text();
}
void StereoSequenceExportOptionDialog::setExportDir(QString const& dir) {
	ui->exportDirLine->setText(dir);
}

int StereoSequenceExportOptionDialog::searchWidth() const {
	return ui->searchWidthSpinBox->value();
}
void StereoSequenceExportOptionDialog::setSearchWidth(int width) {
	ui->searchWidthSpinBox->setValue(width);
}

int StereoSequenceExportOptionDialog::searchRadius() const {
	return ui->searchRadiusSpinBox->value();
}
void StereoSequenceExportOptionDialog::setSearchRadius(int radius) const {
	ui->searchRadiusSpinBox->setValue(radius);
}

float StereoSequenceExportOptionDialog::maxDist() const {
	return ui->maxDistanceSpinBox->value();
}
void StereoSequenceExportOptionDialog::setMaxDist(float dist) {
	ui->maxDistanceSpinBox->setValue(dist);
}

int StereoSequenceExportOptionDialog::erodingDistance() const {
	return ui->erosionSpinBox->value();
}
void StereoSequenceExportOptionDialog::setErodingDistance(int dist) const {
	ui->erosionSpinBox->setValue(dist);
}

int StereoSequenceExportOptionDialog::openingDistance() const {
	return ui->openingSpinBox->value();
}
void StereoSequenceExportOptionDialog::setOpeningDistance(int dist) const {
	ui->openingSpinBox->setValue(dist);
}

void StereoSequenceExportOptionDialog::onOpenDirButtonClicked() {

	QString outFolder = QFileDialog::getExistingDirectory(this, QObject::tr("Export directory"));

	if (!outFolder.isEmpty()) {
		setExportDir(outFolder);
	}

}

} // namespace StereoVisionApp
