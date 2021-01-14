#include "rectifiedimageexportoptionsdialog.h"
#include "ui_rectifiedimageexportoptionsdialog.h"

#include <QFileDialog>
namespace StereoVisionApp {

RectifiedImageExportOptionsDialog::RectifiedImageExportOptionsDialog(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::RectifiedImageExportOptionsDialog)
{
	ui->setupUi(this);

	connect(ui->chooseFolderButton, &QPushButton::clicked,
			this, &RectifiedImageExportOptionsDialog::onFolderSelectButtonClicked);
}

RectifiedImageExportOptionsDialog::~RectifiedImageExportOptionsDialog()
{
	delete ui;
}

QString RectifiedImageExportOptionsDialog::selectedFolder() const {
	return ui->folderLineEdit->text();
}
bool RectifiedImageExportOptionsDialog::useOptimizedCameraParameters() const {
	return ui->parameterSetSelectComboBox->currentIndex() == 1;
}
float RectifiedImageExportOptionsDialog::gamma() const {
	return ui->gammaSpinBox->value();
}

void RectifiedImageExportOptionsDialog::onFolderSelectButtonClicked() {

	QString directory = ui->folderLineEdit->text();
	directory = QFileDialog::getExistingDirectory(this, tr("Output directory"), directory);

	if (!directory.isEmpty()) {
		ui->folderLineEdit->setText(directory);
	}

}

} //namespace StereoVisionApp
