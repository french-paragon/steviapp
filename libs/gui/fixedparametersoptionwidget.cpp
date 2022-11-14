#include "fixedparametersoptionwidget.h"
#include "ui_fixedparametersoptionwidget.h"

namespace StereoVisionApp {

FixedParametersOptionWidget::FixedParametersOptionWidget(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::FixedParametersOptionWidget)
{
	ui->setupUi(this);
}

FixedParametersOptionWidget::~FixedParametersOptionWidget()
{
	delete ui;
}

FixedParameters FixedParametersOptionWidget::getFixedParameters() const {

	FixedParameters flag = NoFixedParameters;

	if (ui->fixedCameraInternalsCheckbox->isChecked()) {
		flag |= CameraInternal;
	}

	if (ui->fixedStereoRigsCheckbox->isChecked()) {
		flag |= StereoRigs;
	}

	if (ui->fixedCameraExternalsCheckbox->isChecked()) {
		flag |= CameraExternal;
	}

	return flag;
}
void FixedParametersOptionWidget::setFixedParameters(FixedParameters parameters) {

	if (parameters & CameraInternal) {
		ui->fixedCameraInternalsCheckbox->setChecked(true);
	} else {
		ui->fixedCameraInternalsCheckbox->setChecked(false);
	}

	if (parameters & StereoRigs) {
		ui->fixedStereoRigsCheckbox->setChecked(true);
	} else {
		ui->fixedStereoRigsCheckbox->setChecked(false);
	}

	if (parameters & CameraExternal) {
		ui->fixedCameraExternalsCheckbox->setChecked(true);
	} else {
		ui->fixedCameraExternalsCheckbox->setChecked(false);
	}
}

} // namespace StereoVisionApp
