#include "lenseditor.h"
#include "ui_lenseditor.h"

#include "datablocks/camera.h"

namespace StereoVisionApp {

const QString LensEditor::LensEditorClassName = "StereoVisionApp::LensEditor";

LensEditor::LensEditor(QWidget *parent) :
	Editor(parent),
	ui(new Ui::LenseEditor),
	_cam(nullptr)
{
	ui->setupUi(this);

	ui->tabWidget->setEnabled(false);

	ui->pixelWidthSpinBox->setValue(1.);
	ui->pixelHeightSpinBox->setValue(1.);

	connect(ui->focalLengthSpintBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LensEditor::onParameterFLenChanged);

	connect(ui->pixelWidthSpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LensEditor::onParameterPixSizeChanged);
	connect(ui->pixelHeightSpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LensEditor::onParameterPixSizeChanged);

	connect(ui->opticalCenterXSpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LensEditor::OnParameterOpticalCenterXChanged);
	connect(ui->opticalCenterYSpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LensEditor::OnParameterOpticalCenterYChanged);

	connect(ui->radialK1SpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LensEditor::onParameterRadialK1Changed);
	connect(ui->radialK2SpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LensEditor::onParameterRadialK2Changed);
	connect(ui->radialK3SpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LensEditor::onParameterRadialK3Changed);

	connect(ui->tangentialP1SpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LensEditor::onParameterTangentialP1Changed);
	connect(ui->tangentialP2SpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LensEditor::onParameterTangentialP2Changed);

	connect(ui->skewB1SpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LensEditor::onParameterSkewB1Changed);
	connect(ui->skewB2SpinBox, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LensEditor::onParameterSkewB2Changed);

	connect(ui->tabWidget, &QTabWidget::currentChanged, this, &LensEditor::onTabSwitch);
}

LensEditor::~LensEditor()
{
	delete ui;
}

void LensEditor::setCamera(Camera* cam) {

	if (cam == _cam) {
		return;
	}

	if (_cam != nullptr) {
		disconnect(_cam, &Camera::FLenChanged, this, &LensEditor::activeCameraFlenChanged);
		disconnect(_cam, &Camera::pixelRatioChanged, this, &LensEditor::activeCameraPixRatioChanged);

		disconnect(_cam, &Camera::opticalCenterXChanged, this, &LensEditor::activeCameraFlenChanged);
		disconnect(_cam, &Camera::opticalCenterYChanged, this, &LensEditor::activeCameraPixRatioChanged);

		disconnect(_cam, &Camera::useRadialDistortionChanged, ui->radialGroupBox, &QGroupBox::setChecked);
		disconnect(_cam, &Camera::useRadialDistortionChanged, ui->optRadialGroupBox, &QGroupBox::setEnabled);

		disconnect(ui->radialGroupBox, &QGroupBox::toggled, _cam, &Camera::setUseRadialDistortionModel);

		disconnect(_cam, &Camera::k1Changed, this, &LensEditor::activeCameraK1Changed);
		disconnect(_cam, &Camera::k2Changed, this, &LensEditor::activeCameraK2Changed);
		disconnect(_cam, &Camera::k3Changed, this, &LensEditor::activeCameraK3Changed);

		disconnect(_cam, &Camera::useTangentialDistortionChanged, ui->tangentialGroupBox, &QGroupBox::setChecked);
		disconnect(_cam, &Camera::useTangentialDistortionChanged, ui->optTangentialGroupBox, &QGroupBox::setEnabled);

		disconnect(ui->tangentialGroupBox, &QGroupBox::toggled, _cam, &Camera::setUseTangentialDistortionModel);

		disconnect(_cam, &Camera::p1Changed, this, &LensEditor::activeCameraP1Changed);
		disconnect(_cam, &Camera::p2Changed, this, &LensEditor::activeCameraP2Changed);

		disconnect(_cam, &Camera::useSkewDistortionChanged, ui->skewGroupBox, &QGroupBox::setChecked);
		disconnect(_cam, &Camera::useSkewDistortionChanged, ui->optSkewGroupBox, &QGroupBox::setEnabled);

		disconnect(ui->skewGroupBox, &QGroupBox::toggled, _cam, &Camera::setUseSkewDistortionModel);

		disconnect(_cam, &Camera::B1Changed, this, &LensEditor::activeCameraB1Changed);
		disconnect(_cam, &Camera::B2Changed, this, &LensEditor::activeCameraB2Changed);

		disconnect(_cam, &Camera::optimizedFLenChanged, this, &LensEditor::activeCameraFlenChanged);
		disconnect(_cam, &Camera::optimizedPixelRatioChanged, this, &LensEditor::activeCameraPixRatioChanged);

		disconnect(_cam, &Camera::optimizedOpticalCenterXChanged, this, &LensEditor::activeCameraFlenChanged);
		disconnect(_cam, &Camera::optimizedOpticalCenterYChanged, this, &LensEditor::activeCameraPixRatioChanged);

		disconnect(_cam, &Camera::optimizedK1Changed, this, &LensEditor::activeCameraK1Changed);
		disconnect(_cam, &Camera::optimizedK2Changed, this, &LensEditor::activeCameraK2Changed);
		disconnect(_cam, &Camera::optimizedK3Changed, this, &LensEditor::activeCameraK3Changed);

		disconnect(_cam, &Camera::optimizedP1Changed, this, &LensEditor::activeCameraP1Changed);
		disconnect(_cam, &Camera::optimizedP2Changed, this, &LensEditor::activeCameraP2Changed);

		disconnect(_cam, &Camera::optimizedB1Changed, this, &LensEditor::activeCameraB1Changed);
		disconnect(_cam, &Camera::optimizedB2Changed, this, &LensEditor::activeCameraB2Changed);

		disconnect(_cam, &QObject::destroyed, this, &LensEditor::activeCameraDestroyed);
	}

	_cam = cam;
	ui->lensViewWidget->setCamera(_cam);

	ui->imageWidthSpinBox->setMaximum(_cam->imSize().width());
	ui->imageHeightSpinBox->setMaximum(_cam->imSize().height());

	ui->imageWidthSpinBox->setValue(_cam->imSize().width());
	ui->imageHeightSpinBox->setValue(_cam->imSize().height());

	ui->opticalCenterXSpinBox->setMinimum(-_cam->imSize().width()*3);
	ui->opticalCenterXSpinBox->setMaximum(_cam->imSize().width()*3);

	ui->opticalCenterYSpinBox->setMinimum(-_cam->imSize().height()*3);
	ui->opticalCenterYSpinBox->setMaximum(_cam->imSize().height()*3);

	ui->optOpticalCenterXSpinBox->setMinimum(-_cam->imSize().width()*3);
	ui->optOpticalCenterXSpinBox->setMaximum(_cam->imSize().width()*3);

	ui->optOpticalCenterYSpinBox->setMinimum(-_cam->imSize().height()*3);
	ui->optOpticalCenterYSpinBox->setMaximum(_cam->imSize().height()*3);

	if (_cam != nullptr) {
		connect(_cam, &Camera::FLenChanged, this, &LensEditor::activeCameraFlenChanged);
		connect(_cam, &Camera::pixelRatioChanged, this, &LensEditor::activeCameraPixRatioChanged);

		connect(_cam, &Camera::opticalCenterXChanged, this, &LensEditor::activeCameraOpticalCenterXChanged);
		connect(_cam, &Camera::opticalCenterYChanged, this, &LensEditor::activeCameraOpticalCenterYChanged);

		connect(_cam, &Camera::useRadialDistortionChanged, ui->radialGroupBox, &QGroupBox::setChecked);
		connect(_cam, &Camera::useRadialDistortionChanged, ui->optRadialGroupBox, &QGroupBox::setEnabled);

		connect(ui->radialGroupBox, &QGroupBox::toggled, _cam, &Camera::setUseRadialDistortionModel);

		connect(_cam, &Camera::k1Changed, this, &LensEditor::activeCameraK1Changed);
		connect(_cam, &Camera::k2Changed, this, &LensEditor::activeCameraK2Changed);
		connect(_cam, &Camera::k3Changed, this, &LensEditor::activeCameraK3Changed);

		connect(_cam, &Camera::useTangentialDistortionChanged, ui->tangentialGroupBox, &QGroupBox::setChecked);
		connect(_cam, &Camera::useTangentialDistortionChanged, ui->optTangentialGroupBox, &QGroupBox::setEnabled);

		connect(ui->tangentialGroupBox, &QGroupBox::toggled, _cam, &Camera::setUseTangentialDistortionModel);

		connect(_cam, &Camera::p1Changed, this, &LensEditor::activeCameraP1Changed);
		connect(_cam, &Camera::p2Changed, this, &LensEditor::activeCameraP2Changed);

		connect(_cam, &Camera::useSkewDistortionChanged, ui->skewGroupBox, &QGroupBox::setChecked);
		connect(_cam, &Camera::useSkewDistortionChanged, ui->optSkewGroupBox, &QGroupBox::setEnabled);

		connect(ui->skewGroupBox, &QGroupBox::toggled, _cam, &Camera::setUseSkewDistortionModel);

		connect(_cam, &Camera::B1Changed, this, &LensEditor::activeCameraB1Changed);
		connect(_cam, &Camera::B2Changed, this, &LensEditor::activeCameraB2Changed);

		connect(_cam, &Camera::optimizedFLenChanged, this, &LensEditor::activeCameraOptimizedFlenChanged);
		connect(_cam, &Camera::optimizedPixelRatioChanged, this, &LensEditor::activeCameraOptimizedPixRatioChanged);

		connect(_cam, &Camera::optimizedOpticalCenterXChanged, this, &LensEditor::activeCameraOptimizedOpticalCenterXChanged);
		connect(_cam, &Camera::optimizedOpticalCenterYChanged, this, &LensEditor::activeCameraOptimizedOpticalCenterYChanged);

		connect(_cam, &Camera::optimizedK1Changed, this, &LensEditor::activeCameraOptimizedK1Changed);
		connect(_cam, &Camera::optimizedK2Changed, this, &LensEditor::activeCameraOptimizedK2Changed);
		connect(_cam, &Camera::optimizedK3Changed, this, &LensEditor::activeCameraOptimizedK3Changed);

		connect(_cam, &Camera::optimizedP1Changed, this, &LensEditor::activeCameraOptimizedP1Changed);
		connect(_cam, &Camera::optimizedP2Changed, this, &LensEditor::activeCameraOptimizedP2Changed);

		connect(_cam, &Camera::optimizedB1Changed, this, &LensEditor::activeCameraOptimizedB1Changed);
		connect(_cam, &Camera::optimizedB2Changed, this, &LensEditor::activeCameraOptimizedB2Changed);

		connect(_cam, &QObject::destroyed, this, &LensEditor::activeCameraDestroyed);


		activeCameraFlenChanged();
		activeCameraPixRatioChanged();

		activeCameraOpticalCenterXChanged();
		activeCameraOpticalCenterYChanged();

		ui->radialGroupBox->setChecked(_cam->useRadialDistortionModel());
		ui->optRadialGroupBox->setEnabled(_cam->useRadialDistortionModel());

		activeCameraK1Changed();
		activeCameraK2Changed();
		activeCameraK3Changed();

		ui->tangentialGroupBox->setChecked(_cam->useTangentialDistortionModel());
		ui->optTangentialGroupBox->setEnabled(_cam->useTangentialDistortionModel());

		activeCameraP1Changed();
		activeCameraP2Changed();

		ui->skewGroupBox->setChecked(_cam->useSkewDistortionModel());
		ui->optSkewGroupBox->setEnabled(_cam->useSkewDistortionModel());

		activeCameraB1Changed();
		activeCameraB2Changed();


		activeCameraOptimizedFlenChanged();
		activeCameraOptimizedPixRatioChanged();

		activeCameraOptimizedOpticalCenterXChanged();
		activeCameraOptimizedOpticalCenterYChanged();

		activeCameraOptimizedK1Changed();
		activeCameraOptimizedK2Changed();
		activeCameraOptimizedK3Changed();

		activeCameraOptimizedP1Changed();
		activeCameraOptimizedP2Changed();

		activeCameraOptimizedB1Changed();
		activeCameraOptimizedB2Changed();


		ui->tabWidget->setEnabled(true);


	} else {
		ui->tabWidget->setEnabled(false);
	}


}


void LensEditor::activeCameraDestroyed() {
	_cam = nullptr;
}

void LensEditor::activeCameraFlenChanged() {

	if (_cam == nullptr) {
		return;
	}

	float flenMM = _cam->fLen().value()*ui->pixelWidthSpinBox->value()/1000;
	ui->focalLengthSpintBox->blockSignals(true);
	ui->focalLengthSpintBox->setValue(flenMM);
	ui->focalLengthSpintBox->blockSignals(false);

}
void LensEditor::activeCameraPixRatioChanged() {

	if (_cam == nullptr) {
		return;
	}

	float sH = ui->pixelWidthSpinBox->value()/_cam->pixelRatio().value();
	ui->pixelHeightSpinBox->blockSignals(true);
	ui->pixelHeightSpinBox->setValue(sH);
	ui->pixelHeightSpinBox->blockSignals(false);
}

void LensEditor::activeCameraOpticalCenterXChanged() {

	if (_cam == nullptr) {
		return;
	}

	ui->opticalCenterXSpinBox->blockSignals(true);
	ui->opticalCenterXSpinBox->setValue(_cam->opticalCenterX().value());
	ui->opticalCenterXSpinBox->blockSignals(false);
}
void LensEditor::activeCameraOpticalCenterYChanged() {

	if (_cam == nullptr) {
		return;
	}

	ui->opticalCenterYSpinBox->blockSignals(true);
	ui->opticalCenterYSpinBox->setValue(_cam->opticalCenterY().value());
	ui->opticalCenterYSpinBox->blockSignals(false);
}

void LensEditor::activeCameraK1Changed() {

	if (_cam == nullptr) {
		return;
	}

	ui->radialK1SpinBox->blockSignals(true);
	ui->radialK1SpinBox->setValue(_cam->k1().value());
	ui->radialK1SpinBox->blockSignals(false);

}
void LensEditor::activeCameraK2Changed() {

	if (_cam == nullptr) {
		return;
	}

	ui->radialK2SpinBox->blockSignals(true);
	ui->radialK2SpinBox->setValue(_cam->k2().value());
	ui->radialK2SpinBox->blockSignals(false);

}
void LensEditor::activeCameraK3Changed() {

	if (_cam == nullptr) {
		return;
	}

	ui->radialK3SpinBox->blockSignals(true);
	ui->radialK3SpinBox->setValue(_cam->k3().value());
	ui->radialK3SpinBox->blockSignals(false);
}

void LensEditor::activeCameraP1Changed() {

	if (_cam == nullptr) {
		return;
	}

	ui->tangentialP1SpinBox->blockSignals(true);
	ui->tangentialP1SpinBox->setValue(_cam->p1().value());
	ui->tangentialP1SpinBox->blockSignals(false);

}
void LensEditor::activeCameraP2Changed() {

	if (_cam == nullptr) {
		return;
	}

	ui->tangentialP2SpinBox->blockSignals(true);
	ui->tangentialP2SpinBox->setValue(_cam->p2().value());
	ui->tangentialP2SpinBox->blockSignals(false);

}

void LensEditor::activeCameraB1Changed() {

	if (_cam == nullptr) {
		return;
	}

	ui->skewB1SpinBox->blockSignals(true);
	ui->skewB1SpinBox->setValue(_cam->B1().value());
	ui->skewB1SpinBox->blockSignals(false);

}
void LensEditor::activeCameraB2Changed() {

	if (_cam == nullptr) {
		return;
	}

	ui->skewB2SpinBox->blockSignals(true);
	ui->skewB2SpinBox->setValue(_cam->B2().value());
	ui->skewB2SpinBox->blockSignals(false);

}

void LensEditor::activeCameraOptimizedFlenChanged() {

	if (_cam == nullptr) {
		return;
	}

	ui->optFocalLegthSpinBox->setValue(_cam->optimizedFLen().value());

}
void LensEditor::activeCameraOptimizedPixRatioChanged() {

	if (_cam == nullptr) {
		return;
	}

	ui->optPixelAspectRatioSpinBox->setValue(_cam->optimizedPixelRatio().value());
}

void LensEditor::activeCameraOptimizedOpticalCenterXChanged() {

	if (_cam == nullptr) {
		return;
	}

	ui->optOpticalCenterXSpinBox->setValue(_cam->optimizedOpticalCenterX().value());

}
void LensEditor::activeCameraOptimizedOpticalCenterYChanged() {

	if (_cam == nullptr) {
		return;
	}

	ui->optOpticalCenterYSpinBox->setValue(_cam->optimizedOpticalCenterY().value());

}

void LensEditor::activeCameraOptimizedK1Changed() {

	if (_cam == nullptr) {
		return;
	}

	ui->optRadialK1SpinBox->setValue(_cam->optimizedK1().value());
}
void LensEditor::activeCameraOptimizedK2Changed() {

	if (_cam == nullptr) {
		return;
	}

	ui->optRadialK2SpinBox->setValue(_cam->optimizedK2().value());

}
void LensEditor::activeCameraOptimizedK3Changed() {

	if (_cam == nullptr) {
		return;
	}

	ui->optRadialK3SpinBox->setValue(_cam->optimizedK3().value());

}

void LensEditor::activeCameraOptimizedP1Changed() {

	if (_cam == nullptr) {
		return;
	}

	ui->optTangentialP1SpinBox->setValue(_cam->optimizedP1().value());
}
void LensEditor::activeCameraOptimizedP2Changed() {

	if (_cam == nullptr) {
		return;
	}

	ui->optTangentialP2SpinBox->setValue(_cam->optimizedP2().value());

}

void LensEditor::activeCameraOptimizedB1Changed() {

	if (_cam == nullptr) {
		return;
	}

	ui->optSkewB1SpinBox->setValue(_cam->optimizedB1().value());

}
void LensEditor::activeCameraOptimizedB2Changed() {

	if (_cam == nullptr) {
		return;
	}

	ui->optSkewB2SpinBox->setValue(_cam->optimizedB2().value());

}

void LensEditor::onParameterFLenChanged() {

	if (_cam == nullptr) {
		return;
	}

	float fLenPx = ui->focalLengthSpintBox->value() / ui->pixelWidthSpinBox->value() * 1000;
	_cam->setFLen(fLenPx);
}

void LensEditor::onParameterPixSizeChanged() {

	if (_cam == nullptr) {
		return;
	}

	float fLenPx = ui->focalLengthSpintBox->value() / ui->pixelWidthSpinBox->value() * 1000;
	_cam->setFLen(fLenPx);

	float pxRatio = ui->pixelWidthSpinBox->value()/ui->pixelHeightSpinBox->value();
	_cam->setPixelRatio(pxRatio);
}

void LensEditor::OnParameterOpticalCenterXChanged() {

	if (_cam == nullptr) {
		return;
	}

	_cam->setOpticalCenterX(ui->opticalCenterXSpinBox->value());

}
void LensEditor::OnParameterOpticalCenterYChanged() {

	if (_cam == nullptr) {
		return;
	}

	_cam->setOpticalCenterY(ui->opticalCenterYSpinBox->value());

}

void LensEditor::onParameterRadialK1Changed() {

	if (_cam == nullptr) {
		return;
	}

	_cam->setK1(ui->radialK1SpinBox->value());

}
void LensEditor::onParameterRadialK2Changed() {

	if (_cam == nullptr) {
		return;
	}

	_cam->setK2(ui->radialK2SpinBox->value());

}
void LensEditor::onParameterRadialK3Changed() {

	if (_cam == nullptr) {
		return;
	}

	_cam->setK3(ui->radialK3SpinBox->value());

}

void LensEditor::onParameterTangentialP1Changed() {

	if (_cam == nullptr) {
		return;
	}

	_cam->setP1(ui->tangentialP1SpinBox->value());

}
void LensEditor::onParameterTangentialP2Changed() {

	if (_cam == nullptr) {
		return;
	}

	_cam->setP2(ui->tangentialP2SpinBox->value());

}

void LensEditor::onParameterSkewB1Changed() {

	if (_cam == nullptr) {
		return;
	}

	_cam->setB1(ui->skewB1SpinBox->value());

}
void LensEditor::onParameterSkewB2Changed() {

	if (_cam == nullptr) {
		return;
	}

	_cam->setB2(ui->skewB2SpinBox->value());

}

void LensEditor::onTabSwitch() {

	bool useOpt = ui->tabWidget->currentIndex() == 1;
	ui->lensViewWidget->useOptimizedValues(useOpt);

}

LensEditorFactory::LensEditorFactory(QObject* parent) :
	EditorFactory(parent)
{

}

QString LensEditorFactory::TypeDescrName() const {
	return tr("Lens Editor");
}
QString LensEditorFactory::itemClassName() const {
	return LensEditor::LensEditorClassName;
}
Editor* LensEditorFactory::factorizeEditor(QWidget* parent) const {
	return new LensEditor(parent);
}

}//namespace StereoVisionApp
