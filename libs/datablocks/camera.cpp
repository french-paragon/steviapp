#include "camera.h"

#include "./itemdatamodel.h"

namespace StereoVisionApp {

Camera::Camera(Project *parent) :
	Camera({800, 800}, parent)
{
	extendDataModel();
}

Camera::Camera(QSize imSize, Project* parent)  :
	DataBlock(parent),
	_imSize(imSize),
	_pix_ratio(1.0, true),
	_useRadialDistortionModel(true),
	_k1(0, false),
	_k2(0, false),
	_k3(0, false),
	_useDenominatorRadialCoeffs(false),
	_k4(0, false),
	_k5(0, false),
	_k6(0, false),
	_useTangentialDistortionModel(true),
	_p1(0, false),
	_p2(0, false),
	_useSkewDistortionModel(false),
	_B1(0, false),
	_B2(0, false)
{
	_c_x.value() = _imSize.width()/2;
	_c_y.value() = _imSize.height()/2;
	_f_pix.value() = _c_x.value();
}

floatParameter Camera::fLen() const
{
	return _f_pix;
}

float Camera::fLenY() const {
	return fLen().value()*pixelRatio().value();
}

void Camera::setFLen(const floatParameter &f_pix)
{
	if (!_f_pix.isApproximatlyEqual(f_pix, 1e-4)) {
		_f_pix = f_pix;
		emit FLenChanged(_f_pix);
		isChanged();
	}
}

floatParameter Camera::pixelRatio() const
{
	return _pix_ratio;
}

void Camera::setPixelRatio(const floatParameter &pix_ratio)
{
	if (!_pix_ratio.isApproximatlyEqual(pix_ratio, 1e-4)) {
		_pix_ratio = pix_ratio;
		emit pixelRatioChanged(_pix_ratio);
		isChanged();
	}
}

floatParameter Camera::opticalCenterX() const
{
	return _c_x;
}

void Camera::setOpticalCenterX(const floatParameter &c_x)
{
	if (!_c_x.isApproximatlyEqual(c_x, 1e-4)) {
		_c_x = c_x;
		emit opticalCenterXChanged(_c_x);
		isChanged();
	}
}

floatParameter Camera::opticalCenterY() const
{
	return _c_y;
}

void Camera::setOpticalCenterY(const floatParameter &c_y)
{
	if (!_c_y.isApproximatlyEqual(c_y, 1e-4)) {
		_c_y = c_y;
		emit opticalCenterYChanged(_c_y);
		isChanged();
	}
}

bool Camera::useRadialDistortionModel() const
{
	return _useRadialDistortionModel;
}

void Camera::setUseRadialDistortionModel(bool useRadialDistortionModel)
{
	if (_useRadialDistortionModel != useRadialDistortionModel) {
		_useRadialDistortionModel = useRadialDistortionModel;
		emit useRadialDistortionChanged(_useRadialDistortionModel);
		isChanged();
	}
}

floatParameter Camera::k1() const
{
	return _k1;
}

void Camera::setK1(const floatParameter &k1)
{
	if (!_k1.isApproximatlyEqual(k1, 1e-8)) {
		_k1 = k1;
		emit k1Changed(_k1);
		isChanged();
	}
}

floatParameter Camera::k2() const
{
	return _k2;
}

void Camera::setK2(const floatParameter &k2)
{
	if (!_k2.isApproximatlyEqual(k2, 1e-8)) {
		_k2 = k2;
		emit k2Changed(_k2);
		isChanged();
	}
}

floatParameter Camera::k3() const
{
	return _k3;
}

void Camera::setK3(const floatParameter &k3)
{
	if (!_k3.isApproximatlyEqual(k3, 1e-8)) {
		_k3 = k3;
		emit k3Changed(_k3);
		isChanged();
	}
}

bool Camera::useDenominatorRadialCoeffs() const
{
	return _useDenominatorRadialCoeffs;
}

void Camera::setUseDenominatorRadialCoeffs(bool useDenominatorRadialCoeffs)
{
	if (_useDenominatorRadialCoeffs != useDenominatorRadialCoeffs) {
		_useDenominatorRadialCoeffs = useDenominatorRadialCoeffs;
		emit useDenominatorRadialCoeffsChanged(_useDenominatorRadialCoeffs);
		isChanged();
	}
}

floatParameter Camera::k4() const
{
	return _k4;
}

void Camera::setK4(const floatParameter &k4)
{
	if (!_k4.isApproximatlyEqual(k4, 1e-8)) {
		_k4 = k4;
		emit k4Changed(_k4);
		isChanged();
	}
}

floatParameter Camera::k5() const
{
	return _k5;
}

void Camera::setK5(const floatParameter &k5)
{
	if (!_k5.isApproximatlyEqual(k5, 1e-8)) {
		_k5 = k5;
		emit k5Changed(_k5);
		isChanged();
	}
}

floatParameter Camera::k6() const
{
	return _k6;
}

void Camera::setK6(const floatParameter &k6)
{
	if (!_k6.isApproximatlyEqual(k6, 1e-8)) {
		_k6 = k6;
		emit k6Changed(_k6);
		isChanged();
	}
}

bool Camera::useTangentialDistortionModel() const
{
	return _useTangentialDistortionModel;
}

void Camera::setUseTangentialDistortionModel(bool useTangentialDistortionModel)
{
	if (_useTangentialDistortionModel != useTangentialDistortionModel) {
		_useTangentialDistortionModel = useTangentialDistortionModel;
		emit useTangentialDistortionChanged(_useTangentialDistortionModel);
		isChanged();
	}
}

floatParameter Camera::p1() const
{
	return _p1;
}

void Camera::setP1(const floatParameter &p1)
{
	if (!_p1.isApproximatlyEqual(p1, 1e-8)) {
		_p1 = p1;
		emit p1Changed(_p1);
		isChanged();
	}
}

floatParameter Camera::p2() const
{
	return _p2;
}

void Camera::setP2(const floatParameter &p2)
{
	if (!_p2.isApproximatlyEqual(p2, 1e-8)) {
		_p2 = p2;
		emit p2Changed(_p2);
		isChanged();
	}
}

bool Camera::useSkewDistortionModel() const
{
	return _useSkewDistortionModel;
}

void Camera::setUseSkewDistortionModel(bool useSkewDistortionModel)
{
	if (_useSkewDistortionModel != useSkewDistortionModel) {
		_useSkewDistortionModel = useSkewDistortionModel;
		emit useSkewDistortionChanged(_useSkewDistortionModel);
		isChanged();
	}
}

floatParameter Camera::B1() const
{
	return _B1;
}

void Camera::setB1(const floatParameter &B1)
{
	if (!_B1.isApproximatlyEqual(B1, 1e-6)) {
		_B1 = B1;
		emit B1Changed(_B1);
		isChanged();
	}
}

floatParameter Camera::B2() const
{
	return _B2;
}

void Camera::setB2(const floatParameter &B2)
{
	if (!_B2.isApproximatlyEqual(B2, 1e-6)) {
		_B2 = B2;
		emit B2Changed(_B2);
		isChanged();
	}
}

void Camera::clearOptimized() {
	_o_f_pix.clearIsSet();
	_o_c_x.clearIsSet();
	_o_c_y.clearIsSet();

	_o_k1.clearIsSet();
	_o_k2.clearIsSet();
	_o_k3.clearIsSet();

	_o_k4.clearIsSet();
	_o_k5.clearIsSet();
	_o_k6.clearIsSet();

	_o_p1.clearIsSet();
	_o_p2.clearIsSet();

	_o_B1.clearIsSet();
	_o_B2.clearIsSet();
}

bool Camera::hasOptimizedParameters() const {
	return _o_f_pix.isSet() or _o_c_x.isSet() or _o_c_y.isSet() or
			_o_k1.isSet() or _o_k2.isSet() or _o_k3.isSet() or _o_k4.isSet() or _o_k5.isSet() or _o_k6.isSet() or
			_o_p1.isSet() or _o_p2.isSet() or _o_B1.isSet() or _o_B2.isSet();
}

floatParameter Camera::optimizedFLen() const
{
	return _o_f_pix;
}

void Camera::setOptimizedFLen(const floatParameter &o_f_pix)
{
	if (!_o_f_pix.isApproximatlyEqual(o_f_pix, 1e-4)) {
		_o_f_pix = o_f_pix;
		emit optimizedFLenChanged(_o_f_pix);
		isChanged();
	}
}

floatParameter Camera::optimizedPixelRatio() const
{
	return _o_pix_ratio;
}

void Camera::setOptimizedPixelRatio(const floatParameter &o_pix_ratio)
{
	if (!_o_pix_ratio.isApproximatlyEqual(o_pix_ratio, 1e-4)) {
		_o_pix_ratio = o_pix_ratio;
		emit optimizedPixelRatioChanged(_o_pix_ratio);
		isChanged();
	}
}

floatParameter Camera::optimizedOpticalCenterX() const
{
	return _o_c_x;
}

void Camera::setOptimizedOpticalCenterX(const floatParameter &o_c_x)
{
	if (!_o_c_x.isApproximatlyEqual(o_c_x, 1e-4)) {
		_o_c_x = o_c_x;
		emit optimizedOpticalCenterXChanged(_o_c_x);
		isChanged();
	}
}

floatParameter Camera::optimizedOpticalCenterY() const
{
	return _o_c_y;
}

void Camera::setOptimizedOpticalCenterY(const floatParameter &o_c_y)
{
	if (!_o_c_y.isApproximatlyEqual(o_c_y, 1e-4)) {
		_o_c_y = o_c_y;
		emit optimizedOpticalCenterYChanged(_o_c_y);
		isChanged();
	}
}

floatParameter Camera::optimizedK1() const
{
	return _o_k1;
}

void Camera::setOptimizedK1(const floatParameter &o_k1)
{
	if (!_o_k1.isApproximatlyEqual(o_k1, 1e-8)) {
		_o_k1 = o_k1;
		emit optimizedK1Changed(_o_k1);
		isChanged();
	}
}

floatParameter Camera::optimizedK2() const
{
	return _o_k2;
}

void Camera::setOptimizedK2(const floatParameter &o_k2)
{
	if (!_o_k2.isApproximatlyEqual(o_k2, 1e-8)) {
		_o_k2 = o_k2;
		emit optimizedK2Changed(_o_k2);
		isChanged();
	}
}

floatParameter Camera::optimizedK3() const
{
	return _o_k3;
}

void Camera::setOptimizedK3(const floatParameter &o_k3)
{
	if (!_o_k3.isApproximatlyEqual(o_k3, 1e-8)) {
		_o_k3 = o_k3;
		emit optimizedK3Changed(_o_k3);
		isChanged();
	}
}

floatParameter Camera::optimizedK4() const
{
	return _o_k4;
}

void Camera::setOptimizedK4(const floatParameter &o_k4)
{
	if (!_o_k4.isApproximatlyEqual(o_k4, 1e-8)) {
		_o_k4 = o_k4;
		emit optimizedK4Changed(_o_k4);
		isChanged();
	}
}

floatParameter Camera::optimizedK5() const
{
	return _o_k5;
}

void Camera::setOptimizedK5(const floatParameter &o_k5)
{
	if (!_o_k5.isApproximatlyEqual(o_k5, 1e-8)) {
		_o_k5 = o_k5;
		emit optimizedK5Changed(_o_k5);
		isChanged();
	}
}

floatParameter Camera::optimizedK6() const
{
	return _o_k6;
}

void Camera::setOptimizedK6(const floatParameter &o_k6)
{
	if (!_o_k6.isApproximatlyEqual(o_k6, 1e-8)) {
		_o_k6 = o_k6;
		emit optimizedK6Changed(_o_k6);
		isChanged();
	}
}

floatParameter Camera::optimizedP1() const
{
	return _o_p1;
}

void Camera::setOptimizedP1(const floatParameter &o_p1)
{
	if (!_o_p1.isApproximatlyEqual(o_p1, 1e-8)) {
		_o_p1 = o_p1;
		emit optimizedP1Changed(_o_p1);
		isChanged();
	}
}

floatParameter Camera::optimizedP2() const
{
	return _o_p2;
}

void Camera::setOptimizedP2(const floatParameter &o_p2)
{
	if (!_o_p2.isApproximatlyEqual(o_p2, 1e-8)) {
		_o_p2 = o_p2;
		emit optimizedP2Changed(_o_p2);
		isChanged();
	}
}

floatParameter Camera::optimizedB1() const
{
	return _o_B1;
}

void Camera::setOptimizedB1(const floatParameter &o_B1)
{
	if (!_o_B1.isApproximatlyEqual(o_B1, 1e-6)) {
		_o_B1 = o_B1;
		emit optimizedB1Changed(_o_B1);
		isChanged();
	}
}

floatParameter Camera::optimizedB2() const
{
	return _o_B2;
}

void Camera::setOptimizedB2(const floatParameter &o_B2)
{
	if (!_o_B2.isApproximatlyEqual(o_B2, 1e-6)) {
		_o_B2 = o_B2;
		emit optimizedB2Changed(_o_B2);
		isChanged();
	}
}

QSize Camera::imSize() const
{
	return _imSize;
}

void Camera::setImSize(const QSize &imSize)
{
	if (_imSize != imSize) {
		_imSize = imSize;
		Q_EMIT imSizeChanged();
	}
}

int Camera::imWidth() const {
	return _imSize.width();
}
void Camera::setImWidth(int width) {
	if (_imSize.width() != width) {
		_imSize.setWidth(width);
		Q_EMIT imSizeChanged();
	}
}

int Camera::imHeight() const {
	return _imSize.height();
}
void Camera::setImHeight(int height) {
	if (_imSize.height() != height) {
		_imSize.setHeight(height);
		Q_EMIT imSizeChanged();
	}
}

QJsonObject Camera::encodeJson() const {

	QJsonObject obj;

	QJsonObject s;

	s.insert("w", _imSize.width());
	s.insert("h", _imSize.height());
	obj.insert("s", s);

	obj.insert("f", floatParameter::toJson(fLen()));
	obj.insert("pix_ratio", floatParameter::toJson(pixelRatio()));

	obj.insert("cx", floatParameter::toJson(opticalCenterX()));
	obj.insert("cy", floatParameter::toJson(opticalCenterY()));

	obj.insert("useRd", useRadialDistortionModel());

	obj.insert("k1", floatParameter::toJson(k1()));
	obj.insert("k2", floatParameter::toJson(k2()));
	obj.insert("k3", floatParameter::toJson(k3()));

	obj.insert("useDRd", useDenominatorRadialCoeffs());

	obj.insert("k4", floatParameter::toJson(k4()));
	obj.insert("k5", floatParameter::toJson(k5()));
	obj.insert("k6", floatParameter::toJson(k6()));

	obj.insert("useTd", useTangentialDistortionModel());

	obj.insert("p1", floatParameter::toJson(p1()));
	obj.insert("p2", floatParameter::toJson(p2()));

	obj.insert("useSd", useSkewDistortionModel());

	obj.insert("B1", floatParameter::toJson(B1()));
	obj.insert("B2", floatParameter::toJson(B2()));


	obj.insert("of", floatParameter::toJson(optimizedFLen()));
	obj.insert("opix_ratio", floatParameter::toJson(optimizedPixelRatio()));

	obj.insert("ocx", floatParameter::toJson(optimizedOpticalCenterX()));
	obj.insert("ocy", floatParameter::toJson(optimizedOpticalCenterY()));

	obj.insert("ok1", floatParameter::toJson(optimizedK1()));
	obj.insert("ok2", floatParameter::toJson(optimizedK2()));
	obj.insert("ok3", floatParameter::toJson(optimizedK3()));

	obj.insert("ok4", floatParameter::toJson(optimizedK4()));
	obj.insert("ok5", floatParameter::toJson(optimizedK5()));
	obj.insert("ok6", floatParameter::toJson(optimizedK6()));

	obj.insert("op1", floatParameter::toJson(optimizedP1()));
	obj.insert("op2", floatParameter::toJson(optimizedP2()));

	obj.insert("oB1", floatParameter::toJson(optimizedB1()));
	obj.insert("oB2", floatParameter::toJson(optimizedB2()));

	return obj;

}
void Camera::configureFromJson(QJsonObject const& obj) {

	if (obj.contains("s")) {
		QJsonObject s = obj.value("s").toObject();
		_imSize = QSize(s.value("w").toInt(), s.value("h").toInt());
	}

	if (obj.contains("f")) {
		_f_pix = floatParameter::fromJson(obj.value("f").toObject());
	}

	if (obj.contains("pix_ratio")) {
		_pix_ratio = floatParameter::fromJson(obj.value("pix_ratio").toObject());
	}

	if (obj.contains("cx")) {
		_c_x = floatParameter::fromJson(obj.value("cx").toObject());
	}

	if (obj.contains("cy")) {
		_c_y = floatParameter::fromJson(obj.value("cy").toObject());
	}

	if (obj.contains("useRd")) {
		_useRadialDistortionModel = obj.value("useRd").toBool();
	}

	if (obj.contains("k1")) {
		_k1 = floatParameter::fromJson(obj.value("k1").toObject());
	}

	if (obj.contains("k2")) {
		_k2 = floatParameter::fromJson(obj.value("k2").toObject());
	}

	if (obj.contains("k3")) {
		_k3 = floatParameter::fromJson(obj.value("k3").toObject());
	}

	if (obj.contains("useDRd")) {
		_useDenominatorRadialCoeffs = obj.value("useDRd").toBool();
	}

	if (obj.contains("k4")) {
		_k4 = floatParameter::fromJson(obj.value("k4").toObject());
	}

	if (obj.contains("k5")) {
		_k5 = floatParameter::fromJson(obj.value("k5").toObject());
	}

	if (obj.contains("k6")) {
		_k6 = floatParameter::fromJson(obj.value("k6").toObject());
	}

	if (obj.contains("useTd")) {
		_useTangentialDistortionModel = obj.value("useTd").toBool();
	}

	if (obj.contains("p1")) {
		_p1 = floatParameter::fromJson(obj.value("p1").toObject());
	}

	if (obj.contains("p2")) {
		_p2 = floatParameter::fromJson(obj.value("p2").toObject());
	}

	if (obj.contains("useSd")) {
		_useSkewDistortionModel = obj.value("useSd").toBool();
	}

	if (obj.contains("B1")) {
		_B1 = floatParameter::fromJson(obj.value("B1").toObject());
	}

	if (obj.contains("B2")) {
		_B2 = floatParameter::fromJson(obj.value("B2").toObject());
	}



	if (obj.contains("of")) {
		_o_f_pix = floatParameter::fromJson(obj.value("of").toObject());
	}

	if (obj.contains("opix_ratio")) {
		_o_pix_ratio = floatParameter::fromJson(obj.value("opix_ratio").toObject());
	}

	if (obj.contains("ocx")) {
		_o_c_x = floatParameter::fromJson(obj.value("ocx").toObject());
	}

	if (obj.contains("ocy")) {
		_o_c_y = floatParameter::fromJson(obj.value("ocy").toObject());
	}

	if (obj.contains("ok1")) {
		_o_k1 = floatParameter::fromJson(obj.value("ok1").toObject());
	}

	if (obj.contains("ok2")) {
		_o_k2 = floatParameter::fromJson(obj.value("ok2").toObject());
	}

	if (obj.contains("ok3")) {
		_o_k3 = floatParameter::fromJson(obj.value("ok3").toObject());
	}

	if (obj.contains("ok4")) {
		_o_k4 = floatParameter::fromJson(obj.value("ok4").toObject());
	}

	if (obj.contains("ok5")) {
		_o_k5 = floatParameter::fromJson(obj.value("ok5").toObject());
	}

	if (obj.contains("ok6")) {
		_o_k6 = floatParameter::fromJson(obj.value("ok6").toObject());
	}

	if (obj.contains("op1")) {
		_o_p1 = floatParameter::fromJson(obj.value("op1").toObject());
	}

	if (obj.contains("op2")) {
		_o_p2 = floatParameter::fromJson(obj.value("op2").toObject());
	}

	if (obj.contains("oB1")) {
		_o_B1 = floatParameter::fromJson(obj.value("oB1").toObject());
	}

	if (obj.contains("oB2")) {
		_o_B2 = floatParameter::fromJson(obj.value("oB2").toObject());
	}

}


void Camera::extendDataModel() {

	ItemDataModel::Category* p = _dataModel->addCategory(tr("Pinhole properties"));

	p->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal> (tr("Focal lenght [px]"),
																												&Camera::fLen,
																												&Camera::setFLen,
																												&Camera::FLenChanged);

	p->addCatProperty<int, Camera, false, ItemDataModel::ItemPropertyDescription::NoValueSignal> (tr("Image width"),
																								  &Camera::imWidth,
																								  &Camera::setImWidth,
																								  &Camera::imSizeChanged);

	p->addCatProperty<int, Camera, false, ItemDataModel::ItemPropertyDescription::NoValueSignal> (tr("Image height"),
																								  &Camera::imHeight,
																								  &Camera::setImHeight,
																								  &Camera::imSizeChanged);

	p->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal> (tr("Optical center X"),
																												&Camera::opticalCenterX,
																												&Camera::setOpticalCenterX,
																												&Camera::opticalCenterXChanged);

	p->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal> (tr("Optical center Y"),
																												&Camera::opticalCenterY,
																												&Camera::setOpticalCenterY,
																												&Camera::opticalCenterYChanged);

	ItemDataModel::Category* r = _dataModel->addCategory(tr("Radial distortion properties"));

	r->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>("k1", &Camera::k1, &Camera::setK1, &Camera::k1Changed);
	r->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>("k2", &Camera::k2, &Camera::setK2, &Camera::k2Changed);
	r->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>("k3", &Camera::k3, &Camera::setK3, &Camera::k3Changed);

	ItemDataModel::Category* t = _dataModel->addCategory(tr("Tangential distortion properties"));

	t->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>("p1", &Camera::p1, &Camera::setP1, &Camera::p1Changed);
	t->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>("p2", &Camera::p2, &Camera::setP2, &Camera::p2Changed);

	ItemDataModel::Category* s = _dataModel->addCategory(tr("Skew distortion properties"));

	s->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>("B1", &Camera::B1, &Camera::setB1, &Camera::B1Changed);
	s->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>("B2", &Camera::B2, &Camera::setB2, &Camera::B2Changed);



	ItemDataModel::Category* optCat = _dataModel->addCategory(tr("Optimizer properties"));

	optCat->addCatProperty<bool, DataBlock, false, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Fixed"),
																										  &DataBlock::isFixed,
																										  &DataBlock::setFixed,
																										  &DataBlock::isFixedChanged);


	ItemDataModel::Category* op = _dataModel->addCategory(tr("Optimized parameters"));

    op->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal> (tr("Focal length [px]"),
																												&Camera::optimizedFLen,
																												nullptr,
																												&Camera::optimizedFLenChanged);

	op->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal> (tr("Optical center X"),
																												&Camera::optimizedOpticalCenterX,
																												 nullptr,
																												&Camera::optimizedOpticalCenterXChanged);

	op->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal> (tr("Optical center Y"),
																												&Camera::optimizedOpticalCenterY,
																												 nullptr,
																												&Camera::optimizedOpticalCenterYChanged);

	//radial distortion
	op->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>("k1",
																												&Camera::optimizedK1,
																												nullptr,
																												&Camera::optimizedK1Changed);

	op->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>("k2",
																												&Camera::optimizedK2,
																												nullptr,
																												&Camera::optimizedK2Changed);

	op->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>("k3",
																												&Camera::optimizedK3,
																												nullptr,
																												&Camera::optimizedK3Changed);


	//tangential distortion
	op->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>("p1",
																												&Camera::optimizedP1,
																												nullptr,
																												&Camera::optimizedP1Changed);

	op->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>("p2",
																												&Camera::optimizedP2,
																												nullptr,
																												&Camera::optimizedP2Changed);


	//skew distortion
	op->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>("B1",
																												&Camera::optimizedB1,
																												nullptr,
																												&Camera::optimizedB1Changed);

	op->addCatProperty<floatParameter, Camera, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>("B2",
																												&Camera::optimizedB2,
																												nullptr,
																												&Camera::optimizedB2Changed);

}

CameraFactory::CameraFactory(QObject* parent) : DataBlockFactory(parent)
{

}

QString CameraFactory::TypeDescrName() const {
	return tr("Camera");
}
DataBlockFactory::FactorizableFlags CameraFactory::factorizable() const {
	return DataBlockFactory::RootDataBlock;
}
DataBlock* CameraFactory::factorizeDataBlock(Project *parent) const {
	return new Camera(parent);
}

QString CameraFactory::itemClassName() const {
	return cameraClassName();
}
QString CameraFactory::cameraClassName() {
	Camera c;
	return c.metaObject()->className();
}


} // namespace StereoVisionApp
