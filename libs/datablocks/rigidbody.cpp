#include "rigidbody.h"

#include "geometry/rotations.h"

namespace StereoVisionApp {

RigidBody::RigidBody(Project *parent) :
	DataBlock(parent)
{

}

RigidBody::RigidBody(DataBlock *parent) :
	DataBlock(parent)
{

}

floatParameter RigidBody::xCoord() const
{
	return _x;
}

void RigidBody::setXCoord(const floatParameter &x)
{
	if (!x.isApproximatlyEqual(_x, 1e-4)) {
		_x = x;
		emit xCoordChanged(x);
		isChanged();
	}
}

floatParameter RigidBody::yCoord() const
{
	return _y;
}

void RigidBody::setYCoord(const floatParameter &y)
{
	if (!y.isApproximatlyEqual(_y, 1e-4)) {
		_y = y;
		emit yCoordChanged(y);
		isChanged();
	}
}

floatParameter RigidBody::zCoord() const
{
	return _z;
}

void RigidBody::setZCoord(const floatParameter &z)
{
	if (!z.isApproximatlyEqual(_z, 1e-4)) {
		_z = z;
		emit zCoordChanged(z);
		isChanged();
	}
}

floatParameter RigidBody::xRot() const
{
	return _rx;
}

void RigidBody::setXRot(const floatParameter &rx)
{
	if (!rx.isApproximatlyEqual(_rx, 1e-4)) {
		_rx = rx;
		emit xRotChanged(rx);
		isChanged();
	}
}

floatParameter RigidBody::yRot() const
{
	return _ry;
}

void RigidBody::setYRot(const floatParameter &ry)
{
	if (!ry.isApproximatlyEqual(_ry, 1e-4)) {
		_ry = ry;
		emit yRotChanged(ry);
		isChanged();
	}
}

floatParameter RigidBody::zRot() const
{
	return _rz;
}

void RigidBody::setZRot(const floatParameter &rz)
{
	if (!rz.isApproximatlyEqual(_rz, 1e-4)) {
		_rz = rz;
		emit zRotChanged(rz);
		isChanged();
	}
}

floatParameterGroup<3> RigidBody::optPos() const {
	return _o_pos;
}
void RigidBody::setOptPos(floatParameterGroup<3> const& o_pos) {
	floatParameterGroup<3> t = o_pos;
	t.setIsSet();

	if (!t.isApproximatlyEqual(_o_pos, 1e-4)) {
		_o_pos = t;
		emit optPosChanged();
		isChanged();
	}
}
void RigidBody::clearOptPos() {
	if (_o_pos.isSet()) {
		_o_pos.clearIsSet();
		emit optPosChanged();
		isChanged();
	}
}

float RigidBody::optXCoord() const {
	return _o_pos.value(0);
}
void RigidBody::setOptXCoord(const float &x) {
	if (_o_pos.value(0) != x) {
		_o_pos.value(0) = x;
		emit optPosChanged();
        isChanged();
	}
}

float RigidBody::optYCoord() const {
	return _o_pos.value(1);
}
void RigidBody::setOptYCoord(const float &y) {
	if (_o_pos.value(1) != y) {
		_o_pos.value(1) = y;
		emit optPosChanged();
        isChanged();
	}
}

float RigidBody::optZCoord() const {
	return _o_pos.value(2);
}
void RigidBody::setOptZCoord(const float &z) {
	if (_o_pos.value(2) != z) {
		_o_pos.value(2) = z;
		emit optPosChanged();
        isChanged();
	}
}

floatParameterGroup<3> RigidBody::optRot() const {
	return _o_rot;
}
void RigidBody::setOptRot(floatParameterGroup<3> const& o_rot) {
	floatParameterGroup<3> t = o_rot;
	t.setIsSet();

	if (!t.isApproximatlyEqual(_o_rot, 1e-4)) {
		_o_rot = t;
		emit optRotChanged();
		isChanged();
	}
}
void RigidBody::clearOptRot() {
	if (_o_rot.isSet()) {
		_o_rot.clearIsSet();
		emit optRotChanged();
		isChanged();
	}
}

float RigidBody::optXRot() const {
	return _o_rot.value(0);
}
void RigidBody::setOptXRot(const float &rx) {
	if (_o_rot.value(0) != rx) {
		_o_rot.value(0) = rx;
		emit optRotChanged();
        isChanged();
	}
}

float RigidBody::optYRot() const {
	return _o_rot.value(1);
}
void RigidBody::setOptYRot(const float &ry){
	if (_o_rot.value(1) != ry) {
		_o_rot.value(1) = ry;
		emit optRotChanged();
        isChanged();
	}
}

float RigidBody::optZRot() const {
	return _o_rot.value(2);
}
void RigidBody::setOptZRot(const float &rz) {
	if (_o_rot.value(2) != rz) {
		_o_rot.value(2) = rz;
		emit optRotChanged();
        isChanged();
	}
}

std::optional<StereoVision::Geometry::AffineTransform<float> > RigidBody::getTransform() const {

	if (!_x.isSet() or !_y.isSet() or _z.isSet()) {
		return std::nullopt;
	}

	if (!_rx.isSet() or !_ry.isSet() or _rz.isSet()) {
		return std::nullopt;
	}

	Eigen::Vector3f t;
	t.x() = _x.value();
	t.y() = _y.value();
	t.z() = _z.value();

	Eigen::Vector3f r;
	r.x() = _rx.value();
	r.y() = _ry.value();
	r.z() = _rz.value();
	Eigen::Matrix3f R = StereoVision::Geometry::rodriguezFormula(r);

	return StereoVision::Geometry::AffineTransform(R, t);

}
std::optional<StereoVision::Geometry::AffineTransform<float> > RigidBody::getOptTransform() const {

	if (!_o_pos.isSet()) {
		return std::nullopt;
	}

	if (!_o_rot.isSet()) {
		return std::nullopt;
	}

	Eigen::Vector3f t;
	t.x() = _o_pos.value(0);
	t.y() = _o_pos.value(1);
	t.z() = _o_pos.value(2);

	Eigen::Vector3f r;
	r.x() = _o_rot.value(0);
	r.y() = _o_rot.value(1);
	r.z() = _o_rot.value(2);
	Eigen::Matrix3f R = StereoVision::Geometry::rodriguezFormula(r);

	return StereoVision::Geometry::AffineTransform(R, t);

}

void RigidBody::setTransform(const StereoVision::Geometry::AffineTransform<float> &transform) {

	floatParameter x = _x;
	floatParameter y = _y;
	floatParameter z = _z;

	x.value() = transform.t.x();
	y.value() = transform.t.y();
	z.value() = transform.t.z();

	setXCoord(x);
	setYCoord(y);
	setZCoord(z);

	Eigen::Vector3f r = StereoVision::Geometry::inverseRodriguezFormula(transform.R);

	floatParameter rx = _rx;
	floatParameter ry = _ry;
	floatParameter rz = _rz;

	rx.value() = r.x();
	ry.value() = r.y();
	rz.value() = r.z();

	setXRot(rx);
	setYRot(ry);
	setZRot(rz);
}

void RigidBody::setOptTransform(const StereoVision::Geometry::AffineTransform<float> &transform) {

	floatParameterGroup<3> o_pos = _o_pos;

	o_pos.clearUncertain();
	o_pos.value(0) = transform.t[0];
	o_pos.value(1) = transform.t[1];
	o_pos.value(2) = transform.t[2];

	setOptPos(o_pos);

	Eigen::Vector3f r = StereoVision::Geometry::inverseRodriguezFormula(transform.R);

	floatParameterGroup<3> o_rot = _o_rot;

	o_rot.clearUncertain();
	o_rot.value(0) = r[0];
	o_rot.value(1) = r[1];
	o_rot.value(2) = r[2];

	setOptRot(o_rot);

}

QJsonObject RigidBody::encodeJson() const {

	QJsonObject obj;

	obj.insert("x", floatParameter::toJson(xCoord()));
	obj.insert("y", floatParameter::toJson(yCoord()));
	obj.insert("z", floatParameter::toJson(zCoord()));

	obj.insert("rx", floatParameter::toJson(xRot()));
	obj.insert("ry", floatParameter::toJson(yRot()));
	obj.insert("rz", floatParameter::toJson(zRot()));

	obj.insert("op", floatParameterGroup<3>::toJson(optPos()));

	obj.insert("or", floatParameterGroup<3>::toJson(optRot()));

	return obj;
}
void RigidBody::configureFromJson(QJsonObject const& data) {

	if (data.contains("x")) {
		_x = floatParameter::fromJson(data.value("x").toObject());
	}
	if (data.contains("y")) {
		_y = floatParameter::fromJson(data.value("y").toObject());
	}
	if (data.contains("z")) {
		_z = floatParameter::fromJson(data.value("z").toObject());
	}

	if (data.contains("rx")) {
		_rx = floatParameter::fromJson(data.value("rx").toObject());
	}
	if (data.contains("ry")) {
		_ry = floatParameter::fromJson(data.value("ry").toObject());
	}
	if (data.contains("rz")) {
		_rz = floatParameter::fromJson(data.value("rz").toObject());
	}

	if (data.contains("op")) {
		_o_pos = floatParameterGroup<3>::fromJson(data.value("op").toObject());
	}

	if (data.contains("or")) {
		_o_rot = floatParameterGroup<3>::fromJson(data.value("or").toObject());
	}

}

} // namespace StereoVisionApp
