#include "point3d.h"

namespace StereoVisionApp {

Point3D::Point3D(Project *parent) : DataBlock(parent)
{

}

Point3D::Point3D(DataBlock *parent) : DataBlock(parent)
{

}

floatParameter Point3D::xCoord() const
{
	return _x;
}

void Point3D::setXCoord(const floatParameter &x)
{
	if (!x.isApproximatlyEqual(_x, 1e-4)) {
		_x = x;
		emit xCoordChanged(x);
		isChanged();
	}
}

floatParameter Point3D::yCoord() const
{
	return _y;
}

void Point3D::setYCoord(const floatParameter &y)
{
	if (!y.isApproximatlyEqual(_y, 1e-4)) {
		_y = y;
		emit yCoordChanged(y);
		isChanged();
	}
}

floatParameter Point3D::zCoord() const
{
	return _z;
}

void Point3D::setZCoord(const floatParameter &z)
{
	if (!z.isApproximatlyEqual(_z, 1e-4)) {
		_z = z;
		emit zCoordChanged(z);
		isChanged();
	}
}

floatParameterGroup<3> Point3D::optPos() const {
	return _o_pos;
}
void Point3D::setOptPos(floatParameterGroup<3> const& o_pos) {
	floatParameterGroup<3> t = o_pos;
	t.setIsSet();

    if (t != _o_pos) {
		_o_pos = t;
		emit optPosChanged();
		isChanged();
	}
}
void Point3D::clearOptPos() {
	if (_o_pos.isSet()) {
		_o_pos.clearIsSet();
		emit optPosChanged();
		isChanged();
	}
}

float Point3D::optXCoord() const {
	return _o_pos.value(0);
}
void Point3D::setOptXCoord(const float &x) {
	if (_o_pos.value(0) != x) {
		_o_pos.value(0) = x;
		emit optPosChanged();
	}
}

float Point3D::optYCoord() const {
	return _o_pos.value(1);
}
void Point3D::setOptYCoord(const float &y) {
	if (_o_pos.value(1) != y) {
		_o_pos.value(1) = y;
		emit optPosChanged();
	}
}

float Point3D::optZCoord() const {
	return _o_pos.value(2);
}
void Point3D::setOptZCoord(const float &z) {
	if (_o_pos.value(2) != z) {
		_o_pos.value(2) = z;
		emit optPosChanged();
	}
}

std::optional<Eigen::Vector3f> Point3D::getPointVec() const {

	if (!_x.isSet() or !_y.isSet() or !_z.isSet()) {
		return std::nullopt;
	}

	Eigen::Vector3f t;
	t.x() = _x.value();
	t.y() = _y.value();
	t.z() = _z.value();

	return t;
}
std::optional<Eigen::Vector3f> Point3D::getOptPointVec() const {

	if (!_o_pos.isSet()) {
		return std::nullopt;
	}

	Eigen::Vector3f t;
	t.x() = _o_pos.value(0);
	t.y() = _o_pos.value(1);
	t.z() = _o_pos.value(2);

	return t;
}

void Point3D::clearOptimized() {
	clearOptPos();
}

bool Point3D::hasOptimizedParameters() const {
	return _o_pos.isSet();
}

QJsonObject Point3D::encodeJson() const {

	QJsonObject obj;

	obj.insert("x", floatParameter::toJson(xCoord()));
	obj.insert("y", floatParameter::toJson(yCoord()));
	obj.insert("z", floatParameter::toJson(zCoord()));

	obj.insert("op", floatParameterGroup<3>::toJson(optPos()));

	return obj;
}

void Point3D::configureFromJson(QJsonObject const& data) {

	if (data.contains("x")) {
		_x = floatParameter::fromJson(data.value("x").toObject());
	}
	if (data.contains("y")) {
		_y = floatParameter::fromJson(data.value("y").toObject());
	}
	if (data.contains("y")) {
		_z = floatParameter::fromJson(data.value("z").toObject());
	}

	if (data.contains("op")) {
		_o_pos = floatParameterGroup<3>::fromJson(data.value("op").toObject());
	}

}

} // namespace StereoVisionApp
