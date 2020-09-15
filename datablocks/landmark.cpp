#include "landmark.h"

namespace StereoVisionApp {

Landmark::Landmark(Project *parent) : DataBlock(parent)
{

}

floatParameter Landmark::xCoord() const
{
	return _x;
}

void Landmark::setXCoord(const floatParameter &x)
{
	if (!x.isApproximatlyEqual(_x, 1e-4)) {
		_x = x;
		emit xCoordChanged(x);
		isChanged();
	}
}

floatParameter Landmark::yCoord() const
{
	return _y;
}

void Landmark::setYCoord(const floatParameter &y)
{
	if (!y.isApproximatlyEqual(_y, 1e-4)) {
		_y = y;
		emit yCoordChanged(y);
		isChanged();
	}
}

floatParameter Landmark::zCoord() const
{
	return _z;
}

void Landmark::setZCoord(const floatParameter &z)
{
	if (!z.isApproximatlyEqual(_z, 1e-4)) {
		_z = z;
		emit zCoordChanged(z);
		isChanged();
	}
}

floatParameter Landmark::optimizedX() const
{
	return _o_x;
}

void Landmark::setOptimisedX(const floatParameter &o_x)
{
	floatParameter t = o_x;
	t.setIsSet();
	t.setUncertainty();

	if (!t.isApproximatlyEqual(_o_x, 1e-4)) {
		_o_x = t;
		emit optXCoordChanged(t);
		isChanged();
	}
}
void Landmark::clearOptimisedX() {
	if (_o_x.isSet()) {
		_o_x.clearIsSet();
		emit optXCoordChanged(_o_x);
		isChanged();
	}
}

floatParameter Landmark::optimizedY() const
{
	return _o_y;
}

void Landmark::setOptimisedY(const floatParameter &o_y)
{
	floatParameter t = o_y;
	t.setIsSet();
	t.setUncertainty();

	if (!t.isApproximatlyEqual(_o_y, 1e-4)) {
		_o_y = t;
		emit optXCoordChanged(t);
		isChanged();
	}
}
void Landmark::clearOptimisedY() {
	if (_o_y.isSet()) {
		_o_y.clearIsSet();
		emit optXCoordChanged(_o_y);
		isChanged();
	}
}

floatParameter Landmark::optimizedZ() const
{
	return _o_z;
}

void Landmark::setOptimisedZ(const floatParameter &o_z)
{
	floatParameter t = o_z;
	t.setIsSet();
	t.setUncertainty();

	if (!t.isApproximatlyEqual(_o_z, 1e-4)) {
		_o_z = t;
		emit optXCoordChanged(t);
		isChanged();
	}
}
void Landmark::clearOptimisedZ() {
	if (_o_z.isSet()) {
		_o_z.clearIsSet();
		emit optXCoordChanged(_o_z);
		isChanged();
	}
}

QJsonObject Landmark::encodeJson() const {

	QJsonObject obj;

	obj.insert("x", floatParameter::toJson(xCoord()));
	obj.insert("y", floatParameter::toJson(yCoord()));
	obj.insert("z", floatParameter::toJson(yCoord()));

	obj.insert("ox", floatParameter::toJson(optimizedX()));
	obj.insert("oy", floatParameter::toJson(optimizedY()));
	obj.insert("oz", floatParameter::toJson(optimizedZ()));

	return obj;
}

void Landmark::configureFromJson(QJsonObject const& data) {

	if (data.contains("x")) {
		_x = floatParameter::fromJson(data.value("x").toObject());
	}
	if (data.contains("y")) {
		_y = floatParameter::fromJson(data.value("y").toObject());
	}
	if (data.contains("y")) {
		_z = floatParameter::fromJson(data.value("z").toObject());
	}

	if (data.contains("ox")) {
		_o_x = floatParameter::fromJson(data.value("ox").toObject());
	}
	if (data.contains("oy")) {
		_o_y = floatParameter::fromJson(data.value("oy").toObject());
	}
	if (data.contains("oy")) {
		_o_z = floatParameter::fromJson(data.value("oz").toObject());
	}

}

LandmarkFactory::LandmarkFactory(QObject* parent) : DataBlockFactory(parent)
{

}

QString LandmarkFactory::TypeDescrName() const {
	return tr("Landmark");
}
DataBlockFactory::FactorizableFlags LandmarkFactory::factorizable() const {
	return DataBlockFactory::RootDataBlock;
}
DataBlock* LandmarkFactory::factorizeDataBlock(Project *parent) const {
	return new Landmark(parent);
}

QString LandmarkFactory::itemClassName() const {
	return landmarkClassName();
}

QString LandmarkFactory::landmarkClassName() {
	Landmark l;
	return l.metaObject()->className();
}

} // namespace StereoVisionApp
