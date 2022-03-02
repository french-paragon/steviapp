#include "landmark.h"
#include "image.h"

#include "./itemdatamodel.h"

#include <QSet>

namespace StereoVisionApp {

Landmark::Landmark(Project *parent) : DataBlock(parent)
{
	extendDataModel();
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

floatParameterGroup<3> Landmark::optPos() const {
	return _o_pos;
}
void Landmark::setOptPos(floatParameterGroup<3> const& o_pos) {
	floatParameterGroup<3> t = o_pos;
	t.setIsSet();

	if (!t.isApproximatlyEqual(_o_pos, 1e-4)) {
		_o_pos = t;
		emit optPosChanged();
		isChanged();
	}
}
void Landmark::clearOptPos() {
	if (_o_pos.isSet()) {
		_o_pos.clearIsSet();
		emit optPosChanged();
		isChanged();
	}
}


QVector<qint64> Landmark::getImagesRefering() const {

	QVector<qint64> referingImgsId;
	referingImgsId.reserve(_referers.size());

	for (QVector<qint64> const& path : _referers) {
		qint64 id = path.first();
		Image* im = qobject_cast<Image*>(getProject()->getById(id));

		if (im != nullptr) {
			referingImgsId.push_back(id);
		}
	}

	return referingImgsId;

}
int Landmark::countImagesRefering(const QSet<qint64> &excluded) const {

	if (!isInProject()) {
		return 0;
	}

	QSet<qint64> referingImgsId;
	for (QVector<qint64> const& path : _referers) {
		qint64 id = path.first();
		Image* im = qobject_cast<Image*>(getProject()->getById(id));

		if (im != nullptr) {
			referingImgsId.insert(id);
		}
	}

	for (qint64 id : excluded) {
		referingImgsId.remove(id);
	}

	return referingImgsId.count();

}
int Landmark::countImagesRefering(QVector<qint64> const& excluded) const {

	QSet<qint64> s(excluded.begin(), excluded.end());
	return countImagesRefering(s);

}

int Landmark::countImagesReferingInList(QSet<qint64> const& included) const {

	return getViewingImgInList(included).size();
}
int Landmark::countImagesReferingInList(QVector<qint64> const& included) const {

	QSet<qint64> s(included.begin(), included.end());
	return countImagesReferingInList(s);
}

QSet<qint64> Landmark::getViewingImgInList(QSet<qint64> const& included) const {

	if (!isInProject()) {
		return {};
	}

	QSet<qint64> referingImgsId;
	for (QVector<qint64> const& path : _referers) {
		qint64 id = path.first();

		if (!included.contains(id)) {
			continue;
		}

		Image* im = qobject_cast<Image*>(getProject()->getById(id));

		if (im != nullptr) {
			referingImgsId.insert(id);
		}
	}

	return referingImgsId;

}

void Landmark::clearOptimized() {
	clearOptPos();
}

bool Landmark::hasOptimizedParameters() const {
	return _o_pos.isSet();
}

QJsonObject Landmark::encodeJson() const {

	QJsonObject obj;

	obj.insert("x", floatParameter::toJson(xCoord()));
	obj.insert("y", floatParameter::toJson(yCoord()));
	obj.insert("z", floatParameter::toJson(zCoord()));

	obj.insert("op", floatParameterGroup<3>::toJson(optPos()));

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

	if (data.contains("op")) {
		_o_pos = floatParameterGroup<3>::fromJson(data.value("op").toObject());
	}

}

void Landmark::extendDataModel() {

	ItemDataModel::Category* g = _dataModel->addCategory(tr("Geometric properties"));

	g->addCatProperty<floatParameter, Landmark, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("X pos"),
																												 &Landmark::xCoord,
																												 &Landmark::setXCoord,
																												 &Landmark::xCoordChanged);

	g->addCatProperty<floatParameter, Landmark, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Y pos"),
																												 &Landmark::yCoord,
																												 &Landmark::setYCoord,
																												 &Landmark::yCoordChanged);

	g->addCatProperty<floatParameter, Landmark, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Z pos"),
																												 &Landmark::zCoord,
																												 &Landmark::setZCoord,
																												 &Landmark::zCoordChanged);

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
