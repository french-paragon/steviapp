#include "landmark.h"
#include "image.h"
#include "localcoordinatesystem.h"

#include "./itemdatamodel.h"

#include <QSet>

namespace StereoVisionApp {

Landmark::Landmark(Project *parent) : Point3D(parent)
{
	extendDataModel();
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

int Landmark::countLocalCoordinateSystemsRefering(QSet<qint64> const& excluded) const {

	if (!isInProject()) {
		return 0;
	}

	QSet<qint64> referingLcsId;
	for (QVector<qint64> const& path : _referers) {
		qint64 id = path.first();
		LocalCoordinateSystem* lcs =getProject()->getDataBlock<LocalCoordinateSystem>(id);

		if (lcs == nullptr) {
			continue;
		}

		LandmarkLocalCoordinates* lmlc = lcs->getLandmarkLocalCoordinatesByLandmarkId(internalId());

		if (lmlc == nullptr) {
			continue;
		}

		if (!lmlc->xCoord().isSet() and !lmlc->yCoord().isSet() and !lmlc->zCoord().isSet()) {
			continue;
		}

		referingLcsId.insert(id);
	}

	for (qint64 id : excluded) {
		referingLcsId.remove(id);
	}

	return referingLcsId.count();

}
int Landmark::countLocalCoordinateSystemsRefering(QVector<qint64> const& excluded) const {

	QSet<qint64> s(excluded.begin(), excluded.end());
	return countLocalCoordinateSystemsRefering(s);

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

void Landmark::extendDataModel() {

	ItemDataModel::Category* g = _dataModel->addCategory(tr("Geometric properties"));

	g->addCatProperty<floatParameter, Point3D, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("X pos"),
																												 &Point3D::xCoord,
																												 &Point3D::setXCoord,
																												 &Point3D::xCoordChanged);

	g->addCatProperty<floatParameter, Point3D, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Y pos"),
																												 &Point3D::yCoord,
																												 &Point3D::setYCoord,
																												 &Point3D::yCoordChanged);

	g->addCatProperty<floatParameter, Point3D, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Z pos"),
																												 &Point3D::zCoord,
																												 &Point3D::setZCoord,
																												 &Point3D::zCoordChanged);



	ItemDataModel::Category* optCat = _dataModel->addCategory(tr("Optimizer properties"));

	optCat->addCatProperty<bool, DataBlock, false, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Enabled"),
																										  &DataBlock::isEnabled,
																										  &DataBlock::setEnabled,
																										  &DataBlock::isEnabledChanged);

	optCat->addCatProperty<bool, DataBlock, false, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Fixed"),
																										  &DataBlock::isFixed,
																										  &DataBlock::setFixed,
																										  &DataBlock::isFixedChanged);


	ItemDataModel::Category* og = _dataModel->addCategory(tr("Optimized geometry"));

	//Position
	og->addCatProperty<float, Point3D, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("X pos"),
																									 &Point3D::optXCoord,
																									 &Point3D::setOptXCoord,
																									 &Point3D::optPosChanged);

	og->addCatProperty<float, Point3D, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Y pos"),
																									 &Point3D::optYCoord,
																									 &Point3D::setOptYCoord,
																									 &Point3D::optPosChanged);

	og->addCatProperty<float, Point3D, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Z pos"),
																									 &Point3D::optZCoord,
																									 &Point3D::setOptZCoord,
																									 &Point3D::optPosChanged);
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
