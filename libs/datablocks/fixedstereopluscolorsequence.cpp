#include "fixedstereopluscolorsequence.h"

#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>

namespace StereoVisionApp {

FixedStereoPlusColorSequence::FixedStereoPlusColorSequence(Project *parent) :
	DataBlock(parent)
{
	_imgList = new FixedStereoPlusColorSequenceImageList(this);
}

QString FixedStereoPlusColorSequence::baseFolder() const
{
	return _baseFolder;
}

QVector<FixedStereoPlusColorSequence::ImageTriplet> FixedStereoPlusColorSequence::imgsTriplets() const
{
	return _imgsTriplets;
}

void FixedStereoPlusColorSequence::setImgsLists(const QString &baseFolder, const QVector<ImageTriplet> &images)
{
	_baseFolder = baseFolder;
	_imgsTriplets = images;

	Q_EMIT imgListChanged();
}

QAbstractItemModel* FixedStereoPlusColorSequence::getImageList() const {
	return _imgList;
}

QJsonObject FixedStereoPlusColorSequence::encodeJson() const {

	QJsonObject obj;

	obj.insert("folder", _baseFolder);

	QJsonArray imgs;

	for (ImageTriplet const& trpl : _imgsTriplets) {
		QJsonObject triplet;

		triplet.insert("left", trpl.StereoLeftImgPath);
		triplet.insert("right", trpl.StereoRightImgPath);
		triplet.insert("rgb", trpl.ColorImagePath);

		imgs.push_back(triplet);
	}

	obj.insert("images", imgs);

	return obj;

}
void FixedStereoPlusColorSequence::configureFromJson(QJsonObject const& data) {

	QString folder = data.value("folder").toString("");

	QVector<ImageTriplet> triplets;

	QJsonValue cand = data.value("images");

	if (cand.isArray()) {
		QJsonArray imgs = cand.toArray();

		for (QJsonValue val : imgs) {
			if (val.isObject()) {

				QJsonObject obj = val.toObject();

				ImageTriplet triplet;

				if (!obj.contains("left") or !obj.contains("right") or !obj.contains("rgb")) {
					continue;
				}

				triplet.StereoLeftImgPath = obj.value("left").toString();
				triplet.StereoRightImgPath = obj.value("right").toString();
				triplet.ColorImagePath = obj.value("rgb").toString();

				triplets.push_back(triplet);
			}
		}
	}

	setImgsLists(folder, triplets);
}


FixedStereoPlusColorSequenceImageList::FixedStereoPlusColorSequenceImageList(FixedStereoPlusColorSequence* parent) :
	QAbstractTableModel(parent),
	_sequence(parent)
{
	connect(parent, &FixedStereoPlusColorSequence::imgListChanged, this, [this] () {
		beginResetModel();
		endResetModel();
	});
}

int FixedStereoPlusColorSequenceImageList::rowCount(const QModelIndex &parent) const {

	Q_UNUSED(parent);

	if (_sequence == nullptr) {
		return 0;
	}

	return _sequence->imgsTriplets().size();
}

int FixedStereoPlusColorSequenceImageList::columnCount(const QModelIndex &parent) const {
	Q_UNUSED(parent);
	return 3;
}

QVariant FixedStereoPlusColorSequenceImageList::data(const QModelIndex &index, int role) const {

	if (index.parent() != QModelIndex()) {
		return QVariant();
	}

	if (_sequence == nullptr) {
		return QVariant();
	}

	switch (role) {
	case Qt::DisplayRole:
		switch (index.column()) {
		case 0:
			return _sequence->imgsTriplets().at(index.row()).StereoLeftImgPath;
		case 1:
			return _sequence->imgsTriplets().at(index.row()).StereoRightImgPath;
		case 2:
			return _sequence->imgsTriplets().at(index.row()).ColorImagePath;
		}
	}

	return QVariant();
}

QVariant FixedStereoPlusColorSequenceImageList::headerData(int section, Qt::Orientation orientation, int role) const {

	if (orientation == Qt::Vertical) {
		return QVariant();
	}

	if (role != Qt::DisplayRole) {
		return QVariant();
	}

	switch (section) {
	case 0:
		return tr("Left image");
	case 1:
		return tr("Right image");
	case 2:
		return tr("Color image");
	}

	return QVariant();
}


FixedStereoPlusColorSequenceFactory::FixedStereoPlusColorSequenceFactory(QObject* parent) :
	DataBlockFactory(parent)
{

}

QString FixedStereoPlusColorSequenceFactory::TypeDescrName() const {
	return tr("Stereo plus color sequence");
}

DataBlockFactory::FactorizableFlags FixedStereoPlusColorSequenceFactory::factorizable() const {
	return DataBlockFactory::RootDataBlock;
}
DataBlock* FixedStereoPlusColorSequenceFactory::factorizeDataBlock(Project *parent) const {
	return new FixedStereoPlusColorSequence(parent);
}

QString FixedStereoPlusColorSequenceFactory::itemClassName() const {
	return FixedStereoPlusColorSequence::staticMetaObject.className();
}

} // namespace StereoVisionApp
