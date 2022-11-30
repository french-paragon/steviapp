#include "fixedcolorstereosequence.h"

#include <QJsonObject>
#include <QJsonArray>

namespace StereoVisionApp {

FixedColorStereoSequence::FixedColorStereoSequence(Project *parent) :
	DataBlock(parent)
{
	_imgList = new FixedColorStereoSequenceImageList(this);
}


QString FixedColorStereoSequence::baseFolder() const {
	return _baseFolder;
}
QVector<FixedColorStereoSequence::ImagePair> FixedColorStereoSequence::imgsPairs() const {
	return _imgsPairs;
}

void FixedColorStereoSequence::setImgsLists(const QString &baseFolder, QVector<ImagePair> const& images) {
	_baseFolder = baseFolder;
	_imgsPairs = images;

	Q_EMIT imgListChanged();
}

QAbstractItemModel* FixedColorStereoSequence::getImageList() const {

	return _imgList;
}

QJsonObject FixedColorStereoSequence::encodeJson() const {

	QJsonObject obj;

	obj.insert("folder", _baseFolder);

	QJsonArray imgs;

	for (ImagePair const& pr : _imgsPairs) {
		QJsonObject pair;

		pair.insert("left", pr.StereoLeftImgPath);
		pair.insert("right", pr.StereoRightImgPath);

		imgs.push_back(pair);
	}

	obj.insert("images", imgs);

	return obj;

}
void FixedColorStereoSequence::configureFromJson(QJsonObject const& data) {

	QString folder = data.value("folder").toString("");

	QVector<ImagePair> pairs;

	QJsonValue cand = data.value("images");

	if (cand.isArray()) {
		QJsonArray imgs = cand.toArray();

		for (QJsonValue val : imgs) {
			if (val.isObject()) {

				QJsonObject obj = val.toObject();

				ImagePair pair;

				if (!obj.contains("left") or !obj.contains("right")) {
					continue;
				}

				pair.StereoLeftImgPath = obj.value("left").toString();
				pair.StereoRightImgPath = obj.value("right").toString();

				pairs.push_back(pair);
			}
		}
	}

	setImgsLists(folder, pairs);

}


FixedColorStereoSequenceImageList::FixedColorStereoSequenceImageList(FixedColorStereoSequence* parent) :
	QAbstractTableModel(parent),
	_sequence(parent)
{
	connect(parent, &FixedColorStereoSequence::imgListChanged, this, [this] () {
		beginResetModel();
		endResetModel();
	});
}

int FixedColorStereoSequenceImageList::rowCount(const QModelIndex &parent) const {

	Q_UNUSED(parent);

	if (_sequence == nullptr) {
		return 0;
	}

	return _sequence->imgsPairs().size();
}

int FixedColorStereoSequenceImageList::columnCount(const QModelIndex &parent) const {
	Q_UNUSED(parent);
	return 3;
}

QVariant FixedColorStereoSequenceImageList::data(const QModelIndex &index, int role) const {

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
			return _sequence->imgsPairs().at(index.row()).StereoLeftImgPath;
		case 1:
			return _sequence->imgsPairs().at(index.row()).StereoRightImgPath;
		}
	}

	return QVariant();
}

QVariant FixedColorStereoSequenceImageList::headerData(int section, Qt::Orientation orientation, int role) const {

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
	}

	return QVariant();
}


FixedColorStereoSequenceFactory::FixedColorStereoSequenceFactory(QObject* parent) :
	DataBlockFactory(parent)
{

}

QString FixedColorStereoSequenceFactory::TypeDescrName() const {
	return tr("Stereo sequence");
}
DataBlockFactory::FactorizableFlags FixedColorStereoSequenceFactory::factorizable() const {
	return DataBlockFactory::RootDataBlock;
}
DataBlock* FixedColorStereoSequenceFactory::factorizeDataBlock(Project *parent) const {
	return new FixedColorStereoSequence(parent);
}

QString FixedColorStereoSequenceFactory::itemClassName() const {
	return FixedColorStereoSequence::staticMetaObject.className();
}

} // namespace StereoVisionApp
