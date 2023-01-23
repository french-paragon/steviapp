#include "fixedstereopluscolorsequence.h"

#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>

namespace StereoVisionApp {

FixedStereoPlusColorSequence::FixedStereoPlusColorSequence(Project *parent) :
	ExportableStereoSequence(parent),
	_leftViewId(-1),
	_rgbViewId(-1),
	_rightViewId(-1)
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

void FixedStereoPlusColorSequence::setLeftViewId(qint64 id) {

	if (id < 0 and _leftViewId >= 0) {
		removeRefered({_leftViewId});
		_leftViewId = -1;
		emit leftViewIdChanged(-1);
		return;
	} else if (id < 0) {
		return;
	}

	if (id == _leftViewId) {
		return;
	}

	if (_leftViewId >= 0) {
		removeRefered({_leftViewId});
	}

	_leftViewId = id;

	if (_leftViewId >= 0) {
		addRefered({_leftViewId});
	}

	emit leftViewIdChanged(_leftViewId);

}

void FixedStereoPlusColorSequence::setRgbViewId(qint64 id) {

	if (id < 0 and _rgbViewId >= 0) {
		removeRefered({_rgbViewId});
		_rgbViewId = -1;
		emit rgbViewIdChanged(-1);
		return;
	} else if (id < 0) {
		return;
	}

	if (id == _rgbViewId) {
		return;
	}

	if (_rgbViewId >= 0) {
		removeRefered({_rgbViewId});
	}

	_rgbViewId = id;

	if (_rgbViewId >= 0) {
		addRefered({_rgbViewId});
	}

	emit rgbViewIdChanged(_rgbViewId);

}

void FixedStereoPlusColorSequence::setRightViewId(qint64 id) {

	if (id < 0 and _rightViewId >= 0) {
		removeRefered({_rightViewId});
		_rightViewId = -1;
		emit rightViewIdChanged(-1);
		return;
	} else if (id < 0) {
		return;
	}

	if (id == _rightViewId) {
		return;
	}

	if (_rightViewId >= 0) {
		removeRefered({_rightViewId});
	}

	_rightViewId = id;

	if (_rightViewId >= 0) {
		addRefered({_rightViewId});
	}

	emit rightViewIdChanged(_rightViewId);

}

qint64 FixedStereoPlusColorSequence::leftViewId() const {
	return _leftViewId;
}
qint64 FixedStereoPlusColorSequence::rgbViewId() const {
	return _rgbViewId;
}
qint64 FixedStereoPlusColorSequence::rightViewId() const {
	return _rightViewId;
}

QJsonObject FixedStereoPlusColorSequence::encodeJson() const {

	QJsonObject obj;

	obj.insert("folder", _baseFolder);

	obj.insert("imgLeftId", _leftViewId);
	obj.insert("imgRgbId", _rgbViewId);
	obj.insert("imgRightId", _rightViewId);


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

	if (data.contains("imgLeftId")) {
		_leftViewId = data.value("imgLeftId").toInt(-1);
	}

	if (data.contains("imgRgbId")) {
		_rgbViewId = data.value("imgRgbId").toInt(-1);
	}

	if (data.contains("imgRightId")) {
		_rightViewId = data.value("imgRightId").toInt(-1);
	}
}

void FixedStereoPlusColorSequence::referedCleared(QVector<qint64> const& referedId) {

	if (referedId.size() != 1) {
		return;
	}

	qint64 id = referedId[0];

	if (id == _leftViewId) {
		_leftViewId = -1;
		Q_EMIT leftViewIdChanged(-1);
	}

	if (id == _rgbViewId) {
		_rgbViewId = -1;
		Q_EMIT rgbViewIdChanged(-1);
	}

	if (id == _rightViewId) {
		_rightViewId = -1;
		Q_EMIT rightViewIdChanged(-1);
	}
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
