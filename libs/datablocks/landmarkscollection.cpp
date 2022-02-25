#include "landmarkscollection.h"

#include "./project.h"

#include <QJsonObject>
#include "landmark.h"

namespace StereoVisionApp {

LandmarksCollection::LandmarksCollection(DataBlock *parent) :
	DataBlock(parent)
{
	connect(this, &LandmarksCollection::attachedLandmarkChanged,
			this, &LandmarksCollection::attachedLandmarkNameChanged);
}

qint64 LandmarksCollection::getNthLandmarkId(int idx) const {

	if (idx >= 0 and idx < collectionSize()) {
		if (_landmarks.size() >= idx) {
			scaleLandmarksVectors();
		}
		return _landmarks[idx];
	}
	return -1;
}

void LandmarksCollection::setNthLandmarkId(int idx, qint64 id) {

	qint64 previous_id = getNthLandmarkId(idx);

	if (id < 0 and previous_id >= 0) {
		removeRefered({previous_id});
		setNthLandmarkIdImpl(idx, -1);
		emit attachedLandmarkChanged(idx, -1);
		return;
	} else if (id < 0) {
		return;
	}

	qint64 fid = id;

	for (int i = 0; i < collectionSize(); i++) {
		qint64 oid = getNthLandmarkId(i);

		if (oid >= 0 and oid == fid) {
			fid = -1;
			break;
		}
	}

	if (previous_id != fid) {
		if (previous_id >= 0) {
			removeRefered({previous_id});
		}
		setNthLandmarkIdImpl(idx, fid);
		if (fid >= 0) {
			addRefered({fid});
		}
		emit attachedLandmarkChanged(idx, fid);
		isChanged();
	}

}


Landmark* LandmarksCollection::getNthLandmark(int idx) const {
	qint64 id = getNthLandmarkId(idx);
	Project* p = getProject();

	if (p != nullptr) {
		Landmark* lm = qobject_cast<Landmark*>(p->getById(id));
		if (lm != nullptr) {
			return lm;
		}
	}

	return nullptr;
}
QString LandmarksCollection::getNthLandmarkName(int idx) const {
	Landmark* lm = getNthLandmark(idx);
	if (lm != nullptr) {
		return lm->objectName();
	}
	return "Unset Landmark";
}

void LandmarksCollection::scaleLandmarksVectors() const {

	int pSize = _landmarks.size();

	if (_landmarks.size() != collectionSize()) {
		_landmarks.resize(collectionSize());
		if (pSize < collectionSize()) {
			for (int i = pSize; i < collectionSize(); i++) {
				_landmarks[i] = -1;
			}
		}
	}
}

void LandmarksCollection::setNthLandmarkIdImpl(int idx, qint64 id) {

	if (idx >= 0 and idx < collectionSize()) {
		scaleLandmarksVectors();
		_landmarks[idx] = id;
	}
}

QJsonObject LandmarksCollection::encodeJson() const {

	QJsonObject obj;

	for (int i = 0; i < collectionSize(); i++) {
		obj.insert(QString("landmark%1").arg(i+1), getNthLandmarkId(i));
	}

	return obj;
}
void LandmarksCollection::configureFromJson(QJsonObject const& data) {

	for (int i = 0; i < collectionSize(); i++) {
		QString key = QString("landmark%1").arg(i+1);
		if (data.contains(key)) {
			setNthLandmarkIdImpl(i, data.value(key).toInt());
		}
	}
}


LandmarksTriplet::LandmarksTriplet(DataBlock* parent) :
	LandmarksCollection(parent)
{
	connect(this, &LandmarksCollection::attachedLandmarkNameChanged, [this] (int idx) {
		if (idx == 0) {
			emit attachedLandmark1NameChanged();
		} else if (idx == 1) {
			emit attachedLandmark2NameChanged();
		}else if (idx == 2) {
			emit attachedLandmark3NameChanged();
		}
	});
}

int LandmarksTriplet::collectionSize() const {
	return 3;
}

QString LandmarksTriplet::nameLandmark1() const {
	return getNthLandmarkName(0);
}
QString LandmarksTriplet::nameLandmark2() const {
	return getNthLandmarkName(1);
}
QString LandmarksTriplet::nameLandmark3() const {
	return getNthLandmarkName(2);
}


LandmarksPair::LandmarksPair(DataBlock* parent) :
	LandmarksCollection(parent)
{
	connect(this, &LandmarksCollection::attachedLandmarkNameChanged, [this] (int idx) {
		if (idx == 0) {
			emit attachedLandmark1NameChanged();
		} else if (idx == 1) {
			emit attachedLandmark2NameChanged();
		}
	});
}

int LandmarksPair::collectionSize() const {
	return 2;
}

QString LandmarksPair::nameLandmark1() const {
	return getNthLandmarkName(0);
}
QString LandmarksPair::nameLandmark2() const {
	return getNthLandmarkName(1);
}

} // namespace StereoVisionApp
