#ifndef STEREOVISIONAPP_LANDMARKSCOLLECTION_H
#define STEREOVISIONAPP_LANDMARKSCOLLECTION_H

#include "./project.h"

namespace StereoVisionApp {

class Landmark;

class LandmarksCollection : public DataBlock
{
	Q_OBJECT
public:
	LandmarksCollection(DataBlock* parent);

	virtual int collectionSize() const = 0;

	qint64 getNthLandmarkId(int idx) const;
	void setNthLandmarkId(int idx, qint64 id);

	Landmark* getNthLandmark(int idx) const;
	QString getNthLandmarkName(int idx) const;

Q_SIGNALS:

	void attachedLandmarkChanged(int idx, qint64 new_landmark_id);
	void attachedLandmarkNameChanged(int idx);

protected:

	void scaleLandmarksVectors() const;
	void setNthLandmarkIdImpl(int idx, qint64 id);

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	mutable QVector<qint64> _landmarks;
};

class LandmarksTriplet : public LandmarksCollection
{
	Q_OBJECT
public:
	LandmarksTriplet(DataBlock* parent);

	virtual int collectionSize() const;

	QString nameLandmark1() const;
	QString nameLandmark2() const;
	QString nameLandmark3() const;

Q_SIGNALS:

	void attachedLandmark1NameChanged();
	void attachedLandmark2NameChanged();
	void attachedLandmark3NameChanged();

};

class LandmarksPair : public LandmarksCollection
{
	Q_OBJECT
public:
	LandmarksPair(DataBlock* parent);

	virtual int collectionSize() const;

	QString nameLandmark1() const;
	QString nameLandmark2() const;

Q_SIGNALS:

	void attachedLandmark1NameChanged();
	void attachedLandmark2NameChanged();

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LANDMARKSCOLLECTION_H
