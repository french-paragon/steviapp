#ifndef STEREOVISIONAPP_CAMERACALIBRATION_H
#define STEREOVISIONAPP_CAMERACALIBRATION_H

#include "./project.h"

#include "LibStevi/imageProcessing/checkBoardDetection.h"

#include <QThread>
#include <QMutex>
#include <QSet>
#include <QMap>

namespace StereoVisionApp {

class CameraCalibrationImageList;
class CameraCalibrationWorker;

class CameraCalibration : public DataBlock
{
	Q_OBJECT
public:
	explicit CameraCalibration(Project* parent);

	inline int nImgs() const {
		return _imagesInfos.size();
	}

	void addImg(qint64 id);
	void setImageCorners(qint64 id,
						 QVector<StereoVision::discretCheckCornerInfos> candidates,
						 QVector<StereoVision::discretCheckCornerInfos> selected,
						 QVector<StereoVision::refinedCornerInfos> corners);
	void clearImageCorners(qint64 id);
	void removeImg(qint64 id);

	std::optional<QVector<StereoVision::discretCheckCornerInfos>> getDetectedCandidates(qint64 id) const;
	std::optional<QVector<StereoVision::discretCheckCornerInfos>> getFilteredCandidates(qint64 id) const;
	std::optional<QVector<StereoVision::refinedCornerInfos>> getImageCorners(qint64 id) const;
	inline bool hasImage(qint64 id) const {
		QMutexLocker locker(&_workersSyncMutex);
		return _images.contains(id);
	}

	inline bool imageIsActive(qint64 id) const {
		return _imagesInfos.value(id, {false, {}, {}, {}}).Active;
	}

	inline QList<qint64> loadedImages() const {
		QMutexLocker locker(&_workersSyncMutex);
		return _images.values();
	}

	inline CameraCalibrationImageList* imageList() const {
		return _imageList;
	}

Q_SIGNALS:

	void imageAdded(qint64 id);
	void imageRemoved(qint64 id);
	void imageDataAcquired(qint64 id);

protected:

	struct ImgInfos {

		bool Active;
		QVector<StereoVision::discretCheckCornerInfos> Candidates;
		QVector<StereoVision::discretCheckCornerInfos> Filtered;
		QVector<StereoVision::refinedCornerInfos> DetectedCorners;

	};

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	mutable QSet<qint64> _images;
	QMap<qint64, ImgInfos> _imagesInfos;

	CameraCalibrationImageList* _imageList;

	QList<CameraCalibrationWorker*> _worker_queue;

	mutable QMutex _workersSyncMutex;

	friend class CameraCalibrationImageList;
	friend class CameraCalibrationWorker;

};

class CameraCalibrationWorker : public QThread
{
	Q_OBJECT
public:
	explicit CameraCalibrationWorker(CameraCalibration* parent, qint64 imgId, QString imageFile);

	void run() override;
	void endWorker();

private:

	CameraCalibration* _calibration;
	qint64 _imgId;
	QString _imageFile;
};

class CameraCalibrationImageList : public QAbstractListModel
{
	Q_OBJECT
public:
	explicit CameraCalibrationImageList(CameraCalibration* parent);

	int rowCount(const QModelIndex &parent = QModelIndex()) const override;
	QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

	QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;


protected:

	CameraCalibration* _calibration;

	friend class CameraCalibration;

};

class CameraCalibrationFactory : public DataBlockFactory
{
	Q_OBJECT
public:
	explicit CameraCalibrationFactory(QObject* parent = nullptr);

	QString TypeDescrName() const override;
	FactorizableFlags factorizable() const override;
	DataBlock* factorizeDataBlock(Project *parent = nullptr) const override;

	QString itemClassName() const override;
	static QString cameraCalibrationClassName();
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CAMERACALIBRATION_H
