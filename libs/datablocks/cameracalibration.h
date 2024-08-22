#ifndef STEREOVISIONAPP_CAMERACALIBRATION_H
#define STEREOVISIONAPP_CAMERACALIBRATION_H

#include "./project.h"
#include "./floatparameter.h"

#include <StereoVision/imageProcessing/checkBoardDetection.h>

#include <QThread>
#include <QMutex>
#include <QSet>
#include <QMap>
#include <QPoint>
#include <QPointF>

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

	void setImageEstimatedPose(qint64 id, floatParameterGroup<6> const& pose);

	inline std::optional<floatParameterGroup<6>> getImageEstimatedPose(qint64 id) const {
		if (_camerasLocations.contains(id)) {
			return _camerasLocations.value(id);
		}
		return std::nullopt;
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

	int nSelectedCameras() const;
	qint64 nthSelectedCameras(int idx) const;

	inline int selectedGridSize() const {
		return _selected_grid_width*_selected_grid_height;
	}
	inline int selectedGridWidth() const {
		return _selected_grid_width;
	}
	inline int selectedGridHeight() const {
		return _selected_grid_height;
	}

	void configureSelectedImages();
	void configureSelectedImages(int gridWidth, int gridHeight);
	void clearConfiguredImages();

	inline float getGridSize() const {
		return _grid_size;
	}
	inline void setGridSize(float grid_size) {
		_grid_size = grid_size;
	}


	inline QPointF gridPointXYCoordinate(QPoint pos) const {
		return QPointF((pos.x() - static_cast<float>(_selected_grid_width-1)/2.)*_grid_size,
					   (static_cast<float>(_selected_grid_height-1)/2. - pos.y())*_grid_size);
	}

	/*!
	 * \brief getJsonRepresentation do nothing for this class
	 * \return an empty object
	 */
	QJsonObject getJsonRepresentation() const override;
	/*!
	 * \brief setParametersFromJsonRepresentation do nothing for this class
	 */
	void setParametersFromJsonRepresentation(QJsonObject const& rep) override;

Q_SIGNALS:

	void imageAdded(qint64 id);
	void imageRemoved(qint64 id);
	void imageDataAcquired(qint64 id);

	void selectedImagesConfigured(bool configured);

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
	QMap<qint64, floatParameterGroup<6>> _camerasLocations;

    CameraCalibrationImageList* _imageList;

	mutable QMutex _workersSyncMutex;

	friend class CameraCalibrationImageList;
	friend class CameraCalibrationWorker;

	QVector<qint64> _selectedImages;
	int _selected_grid_width;
	int _selected_grid_height;

	float _grid_size;

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
