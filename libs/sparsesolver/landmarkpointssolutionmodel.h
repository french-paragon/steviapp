#ifndef STEREOVISIONAPP_LANDMARKPOINTSSOLUTIONMODEL_H
#define STEREOVISIONAPP_LANDMARKPOINTSSOLUTIONMODEL_H

#include <QAbstractTableModel>

namespace StereoVisionApp {

class Landmark;
class Image;

class LandmarkPointsSolutionModel : public QAbstractTableModel
{
public:
	LandmarkPointsSolutionModel(QObject* parent = nullptr);

	int rowCount(const QModelIndex &parent = QModelIndex()) const override;
	int columnCount(const QModelIndex &parent = QModelIndex()) const override;

	QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
	QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

	void setLandmark(Landmark* img);

protected:

	void refreshModel();

	enum coordIndex {
		CoordX = 0,
		CoordY = 1,
		CoordZ = 2,
	};

	Image* getImage(int imId) const;
	QVariant getImageLabel(int imId) const;
	QVariant getPointImageCoord(int imId, int coordId) const;
	QVariant getPointReprojectedImageCoord(int imId, int coordId) const;
	QVariant getPointReprojectedImageError(int imId, int coordId) const;
	QVariant getPointWorldCoordinate(int coordId) const;
	QVariant getPointCameraCoordinate(int imId, int coordId) const;

	std::optional<float> getImagePtCoord(int imId, int coordId) const;
	std::optional<float> getImageReprojCoord(int imId, int coordId) const;

	Landmark* _landmark;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LANDMARKPOINTSSOLUTIONMODEL_H
