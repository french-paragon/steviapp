#ifndef STEREOVISIONAPP_IMAGEPOINTSSOLUTIONMODEL_H
#define STEREOVISIONAPP_IMAGEPOINTSSOLUTIONMODEL_H

#include <QAbstractTableModel>

namespace StereoVisionApp {

class Image;

class ImagePointsSolutionModel : public QAbstractTableModel
{
public:
	ImagePointsSolutionModel(QObject* parent = nullptr);

	int rowCount(const QModelIndex &parent = QModelIndex()) const override;
	int columnCount(const QModelIndex &parent = QModelIndex()) const override;

	QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
	QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

	void setImage(Image* img);

protected:

	enum coordIndex {
		CoordX = 0,
		CoordY = 1,
		CoordZ = 2,
	};

	QVariant getPointLabel(int ptId) const;
	QVariant getPointImageCoord(int ptId, int coordId) const;
	QVariant getPointWorldCoordinate(int ptId, int coordId) const;
	QVariant getPointCameraCoordinate(int ptId, int coordId) const;

	Image* _image;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMAGEPOINTSSOLUTIONMODEL_H
