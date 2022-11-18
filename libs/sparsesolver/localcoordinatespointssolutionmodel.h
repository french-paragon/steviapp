#ifndef STEREOVISIONAPP_LOCALCOORDINATESPOINTSSOLUTIONMODEL_H
#define STEREOVISIONAPP_LOCALCOORDINATESPOINTSSOLUTIONMODEL_H

#include <QAbstractTableModel>

namespace StereoVisionApp {

class Landmark;
class LocalCoordinateSystem;
class LandmarkLocalCoordinates;

class LocalCoordinatesPointsSolutionModel : public QAbstractTableModel
{
public:
	LocalCoordinatesPointsSolutionModel(QObject* parent = nullptr);

	int rowCount(const QModelIndex &parent = QModelIndex()) const override;
	int columnCount(const QModelIndex &parent = QModelIndex()) const override;

	QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
	QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

	void setLocalCoordinateSystem(LocalCoordinateSystem *lcs);

protected:

	void refreshModel();

	enum coordIndex {
		CoordX = 0,
		CoordY = 1,
		CoordZ = 2,
	};

	LandmarkLocalCoordinates* getLandmarkLocalCoordinates(int lmId) const;
	Landmark* getLandmark(int lmId) const;
	QVariant getLandmarkLabel(int lmId) const;
	QVariant getLandmarkWorldCoordinate(int lmId, int coordId) const;
	QVariant getLandmarkReprojectedWorldCoordinate(int lmId, int coordId) const;
	QVariant getLandmarkReprojectedError(int lmId, int coordId) const;

	LocalCoordinateSystem* _localCoordinateSystem;
};


} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LOCALCOORDINATESPOINTSSOLUTIONMODEL_H
