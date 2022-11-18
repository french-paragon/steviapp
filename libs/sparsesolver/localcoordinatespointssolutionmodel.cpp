#include "localcoordinatespointssolutionmodel.h"

#include "datablocks/localcoordinatesystem.h"
#include "datablocks/landmark.h"

namespace StereoVisionApp {

LocalCoordinatesPointsSolutionModel::LocalCoordinatesPointsSolutionModel(QObject *parent) :
	QAbstractTableModel(parent),
	_localCoordinateSystem(nullptr)
{

}

int LocalCoordinatesPointsSolutionModel::rowCount(const QModelIndex &parent) const {
	if (parent != QModelIndex()) {
		return 0;
	}

	if (_localCoordinateSystem != nullptr) {
		return _localCoordinateSystem->listTypedSubDataBlocks(LandmarkLocalCoordinates::staticMetaObject.className()).size();
	}

	return 0;
}
int LocalCoordinatesPointsSolutionModel::columnCount(const QModelIndex &parent) const {
	Q_UNUSED(parent);

	int ret = 1; //name of the landmark
	ret += 3; //world space position of the landmark
	ret += 3; //world space position of the landmark, reprojected from local coordinates
	ret += 3; //reprojection error

	return ret;
}

QVariant LocalCoordinatesPointsSolutionModel::data(const QModelIndex &index, int role) const {

	if (index.parent() != QModelIndex()) {
		return QVariant();
	}

	switch (role) {
	case Qt::DisplayRole :
		switch (index.column()) {
		case 0:
			return getLandmarkLabel(index.row());
		case 1:
			return getLandmarkWorldCoordinate(index.row(), CoordX);
		case 2:
			return getLandmarkWorldCoordinate(index.row(), CoordY);
		case 3:
			return getLandmarkWorldCoordinate(index.row(), CoordZ);
		case 4:
			return getLandmarkReprojectedWorldCoordinate(index.row(), CoordX);
		case 5:
			return getLandmarkReprojectedWorldCoordinate(index.row(), CoordY);
		case 6:
			return getLandmarkReprojectedWorldCoordinate(index.row(), CoordZ);
		case 7:
			return getLandmarkReprojectedError(index.row(), CoordX);
		case 8:
			return getLandmarkReprojectedError(index.row(), CoordY);
		case 9:
			return getLandmarkReprojectedError(index.row(), CoordZ);
		default:
			return QVariant();
		}
	case Qt::ToolTipRole :
		return headerData(index.column(), Qt::Horizontal, Qt::ToolTipRole);
	default:
		return QVariant();
	}

	return QVariant();

}
QVariant LocalCoordinatesPointsSolutionModel::headerData(int section, Qt::Orientation orientation, int role) const {

	if (orientation == Qt::Vertical) {
		return QVariant();
	}

	switch (role) {
	case Qt::DisplayRole:
		switch (section) {
		case 0:
			return tr("Landmark");
		case 1:
			return tr("World x");
		case 2:
			return tr("World y");
		case 3:
			return tr("World y");
		case 4:
			return tr("Reprojected x");
		case 5:
			return tr("Reprojected y");
		case 6:
			return tr("Reprojected z");
		case 7:
			return tr("Error x");
		case 8:
			return tr("Error y");
		case 9:
			return tr("Error z");
		default:
			return QVariant();
		}
	case Qt::ToolTipRole:
		switch (section) {
		case 0:
			return tr("Landmark name");
		case 1:
			return tr("Landmark world coordinate x");
		case 2:
			return tr("Landmark world coordinate y");
		case 3:
			return tr("Landmark world coordinate z");
		case 4:
			return tr("Landmark world coordinate x, reprojected from local coordinate");
		case 5:
			return tr("Landmark world coordinate y, reprojected from local coordinate");
		case 6:
			return tr("Landmark world coordinate z, reprojected from local coordinate");
		case 7:
			return tr("Landmark reprojection error x");
		case 8:
			return tr("Landmark reprojection error y");
		case 9:
			return tr("Landmark reprojection error z");
		default:
			return QVariant();
		}
	default:
		break;
	}

	return QVariant();

}

void LocalCoordinatesPointsSolutionModel::setLocalCoordinateSystem(LocalCoordinateSystem* lcs) {
	beginResetModel();

	if (_localCoordinateSystem != nullptr) {
		disconnect(_localCoordinateSystem, &LocalCoordinateSystem::datablockChanged, this, &LocalCoordinatesPointsSolutionModel::refreshModel);
	}
	_localCoordinateSystem = lcs;

	if (_localCoordinateSystem != nullptr) {
		connect(_localCoordinateSystem, &LocalCoordinateSystem::datablockChanged, this, &LocalCoordinatesPointsSolutionModel::refreshModel);
	}
	endResetModel();
}

void LocalCoordinatesPointsSolutionModel::refreshModel() {
	beginResetModel();
	endResetModel();
}


LandmarkLocalCoordinates* LocalCoordinatesPointsSolutionModel::getLandmarkLocalCoordinates(int lmId) const {
	qint64 lmlc_id = _localCoordinateSystem->listTypedSubDataBlocks(LandmarkLocalCoordinates::staticMetaObject.className())[lmId];

	LandmarkLocalCoordinates* lmlc = _localCoordinateSystem->getLandmarkLocalCoordinates(lmlc_id);
	return lmlc;
}

Landmark* LocalCoordinatesPointsSolutionModel::getLandmark(int lmId) const {
	LandmarkLocalCoordinates* lmlc = getLandmarkLocalCoordinates(lmId);

	if (lmlc == nullptr) {
		return nullptr;
	}

	Landmark* lm = lmlc->attachedLandmark();
	return lm;
}

QVariant LocalCoordinatesPointsSolutionModel::getLandmarkLabel(int lmId) const {
	Landmark* lm = getLandmark(lmId);

	if (lm != nullptr) {
		return lm->objectName();
	}

	return tr("Missing landmark");
}

QVariant LocalCoordinatesPointsSolutionModel::getLandmarkWorldCoordinate(int lmId, int coordId) const{
	Landmark* lm = getLandmark(lmId);

	if (lm == nullptr) {
		return QVariant();
	}

	if (!lm->optPos().isSet()) {
		return QVariant();
	}

	return lm->optPos().value(coordId);
}

QVariant LocalCoordinatesPointsSolutionModel::getLandmarkReprojectedWorldCoordinate(int lmId, int coordId) const {

	auto cand = _localCoordinateSystem->getOptTransform();

	if (!cand.has_value()) {
		return QVariant();
	}

	StereoVision::Geometry::AffineTransform localToWorld = cand.value();

	LandmarkLocalCoordinates* lmlc = getLandmarkLocalCoordinates(lmId);

	if (lmlc == nullptr) {
		return QVariant();
	}

	auto cand2 = lmlc->getPointVec();

	if (!cand2.has_value()) {
		return QVariant();
	}

	Eigen::Vector3f local_pos = cand2.value();
	Eigen::Vector3f world_pos = localToWorld*local_pos;

	return world_pos[coordId];

}

QVariant LocalCoordinatesPointsSolutionModel::getLandmarkReprojectedError(int lmId, int coordId) const {

	auto cand = _localCoordinateSystem->getOptTransform();

	if (!cand.has_value()) {
		return QVariant();
	}

	StereoVision::Geometry::AffineTransform localToWorld = cand.value();

	LandmarkLocalCoordinates* lmlc = getLandmarkLocalCoordinates(lmId);

	if (lmlc == nullptr) {
		return QVariant();
	}

	auto cand2 = lmlc->getPointVec();

	if (!cand2.has_value()) {
		return QVariant();
	}

	Eigen::Vector3f local_pos = cand2.value();
	Eigen::Vector3f world_pos_proj = localToWorld*local_pos;

	Landmark* lm = getLandmark(lmId);

	if (lm == nullptr) {
		return QVariant();
	}

	auto cand3 = lm->getOptPointVec();

	if (!cand3.has_value()) {
		return QVariant();
	}

	Eigen::Vector3f world_pos = cand3.value();

	return (world_pos - world_pos_proj)[coordId];
}

} // namespace StereoVisionApp
