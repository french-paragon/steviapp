#include "landmarkpointssolutionmodel.h"

#include "datablocks/landmark.h"
#include "datablocks/image.h"
#include "datablocks/camera.h"
#include "helperfunctions.h"

#include <StereoVision/geometry/alignement.h>

namespace StereoVisionApp {

LandmarkPointsSolutionModel::LandmarkPointsSolutionModel(QObject* parent) :
	QAbstractTableModel(parent),
	_landmark(nullptr)
{

}

int LandmarkPointsSolutionModel::rowCount(const QModelIndex &parent) const {
	if (parent != QModelIndex()) {
		return 0;
	}

	if (_landmark != nullptr) {
		return _landmark->getImagesRefering().length();
	}

	return 0;
}
int LandmarkPointsSolutionModel::columnCount(const QModelIndex &parent) const {

	Q_UNUSED(parent);

	int ret = 1; //name of the point
	ret += 2; //position of the points in the image
	ret += 2; //position of the points in the image reprojected
	ret += 2; //reprojection error
	ret += 3; //computed position of the points in image space
	ret += 3; //computed position of the points in world space

	return ret;
}

QVariant LandmarkPointsSolutionModel::data(const QModelIndex &index, int role) const {

	if (index.parent() != QModelIndex()) {
		return QVariant();
	}

	switch (role) {
	case Qt::DisplayRole :
		switch (index.column()) {
		case 0:
			return getImageLabel(index.row());
		case 1:
			return getPointImageCoord(index.row(), CoordX);
		case 2:
			return getPointImageCoord(index.row(), CoordY);
		case 3:
			return getPointReprojectedImageCoord(index.row(), CoordX);
		case 4:
			return getPointReprojectedImageCoord(index.row(), CoordY);
		case 5:
			return getPointReprojectedImageError(index.row(), CoordX);
		case 6:
			return getPointReprojectedImageError(index.row(), CoordY);
		case 7:
			return getPointCameraCoordinate(index.row(), CoordX);
		case 8:
			return getPointCameraCoordinate(index.row(), CoordY);
		case 9:
			return getPointCameraCoordinate(index.row(), CoordZ);
		case 10:
			return getPointWorldCoordinate(CoordX);
		case 11:
			return getPointWorldCoordinate(CoordY);
		case 12:
			return getPointWorldCoordinate(CoordZ);
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

QVariant LandmarkPointsSolutionModel::headerData(int section, Qt::Orientation orientation, int role) const {
	if (orientation == Qt::Vertical) {
		return QVariant();
	}

	switch (role) {
	case Qt::DisplayRole:
		switch (section) {
		case 0:
			return tr("Image");
		case 1:
			return tr("Pix x");
		case 2:
			return tr("Pix y");
		case 3:
			return tr("Pix x (reprojected)");
		case 4:
			return tr("Pix y (reprojected)");
		case 5:
			return tr("X reprojection error");
		case 6:
			return tr("Y reprojection error");
		case 7:
			return tr("x pos cam");
		case 8:
			return tr("y pos cam");
		case 9:
			return tr("z pos cam");
		case 10:
			return tr("x pos world");
		case 11:
			return tr("y pos world");
		case 12:
			return tr("z pos world");
		default:
			return QVariant();
		}
	case Qt::ToolTipRole:
		switch (section) {
		case 0:
			return tr("Image name");
		case 1:
			return tr("Pixel x coordinate");
		case 2:
			return tr("Pixel y coordinate");
		case 3:
			return tr("Pixel x coordinate reprojected from world space");
		case 4:
			return tr("Pixel y coordinate reprojected from world space");
		case 5:
			return tr("Pixel x reprojection error");
		case 6:
			return tr("Pixel y reprojection error");
		case 7:
			return tr("Point x position in camera space");
		case 8:
			return tr("Point y position in camera space");
		case 9:
			return tr("Point z position in camera space");
		case 10:
			return tr("Point x position in world space");
		case 11:
			return tr("Point y position in world space");
		case 12:
			return tr("Point z position in world space");
		default:
			return QVariant();
		}
	default:
		break;
	}

	return QVariant();
}

void LandmarkPointsSolutionModel::setLandmark(Landmark* lm) {
	beginResetModel();

	if (_landmark != nullptr) {
		disconnect(_landmark, &Landmark::datablockChanged, this, &LandmarkPointsSolutionModel::refreshModel);
	}
	_landmark= lm;

	if (_landmark != nullptr) {
		connect(_landmark, &Landmark::datablockChanged, this, &LandmarkPointsSolutionModel::refreshModel);
	}

	endResetModel();
}

void LandmarkPointsSolutionModel::refreshModel() {
	beginResetModel();
	endResetModel();
}

Image* LandmarkPointsSolutionModel::getImage(int imId) const {


	qint64 im_id = _landmark->getImagesRefering()[imId];

	Image* im = _landmark->getProject()->getDataBlock<Image>(im_id);
	return im;
}

QVariant LandmarkPointsSolutionModel::getImageLabel(int imId) const {

	if (_landmark == nullptr) {
		return QVariant();
	}

	Image* im = getImage(imId);

	if (im != nullptr) {
		return im->objectName();
	}

	return QVariant();
}

QVariant LandmarkPointsSolutionModel::getPointImageCoord(int imId, int coordId) const {

	auto val = getImagePtCoord(imId, coordId);

	if (val.has_value()) {
		return val.value();
	}

	return QVariant();
}
QVariant LandmarkPointsSolutionModel::getPointReprojectedImageCoord(int imId, int coordId) const {

	auto val = getImageReprojCoord(imId, coordId);

	if (val.has_value()) {
		return val.value();
	}

	return QVariant();
}
QVariant LandmarkPointsSolutionModel::getPointReprojectedImageError(int imId, int coordId) const {

	auto im = getImagePtCoord(imId, coordId);
	auto proj = getImageReprojCoord(imId, coordId);

	if (im.has_value() and proj.has_value()) {
		return proj.value() - im.value();
	}

	return QVariant();

}

std::optional<float> LandmarkPointsSolutionModel::getImagePtCoord(int imId, int coordId) const {

	if (_landmark == nullptr) {
		return std::nullopt;
	}

	Image* im = getImage(imId);

	if (im != nullptr) {
		ImageLandmark* ilm = im->getImageLandmarkByLandmarkId(_landmark->internalId());
		if (ilm != nullptr) {
			if (coordId == CoordX) {
				return ilm->x().value();
			}else if (coordId == CoordY) {
				return ilm->y().value();
			}
		}
	}

	return std::nullopt;
}
std::optional<float> LandmarkPointsSolutionModel::getImageReprojCoord(int imId, int coordId) const {


	if (_landmark == nullptr) {
		return std::nullopt;
	}

	auto val = getPointPosition(_landmark);

	if (!val.has_value()) {
		return std::nullopt;
	}

	Eigen::Vector3f ppos = val.value();

	Image* im = getImage(imId);

	if (im != nullptr) {

		qint64 camid = im->assignedCamera();

		Camera* cam = im->getProject()->getDataBlock<Camera>(camid);

		if (cam != nullptr) {

			float flen;
			float ppx;
			float ppy;

			if (cam->optimizedFLen().isSet()) {
				flen = cam->optimizedFLen().value();
			} else {
				flen = cam->fLen().value();
			}

			if (cam->optimizedOpticalCenterX().isSet()) {
				ppx = cam->optimizedOpticalCenterX().value();
			} else {
				ppx = cam->opticalCenterX().value();
			}

			if (cam->optimizedOpticalCenterY().isSet()) {
				ppy = cam->optimizedOpticalCenterY().value();
			} else {
				ppy = cam->opticalCenterY().value();
			}

			auto world2cam = getWorldToImageTransform(im);

			if (world2cam.has_value()) {

				Eigen::Vector3f pcam = world2cam.value()*ppos;

				if (pcam[2] < 0.0) {
					return std::nanf("");
				}

				Eigen::Vector2f pos = StereoVision::Geometry::projectPoints(pcam);

				if (coordId == 0) {
					return flen*pos[0] + ppx;
				}
				if (coordId == 1) {
					return flen*pos[1] + ppy;
				}
			}
		}

	}

	return std::nullopt;
}

QVariant LandmarkPointsSolutionModel::getPointWorldCoordinate(int coordId) const {

	if (_landmark == nullptr) {
		return QVariant();
	}

	auto val = getPointPosition(_landmark);
	if (val.has_value()) {
		return val.value()[coordId];
	}

	return QVariant();
}

QVariant LandmarkPointsSolutionModel::getPointCameraCoordinate(int imId, int coordId) const {

	Image* im = getImage(imId);

	if (im == nullptr) {
		return QVariant();
	}

	if (_landmark != nullptr) {
		auto val = getPointPosition(_landmark);
		auto transform = getWorldToImageTransform(im);
		if (val.has_value() and transform.has_value()) {
			Eigen::Vector3f camCoord = transform.value()*val.value();
			return camCoord[coordId];
		}
	}

	return QVariant();
}


} // namespace StereoVisionApp
