#include "imagepointssolutionmodel.h"

#include "datablocks/image.h"
#include "datablocks/landmark.h"

#include "helperfunctions.h"

namespace StereoVisionApp {

ImagePointsSolutionModel::ImagePointsSolutionModel(QObject *parent) :
	QAbstractTableModel(parent),
	_image(nullptr)
{

}

int ImagePointsSolutionModel::rowCount(const QModelIndex &parent) const {
	if (parent != QModelIndex()) {
		return 0;
	}

	if (_image != nullptr) {
		return _image->listTypedSubDataBlocks(ImageLandmark::staticMetaObject.className()).length();
	}

	return 0;
}
int ImagePointsSolutionModel::columnCount(const QModelIndex &parent) const {

	Q_UNUSED(parent);

	int ret = 1; //name of the point
	ret += 2; //position of the points in the image
	ret += 3; //computed position of the points in image space
	ret += 3; //computed position of the points in world space

	return ret;
}

QVariant ImagePointsSolutionModel::data(const QModelIndex &index, int role) const {

	if (index.parent() != QModelIndex()) {
		return QVariant();
	}

	switch (role) {
	case Qt::DisplayRole :
		switch (index.column()) {
		case 0:
			return getPointLabel(index.row());
		case 1:
			return getPointImageCoord(index.row(), CoordX);
		case 2:
			return getPointImageCoord(index.row(), CoordY);
		case 3:
			return getPointCameraCoordinate(index.row(), CoordX);
		case 4:
			return getPointCameraCoordinate(index.row(), CoordY);
		case 5:
			return getPointCameraCoordinate(index.row(), CoordZ);
		case 6:
			return getPointWorldCoordinate(index.row(), CoordX);
		case 7:
			return getPointWorldCoordinate(index.row(), CoordY);
		case 8:
			return getPointWorldCoordinate(index.row(), CoordZ);
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

QVariant ImagePointsSolutionModel::headerData(int section, Qt::Orientation orientation, int role) const {
	if (orientation == Qt::Vertical) {
		return QVariant();
	}

	switch (role) {
	case Qt::DisplayRole:
		switch (section) {
		case 0:
			return tr("Point");
		case 1:
			return tr("Pix x");
		case 2:
			return tr("Pix y");
		case 3:
			return tr("x pos cam");
		case 4:
			return tr("y pos cam");
		case 5:
			return tr("z pos cam");
		case 6:
			return tr("x pos world");
		case 7:
			return tr("y pos world");
		case 8:
			return tr("z pos world");
		default:
			return QVariant();
		}
	case Qt::ToolTipRole:
		switch (section) {
		case 0:
			return tr("Point label");
		case 1:
			return tr("Pixel x coordinate");
		case 2:
			return tr("Pixel y coordinate");
		case 3:
			return tr("Point x position in camera space");
		case 4:
			return tr("Point y position in camera space");
		case 5:
			return tr("Point z position in camera space");
		case 6:
			return tr("Point x position in world space");
		case 7:
			return tr("Point y position in world space");
		case 8:
			return tr("Point z position in world space");
		default:
			return QVariant();
		}
	default:
		break;
	}

	return QVariant();
}


void ImagePointsSolutionModel::setImage(Image* img) {
	beginResetModel();

	if (_image != nullptr) {
		connect(_image, &Image::datablockChanged, this, &ImagePointsSolutionModel::refreshModel);
	}
	_image = img;

	if (_image != nullptr) {
		connect(_image, &Image::datablockChanged, this, &ImagePointsSolutionModel::refreshModel);
	}
	endResetModel();
}

void ImagePointsSolutionModel::refreshModel() {
	beginResetModel();
	endResetModel();
}

QVariant ImagePointsSolutionModel::getPointLabel(int ptId) const {

	if (_image == nullptr) {
		return QVariant();
	}

	qint64 ilm_id = _image->listTypedSubDataBlocks(ImageLandmark::staticMetaObject.className())[ptId];
	ImageLandmark* ilm = _image->getImageLandmark(ilm_id);

	if (ilm != nullptr) {
		Landmark* lm = ilm->attachedLandmark();
		if (lm != nullptr) {
			return lm->objectName();
		}
	}

	return QVariant();
}

QVariant ImagePointsSolutionModel::getPointImageCoord(int ptId, int coordId) const {
	if (_image == nullptr) {
		return QVariant();
	}

	qint64 ilm_id = _image->listTypedSubDataBlocks(ImageLandmark::staticMetaObject.className())[ptId];
	ImageLandmark* ilm = _image->getImageLandmark(ilm_id);

	if (ilm != nullptr) {
		if (coordId == CoordX) {
			return ilm->x().value();
		}else if (coordId == CoordY) {
			return ilm->y().value();
		}
	}

	return QVariant();
}

QVariant ImagePointsSolutionModel::getPointWorldCoordinate(int ptId, int coordId) const {

	if (_image == nullptr) {
		return QVariant();
	}

	qint64 ilm_id = _image->listTypedSubDataBlocks(ImageLandmark::staticMetaObject.className())[ptId];
	ImageLandmark* ilm = _image->getImageLandmark(ilm_id);

	if (ilm != nullptr) {
		Landmark* lm = ilm->attachedLandmark();
		if (lm != nullptr) {
			auto val = getPointPosition(lm);
			if (val.has_value()) {
				return val.value()[coordId];
			}
		}
	}

	return "not set";
}

QVariant ImagePointsSolutionModel::getPointCameraCoordinate(int ptId, int coordId) const {

	if (_image == nullptr) {
		return QVariant();
	}

	qint64 ilm_id = _image->listTypedSubDataBlocks(ImageLandmark::staticMetaObject.className())[ptId];
	ImageLandmark* ilm = _image->getImageLandmark(ilm_id);

	if (ilm != nullptr) {
		Landmark* lm = ilm->attachedLandmark();
		if (lm != nullptr) {
			auto val = getPointPosition(lm);
			auto transform = getWorldToImageTransform(_image);
			if (val.has_value() and transform.has_value()) {
				Eigen::Vector3f camCoord = transform.value()*val.value();
				return camCoord[coordId];
			}
		}
	}

	return "not set";
}

} // namespace StereoVisionApp
