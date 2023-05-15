#include "cameracalibrationsparsealignementviewerinterface.h"

#include "datablocks/project.h"
#include "datablocks/camera.h"
#include "datablocks/image.h"

namespace StereoVisionApp {

CameraCalibrationSparseAlignementViewerInterface::CameraCalibrationSparseAlignementViewerInterface(float gridSize, QObject *parent) :
	AbstractSparseAlignementDataInterface(parent),
	_currentCalibration(nullptr),
	_grid_size(gridSize)
{

}

void CameraCalibrationSparseAlignementViewerInterface::setCalibration(CameraCalibration* c) {

	if (c == _currentCalibration) {
		return;
	}

	clearCalibration();

	_currentCalibration = c;
	connect(c, &CameraCalibration::selectedImagesConfigured, this, &ProjectSparseAlignementDataInterface::dataChanged);

	Q_EMIT dataChanged();

}
void CameraCalibrationSparseAlignementViewerInterface::clearCalibration() {

	if (_currentCalibration != nullptr) {
		disconnect(_currentCalibration, &CameraCalibration::selectedImagesConfigured, this, &ProjectSparseAlignementDataInterface::dataChanged);
		_currentCalibration = nullptr;
	}
}

int CameraCalibrationSparseAlignementViewerInterface::nCameras() const {
	if (_currentCalibration == nullptr) {
		return 0;
	}

	return _currentCalibration->nSelectedCameras();
}

int CameraCalibrationSparseAlignementViewerInterface::nPoints() const {
	if (_currentCalibration == nullptr) {
		return 0;
	}
	return _currentCalibration->selectedGridSize();
}

QMatrix4x4 CameraCalibrationSparseAlignementViewerInterface::getCameraTransform(int idx) const {

	if (_currentCalibration == nullptr) {
		return QMatrix4x4();
	}

	qint64 imId = _currentCalibration->nthSelectedCameras(idx);

	auto imgTransform = _currentCalibration->getImageEstimatedPose(imId);

	Project* currentProject = _currentCalibration->getProject();
	Image* im = currentProject->getDataBlock<Image>(imId);

	if (imgTransform.has_value() and im != nullptr) {

		floatParameterGroup<6> pose = imgTransform.value();

		Camera* cam = qobject_cast<Camera*>(currentProject->getById(im->assignedCamera()));

		QMatrix4x4 camScale;
		QMatrix4x4 camRotate;
		QMatrix4x4 camTranslate;

		if (cam != nullptr) {
			QSize s = cam->imSize();
			qreal aspect_ratio = static_cast<qreal>(s.width())/static_cast<qreal>(s.height());
			if (aspect_ratio > 1) {
				camScale.scale(1., 1./aspect_ratio);
			} else {
				camScale.scale(aspect_ratio, 1.0);
			}
		}

		QVector3D r;
		r.setX(pose.value(3));
		r.setY(pose.value(4));
		r.setZ(pose.value(5));

		camRotate.rotate(r.length()/M_PI*180, r);

		camTranslate.translate(pose.value(0),
							   pose.value(1),
							   pose.value(2));

		QMatrix4x4 camTransform = camTranslate*camRotate*camScale;

		return camTransform;
	}

	return QMatrix4x4();
}

QVector3D CameraCalibrationSparseAlignementViewerInterface::getPointPos(int idx) const {

	if (_currentCalibration == nullptr) {
		return QVector3D();
	}

	int w = _currentCalibration->selectedGridWidth();

	int wCoord = idx % w;
	int hCoord = idx/w;

	QPointF pos = _currentCalibration->gridPointXYCoordinate(QPoint(wCoord, hCoord));

	return QVector3D(pos.x(), pos.y(), 0);
}

float CameraCalibrationSparseAlignementViewerInterface::getGridSize() const
{
	return _grid_size;
}

void CameraCalibrationSparseAlignementViewerInterface::setGridSize(float grid_size)
{
	if (grid_size != _grid_size) {
		_grid_size = grid_size;
		dataChanged();
	}
}

void CameraCalibrationSparseAlignementViewerInterface::reload() {
	//nothing needs to be done, as there is no cache.
}

void CameraCalibrationSparseAlignementViewerInterface::hooverPoint(int idx) const {


	if (_currentCalibration != nullptr) {

		if (idx < 0 or idx >= _currentCalibration->selectedGridSize()) {
			return;
		}

		sendStatusMessage(QString("Corner \"%1 %2\"").arg(idx%_currentCalibration->selectedGridWidth()).arg(idx/_currentCalibration->selectedGridWidth()));

	}

}
void CameraCalibrationSparseAlignementViewerInterface::hooverCam(int idx) const {

	if (_currentCalibration != nullptr) {

		qint64 frameId = _currentCalibration->nthSelectedCameras(idx);

		Image* im = _currentCalibration->getProject()->getDataBlock<Image>(frameId);

		if (im != nullptr) {
			sendStatusMessage(QString("Image \"%1\"").arg(im->objectName()));
		}

	}

}

void CameraCalibrationSparseAlignementViewerInterface::clickPoint(int idx) const {
	return hooverPoint(idx);
}
void CameraCalibrationSparseAlignementViewerInterface::clickCam(int idx) const {
	return hooverCam(idx);
}

} // namespace StereoVisionApp
