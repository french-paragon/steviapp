#include "sparsealignementviewer.h"

#include "datablocks/project.h"
#include "datablocks/landmark.h"
#include "datablocks/image.h"
#include "datablocks/camera.h"
#include "datablocks/localcoordinatesystem.h"

#include <StereoVision/geometry/rotations.h>

#include <cmath>

#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QOpenGLTexture>
#include <QOffscreenSurface>
#include <QOpenGLFramebufferObjectFormat>

#include <QFileDialog>

#include <QWheelEvent>
#include <QGuiApplication>

#include "openGlDrawables/opengldrawablescenegrid.h"
#include "openGlDrawables/opengldrawablelandmarkset.h"
#include "openGlDrawables/opengldrawablecamerasset.h"
#include "openGlDrawables/opengldrawablelocalcoordinatesystem.h"

namespace StereoVisionApp {


AbstractSparseAlignementDataInterface::AbstractSparseAlignementDataInterface(QObject* parent) :
	QObject(parent)
{

}

ProjectSparseAlignementDataInterface::ProjectSparseAlignementDataInterface(QObject* parent) :
	AbstractSparseAlignementDataInterface(parent),
	_currentProject(nullptr)
{

}

void ProjectSparseAlignementDataInterface::setProject(Project* p) {

	if (p == _currentProject) {
		return;
	}

	clearProject();

	_currentProject = p;
	connect(p, &Project::projectDataChanged, this, &ProjectSparseAlignementDataInterface::reloadCache);

	reloadCache();

}
void ProjectSparseAlignementDataInterface::clearProject() {

	if (_currentProject != nullptr) {
		disconnect(_currentProject, &Project::projectDataChanged, this, &ProjectSparseAlignementDataInterface::reloadCache);
		_currentProject = nullptr;
	}

	reloadCache();
}

int ProjectSparseAlignementDataInterface::nCameras() const {
	return _loadedFrames.size();
}
int ProjectSparseAlignementDataInterface::nPoints() const {
	return _loadedLandmarks.size();
}
int ProjectSparseAlignementDataInterface::nLocalSystems() const {
    return _loadedLocalCoordinateSystems.size();
}

QMatrix4x4 ProjectSparseAlignementDataInterface::getCameraTransform(int idx) const {

	qint64 imId = _loadedFrames.at(idx);

	Image* im = qobject_cast<Image*>(_currentProject->getById(imId));

	if (im != nullptr) {

		if (im->optPos().isSet() and
				im->optRot().isSet() ) {

			Camera* cam = qobject_cast<Camera*>(_currentProject->getById(im->assignedCamera()));

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
			r.setX(im->optRot().value(0));
			r.setY(im->optRot().value(1));
			r.setZ(im->optRot().value(2));

			camRotate.rotate(r.length()/M_PI*180, r);

			camTranslate.translate(im->optPos().value(0),
								   im->optPos().value(1),
								   im->optPos().value(2));

			QMatrix4x4 camTransform = camTranslate*camRotate*camScale;

			return camTransform;
		}
	}

	return QMatrix4x4();

}

QMatrix4x4 ProjectSparseAlignementDataInterface::getLocalSystemTransform(int idx) const {

    qint64 lcsId = _loadedLocalCoordinateSystems.at(idx);

    LocalCoordinateSystem* lcs = qobject_cast<LocalCoordinateSystem*>(_currentProject->getById(lcsId));

    if (lcs != nullptr) {

        if (lcs->optPos().isSet() and
                lcs->optRot().isSet() ) {

            QMatrix4x4 lcsRotate;
            QMatrix4x4 lcsTranslate;

            QVector3D r;
            r.setX(lcs->optRot().value(0));
            r.setY(lcs->optRot().value(1));
            r.setZ(lcs->optRot().value(2));

            lcsRotate.rotate(r.length()/M_PI*180, r);

            lcsTranslate.translate(lcs->optPos().value(0),
                                   lcs->optPos().value(1),
                                   lcs->optPos().value(2));

            QMatrix4x4 lcsTransform = lcsTranslate*lcsRotate;

            return lcsTransform;
        }
    }

    return QMatrix4x4();

}

QVector3D ProjectSparseAlignementDataInterface::getPointPos(int idx) const {

    if (_currentProject == nullptr) {
        return QVector3D(std::nanf(""), std::nanf(""), std::nanf(""));
    }

	QVector3D ret;

	qint64 lmId = _loadedLandmarks.at(idx);

	Landmark* lm = qobject_cast<Landmark*>(_currentProject->getById(lmId));

    if (lm == nullptr) {
        return QVector3D(std::nanf(""), std::nanf(""), std::nanf(""));
    }

    std::optional<Eigen::Vector3d> optLocalPos = lm->getOptimizableCoordinates(true);

    if (!optLocalPos.has_value()) {
        return QVector3D(std::nanf(""), std::nanf(""), std::nanf(""));
    }

    Eigen::Vector3d& localPos = optLocalPos.value();

    if (_currentProject->hasLocalCoordinateFrame()) {
        localPos = _currentProject->ecef2local()*localPos;
    }

    ret.setX(localPos.x());
    ret.setY(localPos.y());
    ret.setZ(localPos.z());

	return ret;

}

void ProjectSparseAlignementDataInterface::reload() {
	reloadCache();
}

void ProjectSparseAlignementDataInterface::reloadCache() {

    _loadedFrames.clear();
    _loadedLandmarks.clear();
    _loadedLocalCoordinateSystems.clear();

	if (_currentProject == nullptr) {
		Q_EMIT dataChanged();
		return;
	}

	QVector<qint64> lmIds = _currentProject->getIdsByClass(LandmarkFactory::landmarkClassName());

	for (qint64 id : lmIds) {
		Landmark* lm = qobject_cast<Landmark*>(_currentProject->getById(id));

		if (lm != nullptr) {
			if (lm->optPos().isSet()) {
				_loadedLandmarks.push_back(id);
			}
		}
    }

    QVector<qint64> imIds = _currentProject->getIdsByClass(ImageFactory::imageClassName());

    for (qint64 id : imIds) {
        Image* im = qobject_cast<Image*>(_currentProject->getById(id));

        if (im != nullptr) {
            if (im->optPos().isSet() and im->optRot().isSet()) {
                _loadedFrames.push_back(id);
            }
        }
    }

    QVector<qint64> lcsIds = _currentProject->getIdsByClass(LocalCoordinateSystem::staticMetaObject.className());

    for (qint64 id : lcsIds) {
        LocalCoordinateSystem* lcs = qobject_cast<LocalCoordinateSystem*>(_currentProject->getById(id));

        if (lcs != nullptr) {
            if (lcs->optPos().isSet() and lcs->optRot().isSet()) {
                _loadedLocalCoordinateSystems.push_back(id);
            }
        }
    }

	Q_EMIT dataChanged();
}

void ProjectSparseAlignementDataInterface::hooverPoint(int idx) const {

	if (idx < 0 or idx >= _loadedLandmarks.size()) {
		return;
	}

	if (_currentProject != nullptr) {

		Landmark* lm = _currentProject->getDataBlock<Landmark>(_loadedLandmarks[idx]);

		if (lm != nullptr) {
			sendStatusMessage(QString("Landmark \"%1\"").arg(lm->objectName()));
		}

	}

}
void ProjectSparseAlignementDataInterface::hooverCam(int idx) const {

	if (idx < 0 or idx >= _loadedFrames.size()) {
		return;
	}

	if (_currentProject != nullptr) {

		Image* im = _currentProject->getDataBlock<Image>(_loadedFrames[idx]);

		if (im != nullptr) {
			sendStatusMessage(QString("Frame \"%1\"").arg(im->objectName()));
		}

	}

}
void ProjectSparseAlignementDataInterface::hooverLocalCoord(int idx) const {

    if (idx < 0 or idx >= _loadedLocalCoordinateSystems.size()) {
        return;
    }

    if (_currentProject != nullptr) {

        LocalCoordinateSystem* lcs = _currentProject->getDataBlock<LocalCoordinateSystem>(_loadedLocalCoordinateSystems[idx]);

        if (lcs != nullptr) {
            sendStatusMessage(QString("Local system \"%1\"").arg(lcs->objectName()));
        }

    }
}

void ProjectSparseAlignementDataInterface::clickPoint(int idx) const {
    return hooverPoint(idx);
}
void ProjectSparseAlignementDataInterface::clickCam(int idx) const {
    return hooverCam(idx);
}
void ProjectSparseAlignementDataInterface::clickLocalCoordinateSystem(int idx) const {
    return hooverLocalCoord(idx);
}

SparseAlignementViewer::SparseAlignementViewer(QWidget *parent) :
    OpenGl3DSceneViewWidget(parent),
    _currentInterface(nullptr)
{

    _sceneScale = 1.0;
    _camScale = 2.0;

    _drawableGrid = new OpenGlDrawableSceneGrid(this);
    _drawableGrid->setGridDistance(10.);
    _drawableGrid->setGridSplits(10);

    addDrawable(_drawableGrid);

    _drawableLandmarks = new OpenGlDrawableLandmarkSet(this);
    _drawableLandmarks->setSceneScale(_sceneScale);

    addDrawable(_drawableLandmarks);

    _drawableCameras = new OpenGlDrawableCamerasSet(this);
    _drawableCameras->setSceneScale(_sceneScale);
    _drawableCameras->setCamScale(_camScale);

    addDrawable(_drawableCameras);

    _drawableLocalSystems = new OpenGlDrawableLocalCoordinateSystem(this);
    _drawableLocalSystems->setSceneScale(_sceneScale);
    _drawableLocalSystems->setCoordinatesScale(_camScale); //use the same scale as the camera

    addDrawable(_drawableLocalSystems);
}

SparseAlignementViewer::~SparseAlignementViewer() {

	// Make sure the context is current and then explicitly
	// destroy all underlying OpenGL resources.
    makeCurrent();

    doneCurrent();
}

void SparseAlignementViewer::setInterface(AbstractSparseAlignementDataInterface* i) {
    _drawableLandmarks->setInterface(i);
    _drawableCameras->setInterface(i);

    connect(i, &AbstractSparseAlignementDataInterface::sendStatusMessage, this, &SparseAlignementViewer::sendStatusMessage);
    _currentInterface = i;
}

void SparseAlignementViewer::clearInterface() {
    _drawableLandmarks->clearInterface();
    _drawableCameras->clearInterface();

    disconnect(_currentInterface, &AbstractSparseAlignementDataInterface::sendStatusMessage, this, &SparseAlignementViewer::sendStatusMessage);
}


void SparseAlignementViewer::setSceneScale(float sceneScale)
{
    if (_sceneScale != sceneScale) {
        _sceneScale = sceneScale;
        _drawableLandmarks->setSceneScale(sceneScale);
        _drawableCameras->setSceneScale(sceneScale);
        Q_EMIT sceneScaleChanged(_sceneScale);
    }
}

void SparseAlignementViewer::setCamScale(float camScale)
{
    if (_camScale != camScale) {
        _camScale = camScale;
        _drawableCameras->setCamScale(camScale);
        _drawableLocalSystems->setCoordinatesScale(camScale);
        Q_EMIT camScaleChanged(_camScale);
    }
}

void SparseAlignementViewer::scaleCamerasIn(float steps) {
    float scale = _camScale/powf(0.9, steps/10.);
    setCamScale(scale);
}
void SparseAlignementViewer::scaleCamerasOut(float steps) {
    float scale = _camScale*powf(0.9, steps/10.);
    setCamScale(scale);
}

void SparseAlignementViewer::scaleSceneIn(float steps) {
    float scale = _sceneScale*powf(0.9, steps);
    setSceneScale(scale);
}
void SparseAlignementViewer::scaleSceneOut(float steps) {
    float scale = _sceneScale/powf(0.9, steps);
    setSceneScale(scale);
}

void SparseAlignementViewer::saveViewpoint() {
    QString saveFile = QFileDialog::getSaveFileName(this, tr("Save viewpoint"), QString(), {tr("text file (*.txt)")});

    if (saveFile.isEmpty()) {
        return;
    }

    saveViewpoint(saveFile);
}

void SparseAlignementViewer::saveViewpoint(QString file) {

    QFile out(file);

    if (!out.open(QFile::WriteOnly)) {
        return;
    }

    QTextStream stream(&out);

    stream << _view_distance << ' ';

    stream << _zenith_angle << ' ';
    stream << _azimuth_angle << ' ';

    stream << _x_delta << ' ';
    stream << _y_delta << ' ';

    stream << _sceneScale << ' ';
    stream << _camScale << ' ';

    stream.flush();

    out.close();

}

void SparseAlignementViewer::loadViewpoint() {
	QString loadFile = QFileDialog::getOpenFileName(this, tr("Load viewpoint"), QString(), {tr("text file (*.txt)")});

	if (loadFile.isEmpty()) {
		return;
	}

	loadViewpoint(loadFile);
}

void SparseAlignementViewer::loadViewpoint(QString file) {

	QFile in(file);

	if (!in.open(QFile::ReadOnly)) {
		return;
	}

	QTextStream stream(&in);

	stream >> _view_distance;

	stream >> _zenith_angle;
	stream >> _azimuth_angle;

	stream >> _x_delta;
	stream >> _y_delta;

	stream >> _sceneScale;
	stream >> _camScale;

	in.close();

	update();

}

void SparseAlignementViewer::wheelEvent(QWheelEvent *e) {

    QPoint d = e->angleDelta();
    if (d.y() == 0){
        e->ignore();
        return;
    }

    QGuiApplication* gapp = qGuiApp;
    Qt::KeyboardModifiers kmods;

    if (gapp != nullptr) {
        kmods = gapp->keyboardModifiers();
    }

    if (e->buttons() == Qt::NoButton) {

        if (kmods == Qt::ShiftModifier) {

            float step = d.y()/50.;
            if (step < 0) {
                scaleCamerasOut(-step);
            } else {
                scaleCamerasIn(step);
            }

        } else if (kmods == Qt::ControlModifier) {

            float step = d.y()/50.;
            if (step < 0) {
                scaleSceneOut(-step);
            } else {
                scaleSceneIn(step);
            }

        } else {
            float step = d.y()/50.;
            if (step < 0) {
                zoomOut(-step);
            } else {
                zoomIn(step);
            }
        }
        e->accept();
    } else {
        e->ignore();
    }
}

void SparseAlignementViewer::reloadLandmarks() {
    _drawableLandmarks->reloadLandmarks();
}

void SparseAlignementViewer::clearLandmarks() {
    _drawableLandmarks->clearLandmarks();
}

void SparseAlignementViewer::processVoidClick() {
    Q_EMIT sendStatusMessage("");
}

} // namespace StereoVisionApp
