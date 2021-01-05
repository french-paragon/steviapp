#include "sparsealignementviewer.h"

#include "datablocks/project.h"
#include "datablocks/landmark.h"
#include "datablocks/image.h"
#include "datablocks/camera.h"

#include "geometry/rotations.h"

#include <cmath>

#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>

#include <QWheelEvent>
#include <QGuiApplication>

namespace StereoVisionApp {

SparseAlignementViewer::SparseAlignementViewer(QWidget *parent) :
	QOpenGLWidget(parent),
	_has_been_initialised(false),
	_cam_indices(QOpenGLBuffer::IndexBuffer),
	_gridProgram(nullptr),
	_landMarkPointProgram(nullptr),
	_currentProject(nullptr),
	_hasToReloadLandmarks(true)
{
	_gridDistance = 10.;
	_gridSplits = 10;

	_min_view_distance = 0.01;
	_max_view_distance = 100.;

	_sceneScale = 1.0;
	_camScale = 0.2;

	setFocusPolicy(Qt::StrongFocus);

	resetView();
}

SparseAlignementViewer::~SparseAlignementViewer() {

	// Make sure the context is current and then explicitly
	// destroy all underlying OpenGL resources.
	makeCurrent();

	if (_gridProgram != nullptr) {
		delete _gridProgram;
	}

	if (_landMarkPointProgram != nullptr) {
		delete _landMarkPointProgram;
	}

	if (_camProgram != nullptr) {
		delete _camProgram;
	}

	if (_grid_buffer.isCreated()) {
		_grid_buffer.destroy();
	}

	if (_lm_pos_buffer.isCreated()) {
		_lm_pos_buffer.destroy();
	}

	if (_cam_buffer.isCreated()) {
		_cam_buffer.destroy();
	}

	if (_cam_indices.isCreated()) {
		_cam_indices.destroy();
	}

	_grid_vao.destroy();
	_scene_vao.destroy();
	_cam_vao.destroy();

	doneCurrent();
}


void SparseAlignementViewer::zoomIn(float steps) {
	_view_distance = _view_distance*powf(0.9, steps);
	if (_view_distance < _min_view_distance) {
		_view_distance = _min_view_distance;
	}
	update();
}
void SparseAlignementViewer::zoomOut(float steps) {
	_view_distance = _view_distance/powf(0.9, steps);
	if (_view_distance > _max_view_distance) {
		_view_distance = _max_view_distance;
	}
	update();
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

void SparseAlignementViewer::rotateZenith(float degrees) {

	float newAngle = _zenith_angle + degrees;
	if (newAngle < 0) {
		newAngle = 0;
	} else if (newAngle > 180) {
		newAngle = 180;
	}

	if (newAngle != _zenith_angle) {
		_zenith_angle = newAngle;
		update();
	}

}
void SparseAlignementViewer::rotateAzimuth(float degrees) {

	float newAngle = _azimuth_angle + degrees;

	if (newAngle < 0) {
		newAngle += (floor(fabs(newAngle)/360.) + 1)*360.;
	} else if (newAngle > 360) {
		newAngle -= floor(fabs(newAngle)/360.)*360.;
	}

	if (newAngle != _azimuth_angle) {
		_azimuth_angle = newAngle;
		update();
	}

}

void SparseAlignementViewer::reloadLandmarks() {

	if (_currentProject != nullptr) {
		_loadedLandmarks.clear();
		QVector<qint64> lmIds = _currentProject->getIdsByClass(LandmarkFactory::landmarkClassName());

		for (qint64 id : lmIds) {
			Landmark* lm = qobject_cast<Landmark*>(_currentProject->getById(id));

			if (lm != nullptr) {
				if (lm->optimizedX().isSet() and
					lm->optimizedY().isSet() and
					lm->optimizedZ().isSet()) {
					_loadedLandmarks.push_back(id);
				}
			}
		}

		_llm_pos.clear();
		_llm_pos.reserve(_loadedLandmarks.size()*3);


		for (qint64 id : _loadedLandmarks) {
			Landmark* lm = qobject_cast<Landmark*>(_currentProject->getById(id));
			_llm_pos.push_back(lm->optimizedX().value());
			_llm_pos.push_back(lm->optimizedY().value());
			_llm_pos.push_back(lm->optimizedZ().value());
		}
	}

	_hasToReloadLandmarks = true;
}

void SparseAlignementViewer::clearLandmarks() {
	_loadedLandmarks.clear();
	update();
}

void SparseAlignementViewer::initializeGL() {

	_grid_vao.create();
	_scene_vao.create();
	_cam_vao.create();

	if (_grid_vao.isCreated()) {
		_grid_vao.bind();
	}

	generateGrid();
	setView(width(), height());

	_grid_vao.release();

	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
	f->glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	f->glEnable(GL_MULTISAMPLE);
	f->glEnable(GL_POINT_SMOOTH);
	f->glEnable(GL_PROGRAM_POINT_SIZE);

	_gridProgram = new QOpenGLShaderProgram();
	_gridProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/sparseViewerPerspFloor.vert");
	_gridProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/sparseViewerGrid.frag");

	_lm_pos_buffer.create();
	_lm_pos_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

	reloadLandmarks();

	_landMarkPointProgram = new QOpenGLShaderProgram();
	_landMarkPointProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/sparseViewerPersp.vert");
	_landMarkPointProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/sparseViewerPoints.frag");

	_landMarkPointProgram->link();

	_cam_vao.bind();

	generateCamModel();

	_cam_vao.release();

	_camProgram = new QOpenGLShaderProgram();
	_camProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/sparseViewerCam.vert");
	_camProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/sparseViewerCam.frag");

	_camProgram->link();

	_has_been_initialised = true;
}

void SparseAlignementViewer::paintGL() {

	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

	setView(width(), height());

	_gridProgram->bind();
	_grid_vao.bind();
	_grid_buffer.bind();


	int vertexLocation =  _gridProgram->attributeLocation("in_location");
	_gridProgram->enableAttributeArray(vertexLocation);
	_gridProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 2);

	_gridProgram->setUniformValue("matrixViewProjection", _projectionView*_modelView);

	f->glDrawArrays(GL_LINES, 0, 8*(_gridSplits + 1));

	_grid_vao.release();
	_grid_buffer.release();

	_gridProgram->disableAttributeArray(vertexLocation);
	_gridProgram->release();

	if (_currentProject != nullptr) {

		if (!_loadedLandmarks.empty()) {

			_landMarkPointProgram->bind();
			_scene_vao.bind();
			_lm_pos_buffer.bind();

			if (_hasToReloadLandmarks) {
				_lm_pos_buffer.allocate(_llm_pos.data(), _llm_pos.size()*sizeof (GLfloat));
				_hasToReloadLandmarks = false;
			}

			vertexLocation = _landMarkPointProgram->attributeLocation("in_location");
			_landMarkPointProgram->enableAttributeArray(vertexLocation);
			_landMarkPointProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);

			_landMarkPointProgram->setUniformValue("matrixViewProjection", _projectionView*_modelView);
			_landMarkPointProgram->setUniformValue("sceneScale", _sceneScale);

			f->glDrawArrays(GL_POINTS, 0, _loadedLandmarks.size());

			_scene_vao.release();
			_lm_pos_buffer.release();

			_landMarkPointProgram->disableAttributeArray(vertexLocation);
			_landMarkPointProgram->release();

		}

		//cameras

		_camProgram->bind();
		_cam_vao.bind();
		_cam_buffer.bind();
		_cam_indices.bind();

		vertexLocation = _camProgram->attributeLocation("in_location");
		_camProgram->enableAttributeArray(vertexLocation);
		_camProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);

		_camProgram->setUniformValue("matrixViewProjection", _projectionView*_modelView);
		_camProgram->setUniformValue("sceneScale", _sceneScale);

		QVector<qint64> frames = _currentProject->getIdsByClass(ImageFactory::imageClassName());
		_loadedFrames.clear();
		_loadedFrames.reserve(frames.size());

		for (qint64 id : frames) {

			Image* im = qobject_cast<Image*>(_currentProject->getById(id));

			if (im != nullptr) {

				if (im->optXCoord().isSet() and
						im->optYCoord().isSet() and
						im->optZCoord().isSet() and
						im->optXRot().isSet() and
						im->optYRot().isSet() and
						im->optZRot().isSet() ) {

					_loadedFrames.push_back(im->internalId());

					Camera* cam = qobject_cast<Camera*>(_currentProject->getById(im->assignedCamera()));

					QMatrix4x4 camScale;
					QMatrix4x4 camRotate;
					QMatrix4x4 camTranslate;
					camScale.scale(_camScale);

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
					r.setX(im->optXRot().value());
					r.setY(im->optYRot().value());
					r.setZ(im->optZRot().value());

					camRotate.rotate(r.length()/M_PI*180, r);

					camTranslate.translate(im->optXCoord().value(),
										   im->optYCoord().value(),
										   im->optZCoord().value());

					QMatrix4x4 camTransform = camTranslate*camRotate*camScale;
					_camProgram->setUniformValue("matrixCamToScene", camTransform);

					f->glDrawElements(GL_LINES, 30, GL_UNSIGNED_INT, 0);
					//f->glDrawArrays(GL_LINES, 0, 4);

				}

			}

		}

		_cam_vao.release();
		_cam_buffer.release();
		_cam_indices.release();

		_camProgram->disableAttributeArray(vertexLocation);
		_camProgram->release();
	}


}
void SparseAlignementViewer::resizeGL(int w, int h) {
	setView(w, h);
}

void SparseAlignementViewer::generateGrid() {

	if (_grid_buffer.isCreated()) {
		_grid_buffer.destroy();
	}

	_grid_buffer.create();
	_grid_buffer.bind();

	size_t s = 16*_gridSplits + 8;
	std::vector<GLfloat> b(s);

	for(int i = 0; i < 2*_gridSplits + 1; i++) {
		b[4*i] = _gridDistance;
		b[4*i+1] = (i - _gridSplits)*_gridDistance/(_gridSplits);
		b[4*i+2] = -_gridDistance;
		b[4*i+3] = (i - _gridSplits)*_gridDistance/(_gridSplits);
	}

	for(int i = 2*_gridSplits + 1; i < 2*(2*_gridSplits + 1); i++) {
		int p = i - (2*_gridSplits + 1);
		b[4*i] = (p-_gridSplits)*_gridDistance/(_gridSplits);
		b[4*i+1] = _gridDistance;
		b[4*i+2] = (p-_gridSplits)*_gridDistance/(_gridSplits);
		b[4*i+3] = -_gridDistance;
	}

	int sw = 2*(2*_gridSplits + 1) - 1;
	int tw = _gridSplits;
	float tmp;

	tmp= b[4*sw];
	b[4*sw] = b[4*tw];
	b[4*tw] = tmp;

	tmp= b[4*sw+1];
	b[4*sw+1] = b[4*tw+1];
	b[4*tw+1] = tmp;

	tmp= b[4*sw+2];
	b[4*sw+2] = b[4*tw+2];
	b[4*tw+2] = tmp;

	tmp= b[4*sw+3];
	b[4*sw+3] = b[4*tw+3];
	b[4*tw+3] = tmp;

	_grid_buffer.allocate(b.data(), s*sizeof (GLfloat));
	_grid_buffer.setUsagePattern(QOpenGLBuffer::StaticDraw);

}
void SparseAlignementViewer::generateCamModel() {

	if (_cam_buffer.isCreated()) {
		_cam_buffer.destroy();
	}

	_cam_buffer.create();
	_cam_buffer.bind();

	std::vector<GLfloat> p(11*3);

	//pt0
	p[0] = 1.;
	p[1] = 1.;
	p[2] = 1.;

	//pt1
	p[3] = -1.;
	p[4] = 1.;
	p[5] = 1.;

	//pt2
	p[6] = -1.;
	p[7] = -1.;
	p[8] = 1.;

	//pt3
	p[9] = 1.;
	p[10] = -1.;
	p[11] = 1.;

	//pt4
	p[12] = 0.3;
	p[13] = 0.3;
	p[14] = -0.3;

	//pt5
	p[15] = -0.3;
	p[16] = 0.3;
	p[17] = -0.3;

	//pt6
	p[18] = -0.3;
	p[19] = -0.3;
	p[20] = -0.3;

	//pt7
	p[21] = 0.3;
	p[22] = -0.3;
	p[23] = -0.3;


	//pt8
	p[24] = 0.5;
	p[25] = -1.1;
	p[26] = 1.0;


	//pt9
	p[27] = -0.5;
	p[28] = -1.1;
	p[29] = 1.0;


	//pt10
	p[30] = 0.0;
	p[31] = -1.5;
	p[32] = 1.0;


	_cam_buffer.allocate(p.data(), p.size()*sizeof (GLfloat));
	_cam_buffer.setUsagePattern(QOpenGLBuffer::StaticDraw);


	if (_cam_indices.isCreated()) {
		_cam_indices.destroy();
	}

	_cam_indices.create();
	_cam_indices.bind();

	std::vector<uint32_t> i(30);

	i[0] = 0;
	i[1] = 1;

	i[2] = 1;
	i[3] = 2;

	i[4] = 2;
	i[5] = 3;

	i[6] = 3;
	i[7] = 0;

	i[8] = 0;
	i[9] = 6;

	i[10] = 1;
	i[11] = 7;

	i[12] = 2;
	i[13] = 4;

	i[14] = 3;
	i[15] = 5;

	i[16] = 4;
	i[17] = 5;

	i[18] = 5;
	i[19] = 6;

	i[20] = 6;
	i[21] = 7;

	i[22] = 7;
	i[23] = 4;

	i[24] = 8;
	i[25] = 9;

	i[26] = 9;
	i[27] = 10;

	i[28] = 10;
	i[29] = 8;

	_cam_indices.allocate(i.data(), i.size()*sizeof (uint32_t));
	_cam_indices.setUsagePattern(QOpenGLBuffer::StaticDraw);

}

void SparseAlignementViewer::setProject(Project* p) {

	if (p == _currentProject) {
		return;
	}

	clearProject();

	_currentProject = p;
	connect(p, &Project::projectChanged, this, &SparseAlignementViewer::clearLandmarks);

	if (_has_been_initialised) {
		reloadLandmarks();
	}
}
void SparseAlignementViewer::clearProject() {

	if (_currentProject != nullptr) {
		disconnect(_currentProject, &Project::projectChanged, this, &SparseAlignementViewer::clearLandmarks);
		_currentProject = nullptr;
	}

	clearLandmarks();

}

void SparseAlignementViewer::resetView() {
	resetView(width(), height());
}

void SparseAlignementViewer::resetView(int w, int h) {

	_view_distance = 10.;

	_zenith_angle = 0;
	_azimuth_angle = 0;

	setView(w,h);
}

void SparseAlignementViewer::setView(int w, int h) {
	QMatrix4x4 model;
	QMatrix4x4 view;
	QMatrix4x4 proj;
	//_projectionView.rotate(90.,0.,1.,0.);
	model.rotate(_zenith_angle, -1.0, 0.0);
	model.rotate(_azimuth_angle, 0.0, 0.0, 1.0);

	view.lookAt(QVector3D(0,0,_view_distance),
				QVector3D(0,0,0),
				QVector3D(0,1.0,0));

	proj.perspective(70., w/static_cast<float>(h), 0.1f, 1000.0f);

	_modelView = view*model;
	_projectionView = proj;
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
void SparseAlignementViewer::keyPressEvent(QKeyEvent *e) {

	int k = e->key();
	int m = e->modifiers();

	if (k == Qt::Key_Down and m & Qt::KeypadModifier) {
		rotateZenith(10.);
	} else if (k == Qt::Key_Up and m & Qt::KeypadModifier) {
		rotateZenith(-10.);
	} else if (k == Qt::Key_Left and m & Qt::KeypadModifier) {
		rotateAzimuth(10.);
	} else if (k == Qt::Key_Right and m & Qt::KeypadModifier) {
		rotateAzimuth(-10.);
	} else if (k == Qt::Key_Plus and m & Qt::KeypadModifier) {
		zoomIn(1.);
	} else if (k == Qt::Key_Minus and m & Qt::KeypadModifier) {
		zoomOut(1.);
	} else {
		QWidget::keyPressEvent(e);
	}

}
void SparseAlignementViewer::mousePressEvent(QMouseEvent *e) {

	_previously_pressed = e->buttons();

	if (_previously_pressed == Qt::MiddleButton or
			_previously_pressed == Qt::LeftButton) {

		_motion_origin_pos = e->pos();

		e->accept();
	} else {
		e->ignore();
	}

}
void SparseAlignementViewer::mouseReleaseEvent(QMouseEvent *e) {

	if (_previously_pressed == Qt::MiddleButton) {

		e->ignore();

	} else if (_previously_pressed == Qt::LeftButton) {

		itemClickInfos camInfo = nearestCam(e->pos());
		itemClickInfos landmarkInfo = nearestLandmark(e->pos());

		if (camInfo.itemId >= 0 and landmarkInfo.itemId >= 0) {
			if (camInfo.zDist < landmarkInfo.zDist) {
				frameClick(camInfo.itemId);
			} else {
				landmarkClick(landmarkInfo.itemId);
			}
		} else if (camInfo.itemId >= 0) {
			frameClick(camInfo.itemId);
		} else if (landmarkInfo.itemId >= 0) {
			landmarkClick(landmarkInfo.itemId);
		}

		e->accept();
	}
}

void SparseAlignementViewer::mouseMoveEvent(QMouseEvent *e) {

	int b = e->buttons();

	if (b == Qt::MiddleButton) {
		QPoint nP = e->pos();

		QPoint t = nP - _motion_origin_pos;
		_motion_origin_pos = nP;

		rotateZenith(-t.y()/3.);
		rotateAzimuth(t.x()/3.);
	}

}

SparseAlignementViewer::itemClickInfos SparseAlignementViewer::nearestLandmark(QPoint const& pt, int minPixDist) {

	if (_currentProject == nullptr) {
		return {-1, -1, -1};
	}

	qint64 id = -1;
	float dist = minPixDist;
	float z_dist = std::numeric_limits<float>::infinity();

	QVector2D clickPos(pt.x(), height() - pt.y());

	for (qint64 cand_id : _loadedLandmarks) {

		Landmark* lm = _currentProject->getDataBlock<Landmark>(cand_id);

		if (lm != nullptr) {
			QVector3D p(lm->optimizedX().value()*_sceneScale,
						lm->optimizedY().value()*_sceneScale,
						lm->optimizedZ().value()*_sceneScale);

			QVector3D project = p.project(_modelView, _projectionView, rect());

			if (project.z() > 1) {
				continue;
			}

			QVector2D ppos(project.x(), project.y());

			float d = ppos.distanceToPoint(clickPos);

			if (d < 2*minPixDist) {
				if (project.z() < z_dist) {
					id = cand_id;
					dist = d;
					z_dist = project.z();
				}
			}
		}

	}

	return {id, dist/2, z_dist};

}

SparseAlignementViewer::itemClickInfos SparseAlignementViewer::nearestCam(QPoint const& pt, int minPixDist) {

	if (_currentProject == nullptr) {
		return {-1, -1, -1};
	}

	qint64 id = -1;
	float dist = minPixDist;
	float z_dist = std::numeric_limits<float>::infinity();

	QVector2D clickPos(pt.x(), height() - pt.y());

	for (qint64 cand_id : _loadedFrames) {

		Image* im = _currentProject->getDataBlock<Image>(cand_id);

		if (im != nullptr) {
			QVector3D p(im->optXCoord().value()*_sceneScale,
						im->optYCoord().value()*_sceneScale,
						im->optZCoord().value()*_sceneScale);

			QVector3D project = p.project(_modelView, _projectionView, rect());

			if (project.z() > 1) {
				continue;
			}

			QVector2D ppos(project.x(), project.y());

			float d = ppos.distanceToPoint(clickPos);

			if (d < 2*minPixDist) {
				if (project.z() < z_dist) {
					id = cand_id;
					dist = d;
					z_dist = project.z();
				}
			}
		}

	}

	return {id, dist/2, z_dist};

}

void SparseAlignementViewer::landmarkClick(int landMarkId) {

	if (_currentProject != nullptr) {

		Landmark* lm = _currentProject->getDataBlock<Landmark>(landMarkId);

		if (lm != nullptr) {
			qDebug() << "Landmark [" << lm->objectName() << "] clicked !";
		}

	}

}

void SparseAlignementViewer::frameClick(int frameId) {

	if (_currentProject != nullptr) {

		Image* im = _currentProject->getDataBlock<Image>(frameId);

		if (im != nullptr) {
			qDebug() << "Frame [" << im->objectName() << "] clicked !";
		}

	}
}

void SparseAlignementViewer::setSceneScale(float sceneScale)
{
	_sceneScale = fabs(sceneScale);
	update();
}

void SparseAlignementViewer::setCamScale(float camScale)
{
	_camScale = fabs(camScale);
	update();
}

} // namespace StereoVisionApp
