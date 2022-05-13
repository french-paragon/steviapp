#include "sparsealignementviewer.h"

#include "datablocks/project.h"
#include "datablocks/landmark.h"
#include "datablocks/image.h"
#include "datablocks/camera.h"

#include "geometry/rotations.h"

#include <cmath>

#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QOpenGLTexture>
#include <QOffscreenSurface>
#include <QOpenGLFramebufferObjectFormat>

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
	setMouseTracking(true);

	QSurfaceFormat offscreen_id_format = QSurfaceFormat::defaultFormat();
	offscreen_id_format.setSamples(1); //ensure we use a single sample for offscreen id rendering
	offscreen_id_format.setSwapBehavior(QSurfaceFormat::SingleBuffer); //we are not showing the rendered image, so double buffering not required.

	_obj_raycasting_context = new QOpenGLContext();
	_obj_raycasting_surface = new QOffscreenSurface();
	_obj_raycasting_surface->setFormat(offscreen_id_format);
	_obj_raycasting_fbo = nullptr;

	_id_img = nullptr;

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

	if (_obj_raycasting_context != nullptr) {
		if (_obj_raycasting_surface != nullptr) {
			_obj_raycasting_context->makeCurrent(_obj_raycasting_surface);

			if (_objIdProgram != nullptr) {
				delete _objIdProgram;
			}

			if (_objFixedIdProgram != nullptr) {
				delete _objFixedIdProgram;
			}

			_obj_raycasting_context->doneCurrent();
		}

		delete _obj_raycasting_context;

	}

	if (_obj_raycasting_surface != nullptr) {
		delete _obj_raycasting_surface;
	}

	if (_id_img != nullptr) {
		delete _id_img;
	}
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
				if (lm->optPos().isSet()) {
					_loadedLandmarks.push_back(id);
				}
			}
		}

		_llm_pos.clear();
		_llm_pos.reserve(_loadedLandmarks.size()*3);


		for (qint64 id : _loadedLandmarks) {
			Landmark* lm = qobject_cast<Landmark*>(_currentProject->getById(id));
			_llm_pos.push_back(lm->optPos().value(0));
			_llm_pos.push_back(lm->optPos().value(1));
			_llm_pos.push_back(lm->optPos().value(2));
		}
	}

	_hasToReloadLandmarks = true;
	_hasToReloadLandmarksIds = true;
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

	_lm_pos_id_buffer.create();
	_lm_pos_id_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

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

	initializeObjectIdMaskPart();

	makeCurrent(); //make the current context current again, in case initializeGL was called by paintGL.
}

void SparseAlignementViewer::paintGL() {

	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

	f->glClearColor(1,1,1,1);
	f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	f->glLineWidth(1.);

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

				if (im->optPos().isSet() and
						im->optRot().isSet() ) {

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
					r.setX(im->optRot().value(0));
					r.setY(im->optRot().value(1));
					r.setZ(im->optRot().value(2));

					camRotate.rotate(r.length()/M_PI*180, r);

					camTranslate.translate(im->optPos().value(0),
										   im->optPos().value(1),
										   im->optPos().value(2));

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

		paintObjectIdMask(); //paint the selection mask as well
	}

}

void SparseAlignementViewer::resizeGL(int w, int h) {
	setView(w, h);
}


void SparseAlignementViewer::initializeObjectIdMaskPart() {

	if (_obj_raycasting_context == nullptr or _obj_raycasting_surface == nullptr) {
		return; //no object id context mean no object id masks
	}

	_obj_raycasting_context->makeCurrent(_obj_raycasting_surface);

	_objIdProgram = new QOpenGLShaderProgram();
	_objIdProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/objectId.vert");
	_objIdProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/objectId.frag");

	_objIdProgram->link();

	_objFixedIdProgram = new QOpenGLShaderProgram();
	_objFixedIdProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/objectCstId.vert");
	_objFixedIdProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/objectId.frag");

	_objFixedIdProgram->link();

}

void SparseAlignementViewer::paintObjectIdMask() {

	if (_obj_raycasting_context == nullptr or _obj_raycasting_surface == nullptr) {
		return; //no object id context mean no object id masks
	}

	setView(width(), height());

	_obj_raycasting_context->makeCurrent(_obj_raycasting_surface);

	QOpenGLFramebufferObjectFormat fbo_format;
	fbo_format.setSamples(0);
	fbo_format.setInternalTextureFormat(GL_RGB32F); //this can store an int64 using two floats, plus additional information.

	if (_obj_raycasting_fbo == nullptr) {
		_obj_raycasting_fbo = new QOpenGLFramebufferObject(width(), height(), fbo_format);
	} else if (_obj_raycasting_fbo->size() != size()) {
		delete _obj_raycasting_fbo;
		_obj_raycasting_fbo = new QOpenGLFramebufferObject(width(), height(), fbo_format);
	}

	QOpenGLFunctions *f = _obj_raycasting_context->functions();

	_obj_raycasting_fbo->bind();
	f->glViewport(0, 0, width(), height());

	f->glClearColor(0,0,0,0);
	f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (_currentProject != nullptr) {

		int vertexLocation;
		int idLocation;

		if (!_loadedLandmarks.empty()) {

			vertexLocation = _objIdProgram->attributeLocation("in_location");
			idLocation = _objIdProgram->attributeLocation("in_id");

			_objIdProgram->bind();
			_scene_ids_vao.bind();
			_lm_pos_buffer.bind();

			_objIdProgram->enableAttributeArray(vertexLocation);
			_objIdProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);

			_lm_pos_id_buffer.bind();

			if (_hasToReloadLandmarksIds) {
				_lm_pos_id_buffer.allocate(_loadedLandmarks.data(), _loadedLandmarks.size()* sizeof (qint64));
				_hasToReloadLandmarksIds = false;
			}

			_objIdProgram->enableAttributeArray(idLocation);
			_objIdProgram->setAttributeBuffer(idLocation, GL_FLOAT, 0, 2);

			_objIdProgram->setUniformValue("matrixViewProjection", _projectionView*_modelView);
			_objIdProgram->setUniformValue("matrixObjToScene", QMatrix4x4());
			_objIdProgram->setUniformValue("sceneScale", _sceneScale);
			_objIdProgram->setUniformValue("pointScale", 10.0f);
			_objIdProgram->setUniformValue("typeColorIndex", LandmarkColorCode);

			f->glDrawArrays(GL_POINTS, 0, _loadedLandmarks.size());

			_lm_pos_buffer.release();
			_lm_pos_id_buffer.release();
			_scene_ids_vao.release();

			_landMarkPointProgram->disableAttributeArray(vertexLocation);
			_landMarkPointProgram->release();

		}

		//cameras

		_objFixedIdProgram->bind();
		_scene_ids_vao.bind();
		_cam_buffer.bind();
		_cam_indices.bind();

		vertexLocation = _objFixedIdProgram->attributeLocation("in_location");
		_objFixedIdProgram->enableAttributeArray(vertexLocation);
		_objFixedIdProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);

		_objFixedIdProgram->setUniformValue("matrixViewProjection", _projectionView*_modelView);
		_objFixedIdProgram->setUniformValue("sceneScale", _sceneScale);
		_objFixedIdProgram->setUniformValue("pointScale", 10.0f);
		_objFixedIdProgram->setUniformValue("typeColorIndex", CameraColorCode);

		f->glLineWidth(10.);

		QVector<qint64> frames = _currentProject->getIdsByClass(ImageFactory::imageClassName());
		_loadedFrames.clear();
		_loadedFrames.reserve(frames.size());

		for (qint64 id : frames) {

			Image* im = qobject_cast<Image*>(_currentProject->getById(id));

			if (im != nullptr) {

				if (im->optPos().isSet() and
						im->optRot().isSet() ) {

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
					r.setX(im->optRot().value(0));
					r.setY(im->optRot().value(1));
					r.setZ(im->optRot().value(2));

					camRotate.rotate(r.length()/M_PI*180, r);

					camTranslate.translate(im->optPos().value(0),
										   im->optPos().value(1),
										   im->optPos().value(2));

					QMatrix4x4 camTransform = camTranslate*camRotate*camScale;
					_objFixedIdProgram->setUniformValue("matrixObjToScene", camTransform);

					//internal id
					qint64 id = im->internalId();
					float convSpace[2];

					std::memcpy(convSpace, &id, sizeof (id));

					QVector2D idColorCode;
					idColorCode.setX(convSpace[0]);
					idColorCode.setY(convSpace[1]);

					_objFixedIdProgram->setUniformValue("in_id", idColorCode);


					f->glDrawElements(GL_LINES, 30, GL_UNSIGNED_INT, 0);
					//f->glDrawArrays(GL_LINES, 0, 4);

				}

			}

		}

		_cam_buffer.release();
		_cam_indices.release();
		_scene_ids_vao.release();
	}

	if (_id_img == nullptr) {

		_id_img = new Multidim::Array<float, 3>({height(), width(), 3}, {3*width(), 3, 1});

	} else if (_id_img->shape()[0] != height() or _id_img->shape()[1] != width()) {

		delete _id_img;
		_id_img = new Multidim::Array<float, 3>({height(), width(), 3}, {3*width(), 3, 1});
	}

	// tell openGL we want to read from the back buffer
	//f->glReadBuffer(GL_BACK);
	f->glReadPixels(0, 0, width(), height(), GL_RGB, GL_FLOAT, &(_id_img->atUnchecked(0)));

	auto err = glGetError();
	if (err != GL_NO_ERROR) {
		qDebug() << "OpenGL error while reading  id pass: " << err;
	}

	_obj_raycasting_fbo->release();
	_obj_raycasting_context->doneCurrent();
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

		int x = e->pos().x();
		int y = e->pos().y();

		IdPassInfos cid = idPassAtPos(x, y);

		if (cid.itemType == ItemTypeInfo::Landmark) {
			landmarkClick(cid.itemId);
		} else if (cid.itemType == ItemTypeInfo::Camera) {
			frameClick(cid.itemId);
		} else {
			voidClick();
		}

		e->accept();

		return;
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
	} else if (b == Qt::NoButton) {

		int x = e->pos().x();
		int y = e->pos().y();

		IdPassInfos cid = idPassAtPos(x, y);

		if (cid.itemType == ItemTypeInfo::Landmark) {
			landmarkHover(cid.itemId);
		} else if (cid.itemType == ItemTypeInfo::Camera) {
			frameHover(cid.itemId);
		} else {
			voidHover();
		}

		e->accept();

		return;
	}

}

SparseAlignementViewer::IdPassInfos SparseAlignementViewer::idPassAtPos(int x, int y) {

	IdPassInfos ret = { -1, ItemTypeInfo::Unknown};

	if (_obj_raycasting_context == nullptr or _obj_raycasting_surface == nullptr) {
		return ret; //no object id context mean no object id masks
	}

	if (_obj_raycasting_fbo == nullptr) {
		return ret;
	}

	if (x < 0 or x > width()-1) {
		return ret;
	}

	if (y < 0 or y > height()-1) {
		return ret;
	}

	if (_id_img != nullptr) {

		int y_idx = height() - y - 1;

		float red = _id_img->atUnchecked(y_idx, x, 0);
		float green = _id_img->atUnchecked(y_idx, x, 1);
		float blue = _id_img->atUnchecked(y_idx, x, 2);

		//qDebug() << "red: " << red << " green: " << green << " blue: " << blue;

		if (blue <= 0.1) { //
			return ret;
		}

		int r;
		int g;

		//get the bytes of the float pixel values as integer
		std::memcpy(&r, &red, sizeof r);
		std::memcpy(&g, &green, sizeof g);

		qint64 data = static_cast<qint64>(g) << 32 | r;

		ret.itemId = data;
		if (blue == LandmarkColorCode) {
			//qDebug() << "Landmark";
			ret.itemType = ItemTypeInfo::Landmark;
		}
		if (blue == CameraColorCode) {
			//qDebug() << "Camera";
			ret.itemType = ItemTypeInfo::Camera;
		}

		//qDebug() << data;

		return ret;
	}

	return ret;

	/* // keeping that just in case.
	_obj_raycasting_context->makeCurrent(_obj_raycasting_surface);
	_obj_raycasting_fbo->bind();

	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();//_obj_raycasting_context->functions();

	qint64 data;
	float red;
	float green;

	f->glReadPixels(x, y, 1, 1, GL_RED, GL_FLOAT, &red);
	f->glReadPixels(x, y, 1, 1, GL_GREEN, GL_FLOAT, &green);

	qDebug() << "red: " << red << " green: " << green;

	int r;
	int g;

	//get the bytes of the float pixel values as integer
	std::memcpy(&r, &red, sizeof r);
	std::memcpy(&g, &green, sizeof g);

	qDebug() << "r: " << r << " g: " << g;

	data = static_cast<qint64>(r) << 32 | g;

	return data;

	_obj_raycasting_fbo->release();
	_obj_raycasting_context->doneCurrent();*/
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
			QVector3D p(lm->optPos().value(0)*_sceneScale,
						lm->optPos().value(1)*_sceneScale,
						lm->optPos().value(2)*_sceneScale);

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
			QVector3D p(im->optPos().value(0)*_sceneScale,
						im->optPos().value(1)*_sceneScale,
						im->optPos().value(2)*_sceneScale);

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

void SparseAlignementViewer::landmarkHover(int landMarkId) {

	if (_currentProject != nullptr) {

		Landmark* lm = _currentProject->getDataBlock<Landmark>(landMarkId);

		if (lm != nullptr) {
			sendStatusMessage(QString("Landmark \"%1\"").arg(lm->objectName()));
		}

	}

}

void SparseAlignementViewer::frameHover(int frameId) {

	if (_currentProject != nullptr) {

		Image* im = _currentProject->getDataBlock<Image>(frameId);

		if (im != nullptr) {
			sendStatusMessage(QString("Frame \"%1\"").arg(im->objectName()));
		}

	}
}

void SparseAlignementViewer::voidHover() {
	sendStatusMessage("");
}


void SparseAlignementViewer::landmarkClick(int landMarkId) {
	landmarkHover(landMarkId);
	setMouseTracking(false);
}
void SparseAlignementViewer::frameClick(int frameId) {
	frameHover(frameId);
	setMouseTracking(false);
}
void SparseAlignementViewer::voidClick() {
	voidHover();
	setMouseTracking(true);
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
