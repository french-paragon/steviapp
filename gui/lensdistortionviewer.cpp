#include "lensdistortionviewer.h"

#include "datablocks/camera.h"

#include<QOpenGLShaderProgram>
#include<QOpenGLFunctions>

#include<cmath>

namespace StereoVisionApp {

LensDistortionViewer::LensDistortionViewer(QWidget *parent) :
	QOpenGLWidget(parent),
	_cam(nullptr),
	_useOptimizedValues(false),
	_n_pt_w(40)
{

}

LensDistortionViewer::~LensDistortionViewer() {

	// Make sure the context is current and then explicitly
	// destroy all underlying OpenGL resources.
	makeCurrent();

	if (_gridProgram != nullptr) {
		delete _gridProgram;
	}

	if (_lineProgram != nullptr) {
		delete _lineProgram;
	}

	if (_frameProgram != nullptr) {
		delete _frameProgram;
	}

	if (_grid_buffer.isCreated()) {
		_grid_buffer.destroy();
	}

	if (_line_buffer.isCreated()) {
		_line_buffer.destroy();
	}

	if (_frame_buffer.isCreated()) {
		_frame_buffer.destroy();
	}

	_grid_vao.destroy();
	_line_vao.destroy();
	_frame_vao.destroy();

	doneCurrent();

}

void LensDistortionViewer::setCamera(Camera* cam) {

	if (cam == _cam) {
		return;
	}

	if (_cam != nullptr) {
		disconnect(_cam, &Camera::FLenChanged, this, &LensDistortionViewer::onCamChanged);

		disconnect(_cam, &Camera::opticalCenterXChanged, this, &LensDistortionViewer::onCamChanged);
		disconnect(_cam, &Camera::opticalCenterYChanged, this, &LensDistortionViewer::onCamChanged);

		disconnect(_cam, &Camera::k1Changed, this, &LensDistortionViewer::onCamChanged);
		disconnect(_cam, &Camera::k2Changed, this, &LensDistortionViewer::onCamChanged);
		disconnect(_cam, &Camera::k3Changed, this, &LensDistortionViewer::onCamChanged);

		disconnect(_cam, &Camera::k4Changed, this, &LensDistortionViewer::onCamChanged);
		disconnect(_cam, &Camera::k5Changed, this, &LensDistortionViewer::onCamChanged);
		disconnect(_cam, &Camera::k6Changed, this, &LensDistortionViewer::onCamChanged);

		disconnect(_cam, &Camera::p1Changed, this, &LensDistortionViewer::onCamChanged);
		disconnect(_cam, &Camera::p2Changed, this, &LensDistortionViewer::onCamChanged);

		disconnect(_cam, &Camera::B1Changed, this, &LensDistortionViewer::onCamChanged);
		disconnect(_cam, &Camera::B2Changed, this, &LensDistortionViewer::onCamChanged);

		disconnect(_cam, &QObject::destroyed, this, &LensDistortionViewer::activeCameraDestroyed);
	}

	_cam = cam;

	if (_cam != nullptr) {
		connect(_cam, &Camera::FLenChanged, this, &LensDistortionViewer::onCamChanged);

		connect(_cam, &Camera::opticalCenterXChanged, this, &LensDistortionViewer::onCamChanged);
		connect(_cam, &Camera::opticalCenterYChanged, this, &LensDistortionViewer::onCamChanged);

		connect(_cam, &Camera::k1Changed, this, &LensDistortionViewer::onCamChanged);
		connect(_cam, &Camera::k2Changed, this, &LensDistortionViewer::onCamChanged);
		connect(_cam, &Camera::k3Changed, this, &LensDistortionViewer::onCamChanged);

		connect(_cam, &Camera::k4Changed, this, &LensDistortionViewer::onCamChanged);
		connect(_cam, &Camera::k5Changed, this, &LensDistortionViewer::onCamChanged);
		connect(_cam, &Camera::k6Changed, this, &LensDistortionViewer::onCamChanged);

		connect(_cam, &Camera::p1Changed, this, &LensDistortionViewer::onCamChanged);
		connect(_cam, &Camera::p2Changed, this, &LensDistortionViewer::onCamChanged);

		connect(_cam, &Camera::B1Changed, this, &LensDistortionViewer::onCamChanged);
		connect(_cam, &Camera::B2Changed, this, &LensDistortionViewer::onCamChanged);

		connect(_cam, &QObject::destroyed, this, &LensDistortionViewer::activeCameraDestroyed);
	}

	setFrameTransform(width(), height());
	generateVertices();
	update();

}

void LensDistortionViewer::useOptimizedValues(bool useOptimizedValues) {

	if (useOptimizedValues != _useOptimizedValues) {

		_useOptimizedValues = useOptimizedValues;

		if (_cam != nullptr) {
			update();
		}
	}
}

void LensDistortionViewer::initializeGL() {

	_grid_vao.create();
	_line_vao.create();
	_frame_vao.create();

	generateVertices();

	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
	f->glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	f->glEnable(GL_MULTISAMPLE);

	_gridProgram = new QOpenGLShaderProgram();
	_gridProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/lensViewerDistGrid.vert");
	_gridProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/lensViewerDistGrid.frag");

	_gridProgram->link();

	_lineProgram = new QOpenGLShaderProgram();
	_lineProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/lensViewerOpticalCenterLine.vert");
	_lineProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/lensViewerLine.frag");

	_lineProgram->link();

	_frameProgram = new QOpenGLShaderProgram();
	_frameProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/lensViewerLine.vert");
	_frameProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/lensViewerLine.frag");

	_frameProgram->link();

}
void LensDistortionViewer::paintGL() {

	if (_cam == nullptr) {
		return;
	}

	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

	int vertexLocation;

	//grid
	_grid_vao.bind();
	_gridProgram->bind();
	_grid_buffer.bind();

	vertexLocation =  _gridProgram->attributeLocation("in_location");
	_gridProgram->enableAttributeArray(vertexLocation);
	_gridProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 2);

	_gridProgram->setUniformValue("frameTransform", _frame_transform);

	if (_useOptimizedValues) {
		_gridProgram->setUniformValue("f", _cam->optimizedFLen().value());
		_gridProgram->setUniformValue("pp", _cam->optimizedOpticalCenterX().value(), _cam->optimizedOpticalCenterY().value());
		_gridProgram->setUniformValue("k123", _cam->optimizedK1().value(), _cam->optimizedK2().value(), _cam->optimizedK3().value());
		_gridProgram->setUniformValue("k456", _cam->optimizedK4().value(), _cam->optimizedK5().value(), _cam->optimizedK6().value());
		_gridProgram->setUniformValue("p12", _cam->optimizedP1().value(), _cam->optimizedP2().value());
		_gridProgram->setUniformValue("B12", _cam->optimizedB1().value(), _cam->optimizedB2().value());
	} else {
		_gridProgram->setUniformValue("f", _cam->fLen().value());
		_gridProgram->setUniformValue("pp", _cam->opticalCenterX().value(), _cam->opticalCenterY().value());
		_gridProgram->setUniformValue("k123", _cam->k1().value(), _cam->k2().value(), _cam->k3().value());
		_gridProgram->setUniformValue("k456", _cam->k4().value(), _cam->k5().value(), _cam->k6().value());
		_gridProgram->setUniformValue("p12", _cam->p1().value(), _cam->p2().value());
		_gridProgram->setUniformValue("B12", _cam->B1().value(), _cam->B2().value());
	}

	f->glDrawArrays(GL_LINES, 0, 2*_n_pt_t);

	_grid_vao.release();

	_grid_buffer.release();

	_gridProgram->disableAttributeArray(vertexLocation);
	_gridProgram->release();

	//lines
	_line_vao.bind();
	_lineProgram->bind();
	_line_buffer.bind();

	vertexLocation = _lineProgram->attributeLocation("in_location");
	_lineProgram->enableAttributeArray(vertexLocation);
	_lineProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 2);

	_lineProgram->setUniformValue("frameTransform", _frame_transform);

	if (_useOptimizedValues) {
		_lineProgram->setUniformValue("opticalCenter", static_cast<GLfloat>(_cam->optimizedOpticalCenterX().value()), static_cast<GLfloat>(_cam->optimizedOpticalCenterY().value()));
	} else {
		_lineProgram->setUniformValue("opticalCenter", static_cast<GLfloat>(_cam->opticalCenterX().value()), static_cast<GLfloat>(_cam->opticalCenterY().value()));
	}

	_lineProgram->setUniformValue("frameSize", static_cast<GLfloat>(_cam->imSize().width()), static_cast<GLfloat>(_cam->imSize().height()));

	_lineProgram->setUniformValue("lineColor", QColor(220, 220, 220));

	f->glDrawArrays(GL_LINES, 0, 4);

	_line_vao.release();
	_line_buffer.release();

	_lineProgram->disableAttributeArray(vertexLocation);
	_lineProgram->release();

	//frames
	_frame_vao.bind();
	_frameProgram->bind();
	_frame_buffer.bind();

	vertexLocation = _frameProgram->attributeLocation("in_location");
	_frameProgram->enableAttributeArray(vertexLocation);
	_frameProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 2);

	_frameProgram->setUniformValue("frameTransform", _frame_transform);
	_frameProgram->setUniformValue("lineColor", QColor(100, 100, 100));

	f->glDrawArrays(GL_LINE_LOOP, 0, 4);

	_frame_vao.release();
	_frame_buffer.release();

	_frameProgram->disableAttributeArray(vertexLocation);
	_frameProgram->release();
}
void LensDistortionViewer::resizeGL(int w, int h) {

	setFrameTransform(w, h);
}


void LensDistortionViewer::generateVertices() {


	if (_grid_buffer.isCreated()) {
		_grid_buffer.destroy();
	}

	if (_line_buffer.isCreated()) {
		_line_buffer.destroy();
	}

	if (_frame_buffer.isCreated()) {
		_frame_buffer.destroy();
	}

	if (_cam == nullptr) {
		return;
	}


	QSize s = _cam->imSize();

	float ptInterval = static_cast<float>(s.width())/(_n_pt_w+1.0);
	int n_pt_h = static_cast<int>(floor(static_cast<float>(s.height())/ptInterval));
	_n_pt_t = n_pt_h*_n_pt_w;
	float d_pt_h =  (s.height() - n_pt_h*ptInterval)/2;

	size_t bs = 4*_n_pt_t;
	std::vector<GLfloat> b(bs);

	int p = 0;
	for (int i = 0; i < _n_pt_w; i++) {
		for (int j = 0; j < n_pt_h; j++) {
			b[p++] = (i+1)*ptInterval;
			b[p++] = d_pt_h + j*ptInterval;
			b[p++] = (i+1)*ptInterval;
			b[p++] = d_pt_h + j*ptInterval;
		}
	}

	_grid_vao.bind();
	_grid_buffer.create();
	_grid_buffer.bind();
	_grid_buffer.allocate(b.data(), bs*sizeof (GLfloat));
	_grid_buffer.setUsagePattern(QOpenGLBuffer::StaticDraw);

	_grid_buffer.release();
	_grid_vao.release();

	std::vector<GLfloat> l = {
		0.5, 0.,
		0.5, 1.,
		0., 0.5,
		1., 0.5
	};

	_line_vao.bind();
	_line_buffer.create();
	_line_buffer.bind();
	_line_buffer.allocate(l.data(), l.size()*sizeof (GLfloat));
	_line_buffer.setUsagePattern(QOpenGLBuffer::StaticDraw);

	_line_buffer.release();
	_line_vao.release();

	std::vector<GLfloat> f = {
		0.,0.,
		0.,static_cast<GLfloat>(s.height()),
		static_cast<GLfloat>(s.width()),static_cast<GLfloat>(s.height()),
		static_cast<GLfloat>(s.width()),0.
	};

	_frame_vao.bind();
	_frame_buffer.create();
	_frame_buffer.bind();
	_frame_buffer.allocate(f.data(), f.size()*sizeof (GLfloat));
	_frame_buffer.setUsagePattern(QOpenGLBuffer::StaticDraw);

	_frame_buffer.release();
	_frame_vao.release();
}

void LensDistortionViewer::onCamChanged() {
	update();
}
void LensDistortionViewer::activeCameraDestroyed() {
	_cam = nullptr;
	update();
}

void LensDistortionViewer::setFrameTransform(int w, int h) {

	if (_cam == nullptr) {
		return;
	}

	QSize s = _cam->imSize();

	_frame_transform.setToIdentity();

	float c_ar = static_cast<float>(s.width())/static_cast<float>(s.height());
	float w_ar = static_cast<float>(w)/static_cast<float>(h);

	float scale_x = (c_ar > w_ar) ? 1/static_cast<float>(s.width()) : (c_ar/w_ar)/static_cast<float>(s.width());
	float scale_y = (c_ar > w_ar) ? (w_ar/c_ar)/static_cast<float>(s.height()) : 1/static_cast<float>(s.height());
	_frame_transform.scale(1.99*scale_x, -1.99*scale_y);
	_frame_transform.translate(-static_cast<float>(s.width())/2., -static_cast<float>(s.height())/2.);
}

} // namespace StereoVisionApp
