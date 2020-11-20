#include "sparsealignementviewer.h"

#include <cmath>

#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>

#include <QWheelEvent>

namespace StereoVisionApp {

SparseAlignementViewer::SparseAlignementViewer(QWidget *parent) :
	QOpenGLWidget(parent),
	_gridProgram(nullptr),
	_landMarkPointProgram(nullptr)
{
	_gridDistance = 10.;
	_gridSplits = 10;

	_min_view_distance = 0.01;
	_max_view_distance = 100.;

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

	if (_grid_buffer.isCreated()) {
		_grid_buffer.destroy();
	}

	if (_lm_pos_buffer.isCreated()) {
		_lm_pos_buffer.destroy();
	}

	_vao.destroy();

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

void SparseAlignementViewer::initializeGL() {

	_vao.create();

	if (_vao.isCreated()) {
		_vao.bind();
	}

	generateGrid();
	setView(width(), height());

	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
	f->glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	f->glEnable(GL_MULTISAMPLE);

	_gridProgram = new QOpenGLShaderProgram();
	_gridProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/sparseViewerPerspFloor.vert");
	_gridProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/sparseViewerGrid.frag");

	_gridProgram->link();
}

void SparseAlignementViewer::paintGL() {

	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

	setView(width(), height());

	_gridProgram->bind();
	_vao.bind();
	_grid_buffer.bind();


	int vertexLocation =  _gridProgram->attributeLocation("in_location");
	_gridProgram->enableAttributeArray(vertexLocation);
	_gridProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 2);

	_gridProgram->setUniformValue("matrixViewProjection", _projectionView);

	f->glDrawArrays(GL_LINES, 0, 8*(_gridSplits + 1));

	_vao.release();
	_grid_buffer.release();

	_gridProgram->disableAttributeArray(vertexLocation);
	_gridProgram->release();

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

	_projectionView = proj*view*model;
}

void SparseAlignementViewer::wheelEvent(QWheelEvent *e) {
	QPoint d = e->angleDelta();
	if (d.y() == 0){
		e->ignore();
		return;
	}

	if (e->buttons() == Qt::NoButton) {
		float step = d.y()/50.;
		if (step < 0) {
			zoomOut(-step);
		} else {
			zoomIn(step);
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

	if (_previously_pressed == Qt::MiddleButton) {

		_motion_origin_pos = e->pos();

		e->accept();
	} else {
		e->ignore();
	}

}
void SparseAlignementViewer::mouseReleaseEvent(QMouseEvent *e) {
	e->ignore();
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

} // namespace StereoVisionApp
