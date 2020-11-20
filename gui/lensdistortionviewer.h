#ifndef STEREOVISIONAPP_LENSDISTORTIONVIEWER_H
#define STEREOVISIONAPP_LENSDISTORTIONVIEWER_H

#include <QMatrix4x4>
#include <QOpenGLWidget>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>

class QOpenGLShaderProgram;

namespace StereoVisionApp {

class Camera;

class LensDistortionViewer : public QOpenGLWidget
{
	Q_OBJECT
public:
	explicit LensDistortionViewer(QWidget *parent = nullptr);

	~LensDistortionViewer();

	void setCamera(Camera* cam);
	void useOptimizedValues(bool useOptimizedValues);

signals:

protected:

	void initializeGL() override;
	void paintGL() override;
	void resizeGL(int w, int h) override;

	void generateVertices();

	//void wheelEvent(QWheelEvent *event) override;

private:

	void onCamChanged();
	void activeCameraDestroyed();

	void setFrameTransform(int w, int h);

	Camera* _cam;
	bool _useOptimizedValues;

	int _n_pt_w;
	int _n_pt_t;

	QMatrix4x4 _frame_transform;

	QOpenGLVertexArrayObject _grid_vao;
	QOpenGLVertexArrayObject _line_vao;
	QOpenGLVertexArrayObject _frame_vao;

	QOpenGLBuffer _grid_buffer;
	QOpenGLBuffer _line_buffer;
	QOpenGLBuffer _frame_buffer;

	QOpenGLShaderProgram* _gridProgram;
	QOpenGLShaderProgram* _lineProgram;
	QOpenGLShaderProgram* _frameProgram;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LENSDISTORTIONVIEWER_H
