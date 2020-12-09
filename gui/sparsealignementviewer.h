#ifndef STEREOVISIONAPP_SPARSEALIGNEMENTVIEWER_H
#define STEREOVISIONAPP_SPARSEALIGNEMENTVIEWER_H

#include <QVector>
#include <QMatrix4x4>
#include <QOpenGLWidget>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>

class QOpenGLShaderProgram;

namespace StereoVisionApp {

class Project;

class SparseAlignementViewer : public QOpenGLWidget
{
	Q_OBJECT
public:
	explicit SparseAlignementViewer(QWidget *parent = nullptr);

	~SparseAlignementViewer();

	void setProject(Project* p);
	void clearProject();

	void resetView();

	void zoomIn(float steps = 1);
	void zoomOut(float steps = 1);

	void scaleCamerasIn(float steps = 1);
	void scaleCamerasOut(float steps = 1);

	void rotateZenith(float degrees);
	void rotateAzimuth(float degrees);

	void reloadLandmarks();
	void clearLandmarks();

	void setCamScale(float camScale);

signals:

protected:

	void loadLandmarkImpl();

	void initializeGL() override;
	void paintGL() override;
	void resizeGL(int w, int h) override;

	void generateGrid();
	void generateCamModel();
	void setView(int w, int h);
	void resetView(int w, int h);

	void wheelEvent(QWheelEvent *event) override;
	void keyPressEvent(QKeyEvent *event) override;
	void mousePressEvent(QMouseEvent *event) override;
	void mouseReleaseEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;

private:

	bool _has_been_initialised;

	Qt::MouseButtons _previously_pressed;
	QPoint _motion_origin_pos;

	float _view_distance;
	float _min_view_distance;
	float _max_view_distance;

	float _zenith_angle;
	float _azimuth_angle;

	float _gridDistance;
	int _gridSplits;

	float _landMarkPtRadius;

	float _camScale;

	QMatrix4x4 _projectionView;

	QOpenGLVertexArrayObject _grid_vao;
	QOpenGLVertexArrayObject _scene_vao;
	QOpenGLVertexArrayObject _cam_vao;

	QOpenGLBuffer _grid_buffer;
	QOpenGLBuffer _lm_pos_buffer;
	QOpenGLBuffer _cam_buffer;
	QOpenGLBuffer _cam_indices;

	QOpenGLShaderProgram* _gridProgram;
	QOpenGLShaderProgram* _landMarkPointProgram;
	QOpenGLShaderProgram* _camProgram;

	Project* _currentProject;
	QVector<qint64> _loadedLandmarks;
	std::vector<GLfloat> _llm_pos;
	bool _hasToReloadLandmarks;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_SPARSEALIGNEMENTVIEWER_H
