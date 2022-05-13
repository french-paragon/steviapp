#ifndef STEREOVISIONAPP_SPARSEALIGNEMENTVIEWER_H
#define STEREOVISIONAPP_SPARSEALIGNEMENTVIEWER_H

#include <QVector>
#include <QMatrix4x4>
#include <QOpenGLWidget>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>

#include <MultidimArrays/MultidimArrays.h>

class QOpenGLShaderProgram;
class QOffscreenSurface;
class QOpenGLFramebufferObject;

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

	void scaleSceneIn(float steps = 1);
	void scaleSceneOut(float steps = 1);

	void rotateZenith(float degrees);
	void rotateAzimuth(float degrees);

	void reloadLandmarks();
	void clearLandmarks();

	void setCamScale(float camScale);
	void setSceneScale(float sceneScale);

Q_SIGNALS:

	void sendStatusMessage(QString msg);

protected:

	void loadLandmarkImpl();

	void initializeGL() override;
	void paintGL() override;
	void resizeGL(int w, int h) override;

	void initializeObjectIdMaskPart();
	void paintObjectIdMask();

	void generateGrid();
	void generateFullSurfaceGrid();
	void generateCamModel();
	void setView(int w, int h);
	void resetView(int w, int h);

	void wheelEvent(QWheelEvent *event) override;
	void keyPressEvent(QKeyEvent *event) override;
	void mousePressEvent(QMouseEvent *event) override;
	void mouseReleaseEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;

	enum class ItemTypeInfo {
		Unknown,
		Landmark,
		Camera
	};

	struct IdPassInfos {
		qint64 itemId;
		ItemTypeInfo itemType;
	};

	constexpr static float LandmarkColorCode = 1.0f;
	constexpr static float CameraColorCode = 2.0f;

	IdPassInfos idPassAtPos(int x, int y);

	struct itemClickInfos {
		qint64 itemId;
		float viewDist;
		float zDist;
	};

	itemClickInfos nearestLandmark(QPoint const& pt, int minPixDist = 5);
	itemClickInfos nearestCam(QPoint const& pt, int minPixDist = 5);

	void landmarkHover(int landMarkId);
	void frameHover(int frameId);
	void voidHover();

	void landmarkClick(int landMarkId);
	void frameClick(int frameId);
	void voidClick();

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

	float _sceneScale;
	float _camScale;

	QMatrix4x4 _modelView;
	QMatrix4x4 _projectionView;

	QOpenGLVertexArrayObject _grid_vao;
	QOpenGLVertexArrayObject _scene_vao;
	QOpenGLVertexArrayObject _cam_vao;

	QOpenGLBuffer _grid_buffer;
	QOpenGLBuffer _lm_pos_buffer;
	QOpenGLBuffer _lm_pos_id_buffer;
	QOpenGLBuffer _cam_buffer;
	QOpenGLBuffer _cam_indices;

	QOpenGLShaderProgram* _gridProgram;
	QOpenGLShaderProgram* _landMarkPointProgram;
	QOpenGLShaderProgram* _camProgram;
	QOpenGLShaderProgram* _objIdProgram;
	QOpenGLShaderProgram* _objFixedIdProgram;

	Project* _currentProject;
	QVector<qint64> _loadedLandmarks;
	QVector<qint64> _loadedFrames;
	std::vector<GLfloat> _llm_pos;
	bool _hasToReloadLandmarks;
	bool _hasToReloadLandmarksIds;

	//object id raycasting variables
	//those values are used to render (offscreen) an object id pass and other informations for mouse based interactions
	Multidim::Array<float, 3>* _id_img;

	QOpenGLContext* _obj_raycasting_context;
	QOffscreenSurface* _obj_raycasting_surface;
	QOpenGLFramebufferObject* _obj_raycasting_fbo;

	QOpenGLVertexArrayObject _scene_ids_vao;

	bool _has_objectid_parts_initialised;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_SPARSEALIGNEMENTVIEWER_H
