#ifndef STEREOVISIONAPP_SPARSEALIGNEMENTVIEWER_H
#define STEREOVISIONAPP_SPARSEALIGNEMENTVIEWER_H

#include <QVector>
#include <QMatrix4x4>
#include <QOpenGLWidget>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>

#include <MultidimArrays/MultidimArrays.h>

#include "./opengl3dsceneviewwidget.h"

class QOpenGLShaderProgram;
class QOffscreenSurface;
class QOpenGLFramebufferObject;

namespace StereoVisionApp {

class Project;

class AbstractSparseAlignementDataInterface : public QObject
{
	Q_OBJECT
public:
	explicit AbstractSparseAlignementDataInterface(QObject* parent = nullptr);

    virtual int nCameras() const = 0;
    virtual int nPoints() const = 0;
    virtual int nLocalSystems() const = 0;

	virtual QMatrix4x4 getCameraTransform(int idx) const = 0;
    virtual QMatrix4x4 getLocalSystemTransform(int idx) const = 0;
	virtual QVector3D getPointPos(int idx) const = 0;

	virtual void reload() = 0;

    virtual void hooverPoint(int idx) const = 0;
    virtual void hooverCam(int idx) const = 0;
    virtual void hooverLocalCoord(int idx) const = 0;

    virtual void clickPoint(int idx) const = 0;
    virtual void clickCam(int idx) const = 0;
    virtual void clickLocalCoordinateSystem(int idx) const = 0;

Q_SIGNALS:

	void dataChanged();

	void sendStatusMessage(QString msg) const;

protected:

	friend class SparseAlignementViewer;

};

class ProjectSparseAlignementDataInterface : public AbstractSparseAlignementDataInterface
{
	Q_OBJECT
public:
	explicit ProjectSparseAlignementDataInterface(QObject* parent = nullptr);

	void setProject(Project* p);
	void clearProject();

	int nCameras() const override;
	int nPoints() const override;
    int nLocalSystems() const override;

	QMatrix4x4 getCameraTransform(int idx) const override;
    QMatrix4x4 getLocalSystemTransform(int idx) const override;
	QVector3D getPointPos(int idx) const override;

	void reload() override;

    void hooverPoint(int idx) const override;
    void hooverCam(int idx) const override;
    void hooverLocalCoord(int idx) const override;

    void clickPoint(int idx) const override;
    void clickCam(int idx) const override;
    void clickLocalCoordinateSystem(int idx) const override;

protected:

    void reloadCache();

	Project* _currentProject;
	QVector<qint64> _loadedLandmarks;
	QVector<qint64> _loadedFrames;
    QVector<qint64> _loadedLocalCoordinateSystems;
};

class OpenGlDrawableSceneGrid;
class OpenGlDrawableLandmarkSet;
class OpenGlDrawableCamerasSet;
class OpenGlDrawableLocalCoordinateSystem;

class SparseAlignementViewer : public OpenGl3DSceneViewWidget
{
	Q_OBJECT
public:
	explicit SparseAlignementViewer(QWidget *parent = nullptr);

	~SparseAlignementViewer();

	void setInterface(AbstractSparseAlignementDataInterface* i);
    void clearInterface();

    void setSceneScale(float sceneScale);
    void setCamScale(float camScale);

    void scaleCamerasIn(float steps = 1);
    void scaleCamerasOut(float steps = 1);

    void scaleSceneIn(float steps = 1);
    void scaleSceneOut(float steps = 1);

    void saveViewpoint();
    void saveViewpoint(QString file);

    void loadViewpoint();
    void loadViewpoint(QString file);

    void reloadLandmarks();
    void clearLandmarks();

Q_SIGNALS:

	void sendStatusMessage(QString msg);

protected:

    void processVoidClick() override;

    AbstractSparseAlignementDataInterface* _currentInterface;

    OpenGlDrawableSceneGrid* _drawableGrid;
    OpenGlDrawableLandmarkSet* _drawableLandmarks;
    OpenGlDrawableCamerasSet* _drawableCameras;
    OpenGlDrawableLocalCoordinateSystem* _drawableLocalSystems;

    float _sceneScale;
    float _camScale;

    void wheelEvent(QWheelEvent *event) override;

private:

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_SPARSEALIGNEMENTVIEWER_H
