#ifndef STEREOVISIONAPP_OPENGL3DSCENEVIEWWIDGET_H
#define STEREOVISIONAPP_OPENGL3DSCENEVIEWWIDGET_H

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

class OpenGl3DSceneViewWidget;

class OpenGlDrawable : public QObject
{
    Q_OBJECT
public:
    explicit OpenGlDrawable(OpenGl3DSceneViewWidget* parent);
    virtual ~OpenGlDrawable();

    virtual void initializeGL() = 0;
    virtual void paintGL(QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView) = 0;
    virtual void clearViewRessources() = 0;

    virtual void initializeObjectIdMaskPart();
    virtual void paintObjectIdMask(int drawableCode, QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView);
    virtual void clearObjectsIdsRessources();

    virtual void processClick(int code);
    virtual void processHoover(int code);

Q_SIGNALS:

    void updateRequested();
};

class OpenGl3DSceneViewWidget : public QOpenGLWidget
{
    Q_OBJECT
public:
    explicit OpenGl3DSceneViewWidget(QWidget *parent = nullptr);
    ~OpenGl3DSceneViewWidget();

    void addDrawable(OpenGlDrawable* drawable);
    void removeDrawable(OpenGlDrawable* drawable);

    void resetView();

    void zoomIn(float steps = 1);
    void zoomOut(float steps = 1);

    void rotateZenith(float degrees);
    void rotateAzimuth(float degrees);

    void translateView(float deltaX, float deltaY);

    void saveViewpoint();
    void saveViewpoint(QString file);

    void loadViewpoint();
    void loadViewpoint(QString file);

protected:

    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;

    void initializeObjectIdMaskPart();
    void paintObjectIdMask();

    void generateFullSurfaceGrid();

    void setView(int w, int h);
    void resetView(int w, int h);

    void wheelEvent(QWheelEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

    struct IdPassInfos {
        int itemType;
        qint64 itemId;
    };

    IdPassInfos idPassAtPos(int x, int y);

    virtual void processVoidClick();

    float _view_distance;
    float _min_view_distance;
    float _max_view_distance;

    float _zenith_angle;
    float _azimuth_angle;

    float _x_delta;
    float _y_delta;

    bool _isInitialized;

private:

    int _nextAvailableDrawableId;
    QVector<OpenGlDrawable*> _drawables;
    QMap<OpenGlDrawable*, int> _drawableCodeMap;
    QMap<int, OpenGlDrawable*> _inverseDrawableCodeMap;

    Qt::MouseButtons _previously_pressed;
    QPoint _motion_origin_pos;

    float _landMarkPtRadius;

    QMatrix4x4 _modelView;
    QMatrix4x4 _projectionView;

    //object id raycasting variables
    //those values are used to render (offscreen) an object id pass and other informations for mouse based interactions
    Multidim::Array<float, 3>* _id_img;

    QOpenGLContext* _obj_raycasting_context;
    QOffscreenSurface* _obj_raycasting_surface;
    QOpenGLFramebufferObject* _obj_raycasting_fbo;


    bool _has_objectid_parts_initialised;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_OPENGL3DSCENEVIEWWIDGET_H
