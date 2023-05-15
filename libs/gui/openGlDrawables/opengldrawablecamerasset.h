#ifndef STEREOVISIONAPP_OPENGLDRAWABLECAMERASSET_H
#define STEREOVISIONAPP_OPENGLDRAWABLECAMERASSET_H

#include "../opengl3dsceneviewwidget.h"
#include "../sparsealignementviewer.h"

namespace StereoVisionApp {

class OpenGlDrawableCamerasSet : public OpenGlDrawable
{
    Q_OBJECT
public:
    OpenGlDrawableCamerasSet(OpenGl3DSceneViewWidget* parent);
    ~OpenGlDrawableCamerasSet();

    void initializeGL() override;
    void paintGL(QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView) override;
    void clearViewRessources() override;

    void initializeObjectIdMaskPart() override;
    void paintObjectIdMask(int drawableId, QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView) override;
    void clearObjectsIdsRessources() override;

    void processClick(int code) override;

    void setInterface(AbstractSparseAlignementDataInterface* i);
    void clearInterface();

    void reloadLandmarks();
    void clearLandmarks();

    void setSceneScale(float sceneScale);
    void setCamScale(float camScale);

protected:

    void generateCamModel();

    AbstractSparseAlignementDataInterface* _currentInterface;

    float _sceneScale;
    float _camScale;

    QOpenGLShaderProgram* _camProgram;
    QOpenGLShaderProgram* _objFixedIdProgram;

    QOpenGLVertexArrayObject _cam_vao;
    QOpenGLVertexArrayObject _cam_ids_vao;

    QOpenGLBuffer _cam_buffer;
    QOpenGLBuffer _cam_indices;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_OPENGLDRAWABLECAMERASSET_H
