#ifndef STEREOVISIONAPP_OPENGLDRAWABLELOCALCOORDINATESYSTEM_H
#define STEREOVISIONAPP_OPENGLDRAWABLELOCALCOORDINATESYSTEM_H

#include "../opengl3dsceneviewwidget.h"
#include "../sparsealignementviewer.h"

namespace StereoVisionApp {

class OpenGlDrawableLocalCoordinateSystem : public OpenGlDrawable
{
public:
    OpenGlDrawableLocalCoordinateSystem(OpenGl3DSceneViewWidget* parent);
    ~OpenGlDrawableLocalCoordinateSystem();

    void initializeGL() override;
    void paintGL(QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView) override;
    void clearViewRessources() override;

    void initializeObjectIdMaskPart() override;
    void paintObjectIdMask(int drawableId, QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView) override;
    void clearObjectsIdsRessources() override;

    void processClick(int code) override;

    void setInterface(AbstractSparseAlignementDataInterface* i);
    void clearInterface();

    void setSceneScale(float sceneScale);
    void setCoordinatesScale(float lsScale);

protected:

    void generateLocalCoordinatesModel();

    AbstractSparseAlignementDataInterface* _currentInterface;

    float _sceneScale;
    float _lsScale;

    QOpenGLShaderProgram* _localSystemProgram;
    QOpenGLShaderProgram* _objFixedIdProgram;

    QOpenGLVertexArrayObject _ls_vao;
    QOpenGLVertexArrayObject _ls_ids_vao;

    QOpenGLBuffer _ls_buffer;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_OPENGLDRAWABLELOCALCOORDINATESYSTEM_H
