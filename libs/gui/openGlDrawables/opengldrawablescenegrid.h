#ifndef STEREOVISIONAPP_OPENGLDRAWABLESCENEGRID_H
#define STEREOVISIONAPP_OPENGLDRAWABLESCENEGRID_H

#include "../opengl3dsceneviewwidget.h"

namespace StereoVisionApp {

class OpenGlDrawableSceneGrid : public OpenGlDrawable
{
    Q_OBJECT
public:
    OpenGlDrawableSceneGrid(OpenGl3DSceneViewWidget* parent);
    ~OpenGlDrawableSceneGrid();

    void initializeGL() override;
    void paintGL(QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView) override;
    void clearViewRessources() override;

    float gridDistance() const;
    void setGridDistance(float newGridDistance);

    int gridSplits() const;
    void setGridSplits(int newGridSplits);

protected:

    void generateGrid();

    float _gridDistance;
    int _gridSplits;

    QOpenGLVertexArrayObject _grid_vao;
    QOpenGLBuffer _grid_buffer;
    QOpenGLShaderProgram* _gridProgram;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_OPENGLDRAWABLESCENEGRID_H
