#ifndef STEREOVISIONAPP_OPENGLDRAWABLELANDMARKSET_H
#define STEREOVISIONAPP_OPENGLDRAWABLELANDMARKSET_H

#include "../opengl3dsceneviewwidget.h"
#include "../sparsealignementviewer.h"

namespace StereoVisionApp {

class OpenGlDrawableLandmarkSet : public OpenGlDrawable
{
    Q_OBJECT
public:
    OpenGlDrawableLandmarkSet(OpenGl3DSceneViewWidget* parent);
    ~OpenGlDrawableLandmarkSet();

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

protected:

    bool _has_been_initialised;

    AbstractSparseAlignementDataInterface* _currentInterface;

    float _sceneScale;

    QOpenGLVertexArrayObject _scene_vao;
    QOpenGLVertexArrayObject _scene_ids_vao;

    QOpenGLBuffer _lm_pos_buffer;
    QOpenGLBuffer _lm_pos_id_buffer;

    QOpenGLShaderProgram* _landMarkPointProgram;
    QOpenGLShaderProgram* _objIdProgram;

    std::vector<GLfloat> _llm_pos;
    std::vector<qint64> _llm_id;
    bool _hasToReloadLandmarks;
    bool _hasToReloadLandmarksIds;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_OPENGLDRAWABLELANDMARKSET_H
