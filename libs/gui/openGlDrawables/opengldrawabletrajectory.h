#ifndef STEREOVISIONAPP_OPENGLDRAWABLETRAJECTORY_H
#define STEREOVISIONAPP_OPENGLDRAWABLETRAJECTORY_H

#include "../opengl3dsceneviewwidget.h"

#include <StereoVision/geometry/rotations.h>

namespace StereoVisionApp {

class Trajectory;

class OpenGlDrawableTrajectory : public StereoVisionApp::OpenGlDrawable
{
    Q_OBJECT
public:
    OpenGlDrawableTrajectory(StereoVisionApp::OpenGl3DSceneViewWidget* parent = nullptr);

    void initializeGL();
    void paintGL(QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView);
    void clearViewRessources();

    void setTrajectory(const Trajectory * trajectory, int orientationHandles = 9);
    void setTrajectory(const std::vector<StereoVision::Geometry::AffineTransform<float> > &trajectory, int orientationHandles = 9);
    void setTrajectory(const std::vector<Eigen::Vector3f> &trajectory);
    void clearTrajectory();

    void setSceneScale(float newSceneScale);

    void setSegmentStart(float newSegment_start);
    void setSegmentEnd(float newSegment_end);

    const QColor &baseColor() const;
    void setBaseColor(const QColor &newBaseColor);

    const QColor &highlightSegmentColor() const;
    void setHighlightSegmentColor(const QColor &newHighlightSegmentColor);

protected:

    bool _has_data;
    bool _has_to_reset_gl_buffers;

    float _sceneScale;

    float _segment_start;
    float _segment_end;

    QOpenGLVertexArrayObject _scene_vao;
    QOpenGLVertexArrayObject _scene_ids_vao;

    QOpenGLBuffer _traj_buffer;
    QOpenGLBuffer _idx_buffer;

    QOpenGLBuffer _handle_buffer;
    QOpenGLBuffer _handle_idx_buffer;

    QColor _baseColor;
    QColor _highlightSegmentColor;

    QOpenGLShaderProgram* _trajectoryProgram;

    std::vector<QMatrix4x4> _traj_orient_steps; //list of orientation

    std::vector<GLfloat> _orient_steps_pos;
    std::vector<GLint> _orient_steps_idxs;

    std::vector<GLfloat> _traj_pos;
    std::vector<GLint> _traj_idxs;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_OPENGLDRAWABLETRAJECTORY_H
