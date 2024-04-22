#include "opengldrawabletrajectory.h"

#include "datablocks/trajectory.h"

#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>

namespace StereoVisionApp {

OpenGlDrawableTrajectory::OpenGlDrawableTrajectory(StereoVisionApp::OpenGl3DSceneViewWidget* parent) :
    StereoVisionApp::OpenGlDrawable(parent),
    _has_data(false),
    _segment_start(0),
    _segment_end(1000)
{

}

void OpenGlDrawableTrajectory::initializeGL() {

    _scene_vao.create();

    _traj_buffer.create();
    _traj_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    _idx_buffer.create();
    _idx_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    _trajectoryProgram = new QOpenGLShaderProgram();
    _trajectoryProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/trajectoryViewerLine.vert");
    _trajectoryProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/trajectoryViewerLine.frag");

    _trajectoryProgram->link();

}
void OpenGlDrawableTrajectory::paintGL(QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView) {

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    QOpenGLExtraFunctions * ef = QOpenGLContext::currentContext()->extraFunctions();

    if (_has_data) {

        int vertexLocation;
        int idxsLocation;

        if (!_traj_pos.empty()) {

            vertexLocation = _trajectoryProgram->attributeLocation("in_location");
            idxsLocation = _trajectoryProgram->attributeLocation("in_id");

            _trajectoryProgram->bind();
            _scene_vao.bind();
            _traj_buffer.bind();

            bool hasToReload = _has_to_reset_gl_buffers;

            if (_has_to_reset_gl_buffers) {
                _traj_buffer.allocate(_traj_pos.data(), _traj_pos.size()*sizeof (GLfloat));

                _has_to_reset_gl_buffers = false;
            }

            _trajectoryProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);
            _trajectoryProgram->enableAttributeArray(vertexLocation);

            _idx_buffer.bind();

            if (hasToReload) {
                _idx_buffer.allocate(_traj_idxs.data(), _traj_idxs.size()*sizeof (GLint));
            }

            ef->glVertexAttribIPointer(idxsLocation, 1, GL_INT, 0, 0);
            //_trajectoryProgram->setAttributeBuffer(idxsLocation, GL_FLOAT, 0, 1);
            _trajectoryProgram->enableAttributeArray(idxsLocation);

            _trajectoryProgram->setUniformValue("matrixViewProjection", projectionView*modelView);
            _trajectoryProgram->setUniformValue("sceneScale", _sceneScale);

            _trajectoryProgram->setUniformValue("segmentStart", _segment_start);
            _trajectoryProgram->setUniformValue("segmentEnd", _segment_end);

            _trajectoryProgram->setUniformValue("baseColor", _baseColor);
            _trajectoryProgram->setUniformValue("segmentColor", _highlightSegmentColor);

            f->glDrawArrays(GL_LINE_STRIP, 0, _traj_idxs.size());

            _scene_vao.release();
            _traj_buffer.release();

            _trajectoryProgram->disableAttributeArray(vertexLocation);
            _trajectoryProgram->release();

        }
    }
}

void OpenGlDrawableTrajectory::clearViewRessources() {

    if (_trajectoryProgram != nullptr) {
        delete _trajectoryProgram;
    }

    if (_traj_buffer.isCreated()) {
        _traj_buffer.destroy();
    }
    _scene_vao.destroy();
}

void OpenGlDrawableTrajectory::setTrajectory(const Trajectory * trajectory) {

    std::vector<Eigen::Vector3f> trajData = trajectory->loadTrajectoryPathInProjectLocalFrame(); //get the trajectory
    setTrajectory(trajData);

}

/*!
 * \brief OpenGlDrawableTrajectory::setTrajectory set the trajectory that is displayed
 * \param trajectory the trajectory, as a series of body to world transforms
 */
void OpenGlDrawableTrajectory::setTrajectory(std::vector<StereoVision::Geometry::AffineTransform<float>> const& trajectory) {

    _segment_end = trajectory.size();

    _traj_pos.clear();
    _traj_pos.resize(trajectory.size()*3);

    _traj_idxs.clear();
    _traj_idxs.resize(trajectory.size());

    int i = 0;
    int j = 0;
    for (StereoVision::Geometry::AffineTransform<float> pose : trajectory) {
        _traj_pos[i++] = pose.t[0];
        _traj_pos[i++] = pose.t[1];
        _traj_pos[i++] = pose.t[2];

        _traj_idxs[j] = j;
        j++;
    }

    _has_data = true;
    _has_to_reset_gl_buffers = true;

    Q_EMIT updateRequested();
}

/*!
 * \brief OpenGlDrawableTrajectory::setTrajectory set the trajectory that is displayed
 * \param trajectory the trajectory, as a series of positions
 */
void OpenGlDrawableTrajectory::setTrajectory(const std::vector<Eigen::Vector3f> &trajectory) {

    _segment_end = trajectory.size();

    _traj_pos.clear();
    _traj_pos.resize(trajectory.size()*3);

    _traj_idxs.clear();
    _traj_idxs.resize(trajectory.size());

    int i = 0;
    int j = 0;
    for (Eigen::Vector3f pos : trajectory) {
        _traj_pos[i++] = pos[0];
        _traj_pos[i++] = pos[1];
        _traj_pos[i++] = pos[2];

        _traj_idxs[j] = j;
        j++;
    }

    _has_data = true;
    _has_to_reset_gl_buffers = true;

    Q_EMIT updateRequested();
}

void OpenGlDrawableTrajectory::clearTrajectory() {
    _traj_pos.clear();
    _traj_idxs.clear();

    _has_data = false;
    _has_to_reset_gl_buffers = true;

    Q_EMIT updateRequested();
}

void OpenGlDrawableTrajectory::setSceneScale(float newSceneScale) {
    _sceneScale = newSceneScale;
}

void OpenGlDrawableTrajectory::setSegmentStart(float newSegment_start)
{
    if (newSegment_start != _segment_start) {
        _segment_start = newSegment_start;
        updateRequested();
    }
}

void OpenGlDrawableTrajectory::setSegmentEnd(float newSegment_end)
{
    if (newSegment_end != _segment_end) {
        _segment_end = newSegment_end;
        updateRequested();
    }
}

const QColor &OpenGlDrawableTrajectory::baseColor() const
{
    return _baseColor;
}

void OpenGlDrawableTrajectory::setBaseColor(const QColor &newBaseColor)
{
    _baseColor = newBaseColor;
}

const QColor &OpenGlDrawableTrajectory::highlightSegmentColor() const
{
    return _highlightSegmentColor;
}

void OpenGlDrawableTrajectory::setHighlightSegmentColor(const QColor &newHighlightSegmentColor)
{
    _highlightSegmentColor = newHighlightSegmentColor;
}

} // namespace StereoVisionApp
