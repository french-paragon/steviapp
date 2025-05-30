#include "opengldrawabletrajectory.h"

#include "datablocks/trajectory.h"

#include "sparsealignementviewer.h"

#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>

namespace StereoVisionApp {

const int OpenGlDrawableTrajectory::MAX_TRAJ_SAMPLE = 50000;

OpenGlDrawableTrajectory::OpenGlDrawableTrajectory(StereoVisionApp::OpenGl3DSceneViewWidget* parent) :
    StereoVisionApp::OpenGlDrawable(parent),
    _has_data(false),
    _segment_start(-1),
    _segment_end(-1),
    _sceneScale(1),
    _handleScale(1)
{

    //default colors
    _baseColor = QColor(50,100,250);
    _highlightSegmentColor = QColor(250,150,100);

    _orientHandleXColor = QColor(255,0,0);
    _orientHandleYColor = QColor(0,255,0);
    _orientHandleZColor = QColor(0,0,255);

    //init the handle model:

    _orient_steps_pos.resize(18);
    _orient_steps_idxs.resize(6);

    constexpr float baseScale = 5;

    //x axis
    _orient_steps_pos[0] = 1*baseScale;
    _orient_steps_pos[1] = -1*baseScale;
    _orient_steps_pos[2] = -1*baseScale;
    _orient_steps_pos[3] = -1*baseScale;
    _orient_steps_pos[4] = -1*baseScale;
    _orient_steps_pos[5] = -1*baseScale;

    _orient_steps_idxs[0] = 0;
    _orient_steps_idxs[1] = 0;

    //y axis
    _orient_steps_pos[6] = -1*baseScale;
    _orient_steps_pos[7] = 1*baseScale;
    _orient_steps_pos[8] = -1*baseScale;
    _orient_steps_pos[9] = -1*baseScale;
    _orient_steps_pos[10] = -1*baseScale;
    _orient_steps_pos[11] = -1*baseScale;

    _orient_steps_idxs[2] = 1;
    _orient_steps_idxs[3] = 1;

    //z axis
    _orient_steps_pos[12] = -1*baseScale;
    _orient_steps_pos[13] = -1*baseScale;
    _orient_steps_pos[14] = 1*baseScale;
    _orient_steps_pos[15] = -1*baseScale;
    _orient_steps_pos[16] = -1*baseScale;
    _orient_steps_pos[17] = -1*baseScale;

    _orient_steps_idxs[4] = 2;
    _orient_steps_idxs[5] = 2;

}

void OpenGlDrawableTrajectory::initializeGL() {

    _scene_vao.create();

    _traj_buffer.create();
    _traj_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    _idx_buffer.create();
    _idx_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    _handle_buffer.create();
    _handle_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    _handle_buffer.bind();

    _handle_buffer.allocate(_orient_steps_pos.data(), _orient_steps_pos.size()*sizeof (GLfloat));

    _handle_idx_buffer.create();
    _handle_idx_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    _handle_idx_buffer.bind();

    _handle_idx_buffer.allocate(_orient_steps_idxs.data(), _orient_steps_idxs.size()*sizeof (GLfloat));

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
            _trajectoryProgram->setUniformValue("handleScale", _handleScale);

            _trajectoryProgram->setUniformValue("segmentStart", _segment_start);
            _trajectoryProgram->setUniformValue("segmentEnd", _segment_end);

            _trajectoryProgram->setUniformValue("baseColor", _baseColor);
            _trajectoryProgram->setUniformValue("segmentColor", _highlightSegmentColor);

            _trajectoryProgram->setUniformValue("orientHandleXColor", _orientHandleXColor);
            _trajectoryProgram->setUniformValue("orientHandleYColor", _orientHandleYColor);
            _trajectoryProgram->setUniformValue("orientHandleZColor", _orientHandleZColor);

            _trajectoryProgram->setUniformValue("mode", 0); //trajectory mode

            f->glDrawArrays(GL_LINE_STRIP, 0, _traj_idxs.size());

            _trajectoryProgram->setUniformValue("mode", 1); //orientation handles mode


            _handle_buffer.bind();

            _trajectoryProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);
            _trajectoryProgram->enableAttributeArray(vertexLocation);

            _handle_idx_buffer.bind();

            ef->glVertexAttribIPointer(idxsLocation, 1, GL_INT, 0, 0);
            _trajectoryProgram->enableAttributeArray(idxsLocation);

            for (QMatrix4x4 pose : _traj_orient_steps) {
                _trajectoryProgram->setUniformValue("matrixHandleTransform", pose);
                f->glDrawArrays(GL_LINES, 0, _orient_steps_idxs.size());
            }

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

void OpenGlDrawableTrajectory::setTrajectory(const Trajectory * trajectory, bool optimized, int orientationHandles) {

    StatusOptionalReturn<std::vector<StereoVision::Geometry::AffineTransform<float>>> trajData = trajectory->loadTrajectoryInProjectLocalFrame(optimized); //get the trajectory
    if (!trajData.isValid()) {
        return; //TODO: propagate message
    }
    setTrajectory(trajData.value(), orientationHandles);

}

/*!
 * \brief OpenGlDrawableTrajectory::setTrajectory set the trajectory that is displayed
 * \param trajectory the trajectory, as a series of body to world transforms
 */
void OpenGlDrawableTrajectory::setTrajectory(std::vector<StereoVision::Geometry::AffineTransform<float>> const& trajectory, int orientationHandles) {

    //_segment_end = trajectory.size();

    _traj_orient_steps.clear();

    size_t size = std::min<size_t>(MAX_TRAJ_SAMPLE, trajectory.size());

    _traj_pos.clear();
    _traj_pos.resize(size*3);

    _traj_idxs.clear();
    _traj_idxs.resize(size);

    if (trajectory.empty()) {

        _has_data = true;
        _has_to_reset_gl_buffers = true;

        Q_EMIT updateRequested();

        return;
    }

    int i = 0;
    int j = 0;

    float minX = std::numeric_limits<float>::infinity();
    float maxX = -std::numeric_limits<float>::infinity();

    float minY = std::numeric_limits<float>::infinity();
    float maxY = -std::numeric_limits<float>::infinity();

    float minZ = std::numeric_limits<float>::infinity();
    float maxZ = -std::numeric_limits<float>::infinity();

    for (size_t idx = 0; idx < size; idx++) {

        double percent = double(idx)/std::max<int>(1,size - 1);
        double prog = (trajectory.size()-1)*percent;
        size_t trajIdx = std::round(prog);

        if (trajIdx < 0) {
            trajIdx = 0;
        }

        if (trajIdx >= trajectory.size()) {
            trajIdx = trajectory.size()-1;
        }

        StereoVision::Geometry::AffineTransform<float> const& pose = trajectory[trajIdx];

        minX = std::min(minX, pose.t[0]);
        maxX = std::max(maxX, pose.t[0]);
        minY = std::min(minY, pose.t[1]);
        maxY = std::max(maxY, pose.t[1]);
        minZ = std::min(minZ, pose.t[2]);
        maxZ = std::max(maxZ, pose.t[2]);

        _traj_pos[i++] = pose.t[0];
        _traj_pos[i++] = pose.t[1];
        _traj_pos[i++] = pose.t[2];

        _traj_idxs[j] = j;
        j++;
    }

    for (int i = 0; i < orientationHandles; i++) {
        int orientIdx = i*float(trajectory.size()-1)/std::max(1,orientationHandles);

        StereoVision::Geometry::AffineTransform<float> const& transform = trajectory[orientIdx];

        Eigen::Matrix3f const& R = transform.R;
        Eigen::Vector3f const& t = transform.t;

        QMatrix4x4 matr(
                    R(0,0), R(0,1), R(0,2), t[0],
                    R(1,0), R(1,1), R(1,2), t[1],
                    R(2,0), R(2,1), R(2,2), t[2],
                    0,      0,      0,      1);

        _traj_orient_steps.push_back(matr);
    }

    float rangeX = maxX - minX;
    float rangeY = maxY - minY;
    float rangeZ = maxZ - minZ;

    float maxRange = std::max(std::max(std::max(rangeX, rangeY), rangeZ),1.f);
    if (!std::isfinite(maxRange or maxRange < 1)) {
        maxRange = 1;
    }

    StereoVisionApp::SparseAlignementViewer* parentWidget =
        qobject_cast<StereoVisionApp::SparseAlignementViewer*>(parent());

    if (parentWidget != nullptr) {
        parentWidget->setSceneScale(20/maxRange);
        parentWidget->setCamScale(maxRange/200);
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

    //_segment_end = trajectory.size();

    _traj_orient_steps.clear();

    size_t size = std::min<size_t>(MAX_TRAJ_SAMPLE, trajectory.size());

    _traj_pos.clear();
    _traj_pos.resize(size*3);

    _traj_idxs.clear();
    _traj_idxs.resize(size);

    if (trajectory.empty()) {

        _has_data = true;
        _has_to_reset_gl_buffers = true;

        _segment_start = 0;

        Q_EMIT updateRequested();

        return;
    }

    int i = 0;
    int j = 0;

    float minX = std::numeric_limits<float>::infinity();
    float maxX = -std::numeric_limits<float>::infinity();

    float minY = std::numeric_limits<float>::infinity();
    float maxY = -std::numeric_limits<float>::infinity();

    float minZ = std::numeric_limits<float>::infinity();
    float maxZ = -std::numeric_limits<float>::infinity();

    for (size_t idx = 0; idx < size; idx++) {

        double percent = (size - 1)/double(idx);
        double prog = trajectory.size()*percent;
        size_t trajIdx = std::round(prog);

        if (trajIdx < 0) {
            trajIdx = 0;
        }

        if (trajIdx >= trajectory.size()) {
            trajIdx = trajectory.size()+1;
        }

        Eigen::Vector3f const& pose = trajectory[trajIdx];

        minX = std::min(minX, pose[0]);
        maxX = std::max(maxX, pose[0]);
        minY = std::min(minY, pose[1]);
        maxY = std::max(maxY, pose[1]);
        minZ = std::min(minZ, pose[2]);
        maxZ = std::max(maxZ, pose[2]);

        _traj_pos[i++] = pose[0];
        _traj_pos[i++] = pose[1];
        _traj_pos[i++] = pose[2];

        _traj_idxs[j] = j;
        j++;
    }

    float rangeX = maxX - minX;
    float rangeY = maxY - minY;
    float rangeZ = maxZ - minZ;

    float maxRange = std::max(std::max(std::max(rangeX, rangeY), rangeZ),1.f);
    if (!std::isfinite(maxRange or maxRange < 1)) {
        maxRange = 1;
    }

    StereoVisionApp::SparseAlignementViewer* parentWidget =
        qobject_cast<StereoVisionApp::SparseAlignementViewer*>(parent());

    if (parentWidget != nullptr) {
        parentWidget->setSceneScale(20/maxRange);
        parentWidget->setCamScale(maxRange/200);
    }

    _has_data = true;
    _has_to_reset_gl_buffers = true;

    _segment_start = 0;
    //_segment_end = _traj_idxs.size();

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

void OpenGlDrawableTrajectory::setHandleScale(float newHandleScale) {
    _handleScale = newHandleScale;
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

const QColor &OpenGlDrawableTrajectory::orientHandleColor(StereoVision::Geometry::Axis axis) const {

    switch (axis) {
    case StereoVision::Geometry::Axis::X :
        return _orientHandleXColor;
    case StereoVision::Geometry::Axis::Y :
        return _orientHandleYColor;
    case StereoVision::Geometry::Axis::Z :
        return _orientHandleZColor;
    }

    return _orientHandleXColor;
}
void OpenGlDrawableTrajectory::setOrientHandleColor(StereoVision::Geometry::Axis axis, const QColor &newHandleColor) {
    switch (axis) {
    case StereoVision::Geometry::Axis::X :
        _orientHandleXColor = newHandleColor;
        break;
    case StereoVision::Geometry::Axis::Y :
        _orientHandleYColor = newHandleColor;
        break;
    case StereoVision::Geometry::Axis::Z :
        _orientHandleZColor = newHandleColor;
        break;
    }
}

} // namespace StereoVisionApp
