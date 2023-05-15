#include "opengldrawablelandmarkset.h"

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>

#include <cmath>

namespace StereoVisionApp {

OpenGlDrawableLandmarkSet::OpenGlDrawableLandmarkSet(OpenGl3DSceneViewWidget *parent):
    OpenGlDrawable(parent),
    _has_been_initialised(false),
    _currentInterface(nullptr),
    _sceneScale(1.)
{

}
OpenGlDrawableLandmarkSet::~OpenGlDrawableLandmarkSet() {
}

void OpenGlDrawableLandmarkSet::initializeGL() {

    _scene_vao.create();

    _lm_pos_buffer.create();
    _lm_pos_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    _lm_pos_id_buffer.create();
    _lm_pos_id_buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    reloadLandmarks();

    _landMarkPointProgram = new QOpenGLShaderProgram();
    _landMarkPointProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/sparseViewerPersp.vert");
    _landMarkPointProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/sparseViewerPoints.frag");

    _landMarkPointProgram->link();
}

void OpenGlDrawableLandmarkSet::paintGL(QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView) {

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    if (_currentInterface != nullptr) {

        int vertexLocation;

        if (!_llm_pos.empty()) {

            _landMarkPointProgram->bind();
            _scene_vao.bind();
            _lm_pos_buffer.bind();

            if (_hasToReloadLandmarks) {
                _lm_pos_buffer.allocate(_llm_pos.data(), _llm_pos.size()*sizeof (GLfloat));
                _hasToReloadLandmarks = false;
            }

            vertexLocation = _landMarkPointProgram->attributeLocation("in_location");
            _landMarkPointProgram->enableAttributeArray(vertexLocation);
            _landMarkPointProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);

            _landMarkPointProgram->setUniformValue("matrixViewProjection", projectionView*modelView);
            _landMarkPointProgram->setUniformValue("sceneScale", _sceneScale);

            f->glDrawArrays(GL_POINTS, 0, _llm_pos.size()/3);

            _scene_vao.release();
            _lm_pos_buffer.release();

            _landMarkPointProgram->disableAttributeArray(vertexLocation);
            _landMarkPointProgram->release();

        }
    }
}
void OpenGlDrawableLandmarkSet::clearViewRessources() {

    if (_landMarkPointProgram != nullptr) {
        delete _landMarkPointProgram;
    }

    if (_lm_pos_buffer.isCreated()) {
        _lm_pos_buffer.destroy();
    }
    _scene_vao.destroy();
}

void OpenGlDrawableLandmarkSet::initializeObjectIdMaskPart() {
    _scene_ids_vao.create();

    _objIdProgram = new QOpenGLShaderProgram();
    _objIdProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/objectId.vert");
    _objIdProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/objectId.frag");

    _objIdProgram->link();
}

void OpenGlDrawableLandmarkSet::paintObjectIdMask(int drawableId, QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView) {

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    if (_currentInterface != nullptr) {

        int vertexLocation;
        int idLocation;

        if (!_llm_id.empty()) {

            vertexLocation = _objIdProgram->attributeLocation("in_location");
            idLocation = _objIdProgram->attributeLocation("in_id");

            _objIdProgram->bind();
            _scene_ids_vao.bind();
            _lm_pos_buffer.bind();

            _objIdProgram->enableAttributeArray(vertexLocation);
            _objIdProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);

            _lm_pos_id_buffer.bind();

            if (_hasToReloadLandmarksIds) {
                _lm_pos_id_buffer.allocate(_llm_id.data(), _llm_id.size()* sizeof (qint64));
                _hasToReloadLandmarksIds = false;
            }

            _objIdProgram->enableAttributeArray(idLocation);
            _objIdProgram->setAttributeBuffer(idLocation, GL_FLOAT, 0, 2);

            float LandmarkColorCode;
            static_assert (sizeof(decltype (drawableId)) == sizeof(decltype (LandmarkColorCode)), "Assumption about type size broken");
            std::memcpy(&LandmarkColorCode, &drawableId, sizeof(decltype (drawableId)));

            _objIdProgram->setUniformValue("matrixViewProjection", projectionView*modelView);
            _objIdProgram->setUniformValue("matrixObjToScene", QMatrix4x4());
            _objIdProgram->setUniformValue("sceneScale", _sceneScale);
            _objIdProgram->setUniformValue("pointScale", 10.0f);
            _objIdProgram->setUniformValue("typeColorIndex", LandmarkColorCode);

            f->glDrawArrays(GL_POINTS, 0, _llm_id.size());

            _lm_pos_buffer.release();
            _lm_pos_id_buffer.release();
            _scene_ids_vao.release();

            _objIdProgram->disableAttributeArray(vertexLocation);
            _objIdProgram->release();

        }
    }
}
void OpenGlDrawableLandmarkSet::clearObjectsIdsRessources() {

    if (_objIdProgram != nullptr) {
        delete _objIdProgram;
    }

    _scene_ids_vao.destroy();
}

void OpenGlDrawableLandmarkSet::processClick(int code) {

    if (_currentInterface != nullptr) {

        _currentInterface->clickPoint(code);

    }
}

void OpenGlDrawableLandmarkSet::setInterface(AbstractSparseAlignementDataInterface* i) {

    if (i == _currentInterface) {
        return;
    }

    clearInterface();

    _currentInterface = i;
    connect(i, &AbstractSparseAlignementDataInterface::dataChanged, this, &OpenGlDrawableLandmarkSet::reloadLandmarks);

    if (_has_been_initialised) {
        reloadLandmarks();
    }
}
void OpenGlDrawableLandmarkSet::clearInterface() {

    if (_currentInterface != nullptr) {
        disconnect(_currentInterface, &AbstractSparseAlignementDataInterface::dataChanged, this, &OpenGlDrawableLandmarkSet::reloadLandmarks);
        _currentInterface = nullptr;
    }

    clearLandmarks();
}

void OpenGlDrawableLandmarkSet::reloadLandmarks() {
    if (_currentInterface != nullptr) {

        int nPoints = _currentInterface->nPoints();

        _llm_pos.clear();
        _llm_pos.reserve(nPoints*3);

        _llm_id.clear();
        _llm_id.reserve(nPoints);


        for (int i = 0; i < nPoints; i++) {

            QVector3D pos = _currentInterface->getPointPos(i);

            _llm_pos.push_back(pos.x());
            _llm_pos.push_back(pos.y());
            _llm_pos.push_back(pos.z());

            _llm_id.push_back(i);
        }
    }

    _hasToReloadLandmarks = true;
    _hasToReloadLandmarksIds = true;

    Q_EMIT updateRequested();
}
void OpenGlDrawableLandmarkSet::clearLandmarks() {
    _llm_pos.clear();
    Q_EMIT updateRequested();
}

void OpenGlDrawableLandmarkSet::setSceneScale(float sceneScale) {
    _sceneScale = fabs(sceneScale);
    Q_EMIT updateRequested();
}

} // namespace StereoVisionApp
