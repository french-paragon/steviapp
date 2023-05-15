#include "opengldrawablecamerasset.h"

#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>

#include <cmath>

namespace StereoVisionApp {

OpenGlDrawableCamerasSet::OpenGlDrawableCamerasSet(OpenGl3DSceneViewWidget *parent) :
    OpenGlDrawable(parent),
    _currentInterface(nullptr),
    _cam_indices(QOpenGLBuffer::IndexBuffer)
{

}
OpenGlDrawableCamerasSet::~OpenGlDrawableCamerasSet() {

}

void OpenGlDrawableCamerasSet::initializeGL() {

    _cam_vao.create();

    _cam_vao.bind();

    generateCamModel();

    _cam_vao.release();

    _camProgram = new QOpenGLShaderProgram();
    _camProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/sparseViewerCam.vert");
    _camProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/sparseViewerCam.frag");

    _camProgram->link();

}
void OpenGlDrawableCamerasSet::paintGL(QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView) {

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    if (_currentInterface != nullptr) {

        int vertexLocation;

        //cameras

        _camProgram->bind();
        _cam_vao.bind();
        _cam_buffer.bind();
        _cam_indices.bind();

        vertexLocation = _camProgram->attributeLocation("in_location");
        _camProgram->enableAttributeArray(vertexLocation);
        _camProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);

        _camProgram->setUniformValue("matrixViewProjection", projectionView*modelView);
        _camProgram->setUniformValue("sceneScale", _sceneScale);

        int nFrames = _currentInterface->nCameras();

        for (int i = 0; i < nFrames; i++) {

            QMatrix4x4 camPose = _currentInterface->getCameraTransform(i);

            QMatrix4x4 camScale;
            camScale.scale(_camScale);

            QMatrix4x4 camTransform = camPose*camScale;
            _camProgram->setUniformValue("matrixCamToScene", camTransform);

            f->glDrawElements(GL_LINES, 30, GL_UNSIGNED_INT, 0);

        }

        _cam_vao.release();
        _cam_buffer.release();
        _cam_indices.release();

        _camProgram->disableAttributeArray(vertexLocation);
        _camProgram->release();
    }
}
void OpenGlDrawableCamerasSet::clearViewRessources() {

    if (_camProgram != nullptr) {
        delete _camProgram;
    }

    if (_cam_buffer.isCreated()) {
        _cam_buffer.destroy();
    }

    if (_cam_indices.isCreated()) {
        _cam_indices.destroy();
    }

    _cam_vao.destroy();
    _cam_ids_vao.destroy();

}

void OpenGlDrawableCamerasSet::initializeObjectIdMaskPart() {
    _cam_ids_vao.create();

    _objFixedIdProgram = new QOpenGLShaderProgram();
    _objFixedIdProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/objectCstId.vert");
    _objFixedIdProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/objectId.frag");

    _objFixedIdProgram->link();
}
void OpenGlDrawableCamerasSet::paintObjectIdMask(int drawableId, QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView) {

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    if (_currentInterface != nullptr) {

        int vertexLocation;
        int idLocation;

        //cameras

        _objFixedIdProgram->bind();
        _cam_ids_vao.bind();
        _cam_buffer.bind();
        _cam_indices.bind();

        vertexLocation = _objFixedIdProgram->attributeLocation("in_location");
        _objFixedIdProgram->enableAttributeArray(vertexLocation);
        _objFixedIdProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);

        float CameraColorCode;
        static_assert (sizeof(decltype (drawableId)) == sizeof(decltype (CameraColorCode)), "Assumption about type size broken");
        std::memcpy(&CameraColorCode, &drawableId, sizeof(decltype (drawableId)));

        _objFixedIdProgram->setUniformValue("matrixViewProjection", projectionView*modelView);
        _objFixedIdProgram->setUniformValue("sceneScale", _sceneScale);
        _objFixedIdProgram->setUniformValue("pointScale", 10.0f);
        _objFixedIdProgram->setUniformValue("typeColorIndex", CameraColorCode);

        f->glLineWidth(10.);

        int nFrames = _currentInterface->nCameras();

        for (int i = 0; i < nFrames; i++) {

            QMatrix4x4 camPose = _currentInterface->getCameraTransform(i);

            QMatrix4x4 camScale;
            camScale.scale(_camScale);

            QMatrix4x4 camTransform = camPose*camScale;
            _objFixedIdProgram->setUniformValue("matrixObjToScene", camTransform);

            //internal id
            qint64 id = i;
            float convSpace[2];

            std::memcpy(convSpace, &id, sizeof (id));

            QVector2D idColorCode;
            idColorCode.setX(convSpace[0]);
            idColorCode.setY(convSpace[1]);

            _objFixedIdProgram->setUniformValue("in_id", idColorCode);

            f->glDrawElements(GL_LINES, 30, GL_UNSIGNED_INT, 0);

        }

        _cam_buffer.release();
        _cam_indices.release();
        _cam_ids_vao.release();

        _objFixedIdProgram->disableAttributeArray(vertexLocation);
        _objFixedIdProgram->release();
    }
}
void OpenGlDrawableCamerasSet::clearObjectsIdsRessources() {
    if (_objFixedIdProgram != nullptr) {
        delete _objFixedIdProgram;
    }

    _cam_ids_vao.destroy();
}

void OpenGlDrawableCamerasSet::processClick(int code) {

    if (_currentInterface != nullptr) {
        _currentInterface->clickCam(code);
    }
}

void OpenGlDrawableCamerasSet::setInterface(AbstractSparseAlignementDataInterface* i) {

    if (i == _currentInterface) {
        return;
    }

    clearInterface();

    _currentInterface = i;
    connect(i, &AbstractSparseAlignementDataInterface::dataChanged, this, &OpenGlDrawable::updateRequested);

    Q_EMIT updateRequested();

}
void OpenGlDrawableCamerasSet::clearInterface() {

    if (_currentInterface != nullptr) {
        disconnect(_currentInterface, &AbstractSparseAlignementDataInterface::dataChanged, this, &OpenGlDrawable::updateRequested);
        _currentInterface = nullptr;

        Q_EMIT updateRequested();
    }
}

void OpenGlDrawableCamerasSet::setSceneScale(float sceneScale) {
    _sceneScale = fabs(sceneScale);
    Q_EMIT updateRequested();
}

void OpenGlDrawableCamerasSet::setCamScale(float camScale) {
    _camScale = fabs(camScale);
    Q_EMIT updateRequested();
}

void OpenGlDrawableCamerasSet::generateCamModel() {

    if (_cam_buffer.isCreated()) {
        _cam_buffer.destroy();
    }

    _cam_buffer.create();
    _cam_buffer.bind();

    std::vector<GLfloat> p(11*3);

    //pt0
    p[0] = 1.;
    p[1] = 1.;
    p[2] = 1.;

    //pt1
    p[3] = -1.;
    p[4] = 1.;
    p[5] = 1.;

    //pt2
    p[6] = -1.;
    p[7] = -1.;
    p[8] = 1.;

    //pt3
    p[9] = 1.;
    p[10] = -1.;
    p[11] = 1.;

    //pt4
    p[12] = 0.3;
    p[13] = 0.3;
    p[14] = -0.3;

    //pt5
    p[15] = -0.3;
    p[16] = 0.3;
    p[17] = -0.3;

    //pt6
    p[18] = -0.3;
    p[19] = -0.3;
    p[20] = -0.3;

    //pt7
    p[21] = 0.3;
    p[22] = -0.3;
    p[23] = -0.3;


    //pt8
    p[24] = 0.5;
    p[25] = -1.1;
    p[26] = 1.0;


    //pt9
    p[27] = -0.5;
    p[28] = -1.1;
    p[29] = 1.0;


    //pt10
    p[30] = 0.0;
    p[31] = -1.5;
    p[32] = 1.0;


    _cam_buffer.allocate(p.data(), p.size()*sizeof (GLfloat));
    _cam_buffer.setUsagePattern(QOpenGLBuffer::StaticDraw);


    if (_cam_indices.isCreated()) {
        _cam_indices.destroy();
    }

    _cam_indices.create();
    _cam_indices.bind();

    std::vector<uint32_t> i(30);

    i[0] = 0;
    i[1] = 1;

    i[2] = 1;
    i[3] = 2;

    i[4] = 2;
    i[5] = 3;

    i[6] = 3;
    i[7] = 0;

    i[8] = 0;
    i[9] = 6;

    i[10] = 1;
    i[11] = 7;

    i[12] = 2;
    i[13] = 4;

    i[14] = 3;
    i[15] = 5;

    i[16] = 4;
    i[17] = 5;

    i[18] = 5;
    i[19] = 6;

    i[20] = 6;
    i[21] = 7;

    i[22] = 7;
    i[23] = 4;

    i[24] = 8;
    i[25] = 9;

    i[26] = 9;
    i[27] = 10;

    i[28] = 10;
    i[29] = 8;

    _cam_indices.allocate(i.data(), i.size()*sizeof (uint32_t));
    _cam_indices.setUsagePattern(QOpenGLBuffer::StaticDraw);

}

} // namespace StereoVisionApp
