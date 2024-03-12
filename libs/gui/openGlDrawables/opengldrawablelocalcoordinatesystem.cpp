#include "opengldrawablelocalcoordinatesystem.h"

#include <cmath>

#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QOpenGLContext>

namespace StereoVisionApp {

OpenGlDrawableLocalCoordinateSystem::OpenGlDrawableLocalCoordinateSystem(OpenGl3DSceneViewWidget *parent) :
    OpenGlDrawable(parent),
    _currentInterface(nullptr)
{

}

OpenGlDrawableLocalCoordinateSystem::~OpenGlDrawableLocalCoordinateSystem() {

}

void OpenGlDrawableLocalCoordinateSystem::initializeGL() {

    _ls_vao.create();

    _ls_vao.bind();

    generateLocalCoordinatesModel();

    _ls_vao.release();

    _localSystemProgram = new QOpenGLShaderProgram();
    _localSystemProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/sparseViewerLocalAxis.vert");
    _localSystemProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/sparseViewerLocalAxis.frag");

    _localSystemProgram->link();

}
void OpenGlDrawableLocalCoordinateSystem::paintGL(QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView) {

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    if (_currentInterface != nullptr) {

        int vertexLocation;

        //cameras

        _localSystemProgram->bind();
        _ls_vao.bind();
        _ls_buffer.bind();

        vertexLocation = _localSystemProgram->attributeLocation("in_location");
        _localSystemProgram->enableAttributeArray(vertexLocation);
        _localSystemProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);

        _localSystemProgram->setUniformValue("matrixViewProjection", projectionView*modelView);
        _localSystemProgram->setUniformValue("sceneScale", _sceneScale);

        int nFrames = _currentInterface->nLocalSystems();

        for (int i = 0; i < nFrames; i++) {

            QMatrix4x4 lsPose = _currentInterface->getLocalSystemTransform(i);

            QMatrix4x4 lsScale;
            lsScale.scale(_lsScale);

            QMatrix4x4 lsTransform = lsPose*lsScale;
            _localSystemProgram->setUniformValue("matrixCamToScene", lsTransform);

            f->glDrawArrays(GL_LINES, 0, 3);

        }

        _ls_vao.release();
        _ls_buffer.release();

        _localSystemProgram->disableAttributeArray(vertexLocation);
        _localSystemProgram->release();
    }

}
void OpenGlDrawableLocalCoordinateSystem::clearViewRessources() {

    if (_localSystemProgram != nullptr) {
        delete _localSystemProgram;
    }

    if (_ls_buffer.isCreated()) {
        _ls_buffer.destroy();
    }

    _ls_vao.destroy();
    _ls_ids_vao.destroy();
}

void OpenGlDrawableLocalCoordinateSystem::initializeObjectIdMaskPart() {

    _ls_ids_vao.create();

    _objFixedIdProgram = new QOpenGLShaderProgram();
    _objFixedIdProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/objectCstId.vert");
    _objFixedIdProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/objectId.frag");

    _objFixedIdProgram->link();

}
void OpenGlDrawableLocalCoordinateSystem::paintObjectIdMask(int drawableId,
                                                            QMatrix4x4 const& modelView,
                                                            QMatrix4x4 const& projectionView) {

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    if (_currentInterface != nullptr) {

        int vertexLocation;
        int idLocation;

        //cameras

        _objFixedIdProgram->bind();
        _ls_ids_vao.bind();
        _ls_buffer.bind();

        vertexLocation = _objFixedIdProgram->attributeLocation("in_location");
        _objFixedIdProgram->enableAttributeArray(vertexLocation);
        _objFixedIdProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);

        float LocalSystemColorCode;
        static_assert (sizeof(decltype (drawableId)) == sizeof(decltype (LocalSystemColorCode)), "Assumption about type size broken");
        std::memcpy(&LocalSystemColorCode, &drawableId, sizeof(decltype (drawableId)));

        _objFixedIdProgram->setUniformValue("matrixViewProjection", projectionView*modelView);
        _objFixedIdProgram->setUniformValue("sceneScale", _sceneScale);
        _objFixedIdProgram->setUniformValue("pointScale", 10.0f);
        _objFixedIdProgram->setUniformValue("typeColorIndex", LocalSystemColorCode);

        f->glLineWidth(10.);

        int nFrames = _currentInterface->nLocalSystems();

        for (int i = 0; i < nFrames; i++) {

            QMatrix4x4 lsPose = _currentInterface->getLocalSystemTransform(i);

            QMatrix4x4 lsScale;
            lsScale.scale(_lsScale);

            QMatrix4x4 lsTransform = lsPose*lsScale;
            _objFixedIdProgram->setUniformValue("matrixObjToScene", lsTransform);

            //internal id
            qint64 id = i;
            float convSpace[2];

            std::memcpy(convSpace, &id, sizeof (id));

            QVector2D idColorCode;
            idColorCode.setX(convSpace[0]);
            idColorCode.setY(convSpace[1]);

            _objFixedIdProgram->setUniformValue("in_id", idColorCode);

            f->glDrawArrays(GL_LINES, 0, 3);

        }

        _ls_buffer.release();
        _ls_ids_vao.release();

        _objFixedIdProgram->disableAttributeArray(vertexLocation);
        _objFixedIdProgram->release();
    }

}
void OpenGlDrawableLocalCoordinateSystem::clearObjectsIdsRessources() {
    if (_objFixedIdProgram != nullptr) {
        delete _objFixedIdProgram;
    }

    _ls_ids_vao.destroy();
}

void OpenGlDrawableLocalCoordinateSystem::processClick(int code) {

    if (_currentInterface != nullptr) {
        _currentInterface->clickLocalCoordinateSystem(code);
    }
}

void OpenGlDrawableLocalCoordinateSystem::setInterface(AbstractSparseAlignementDataInterface* i) {

    if (i == _currentInterface) {
        return;
    }

    clearInterface();

    _currentInterface = i;
    connect(i, &AbstractSparseAlignementDataInterface::dataChanged, this, &OpenGlDrawable::updateRequested);

    Q_EMIT updateRequested();
}

void OpenGlDrawableLocalCoordinateSystem::clearInterface() {

    if (_currentInterface != nullptr) {
        disconnect(_currentInterface, &AbstractSparseAlignementDataInterface::dataChanged, this, &OpenGlDrawable::updateRequested);
        _currentInterface = nullptr;

        Q_EMIT updateRequested();
    }
}

void OpenGlDrawableLocalCoordinateSystem::setSceneScale(float sceneScale) {
    _sceneScale = std::fabs(sceneScale);
    Q_EMIT updateRequested();
}

void OpenGlDrawableLocalCoordinateSystem::setCoordinatesScale(float lsScale) {
    _lsScale = std::fabs(lsScale);
    Q_EMIT updateRequested();
}

void OpenGlDrawableLocalCoordinateSystem::generateLocalCoordinatesModel() {


    if (_ls_buffer.isCreated()) {
        _ls_buffer.destroy();
    }

    _ls_buffer.create();
    _ls_buffer.bind();

    std::vector<GLfloat> p(6*3);

    //X axis
    p[0] = 1.;
    p[1] = -1e-12;
    p[2] = -1e-12;

    p[3] = 1e-12;
    p[4] = -1e-12;
    p[5] = -1e-12;

    //Y axis
    p[6] = -1e-12;
    p[7] = 1.;
    p[8] = -1e-12;

    p[9] = -1e-12;
    p[10] = 1e-12;
    p[11] = -1e-12;

    //Z axis
    p[12] = -1e-12;
    p[13] = -1e-12;
    p[14] = 1.;

    p[15] = -1e-12;
    p[16] = -1e-12;
    p[17] = 1e-12;

    _ls_buffer.allocate(p.data(), p.size()*sizeof (GLfloat));
    _ls_buffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
}



} // namespace StereoVisionApp
