#include "opengldrawablescenegrid.h"

#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>

namespace StereoVisionApp {

OpenGlDrawableSceneGrid::OpenGlDrawableSceneGrid(OpenGl3DSceneViewWidget *parent) :
    OpenGlDrawable(parent),
    _gridDistance(10.),
    _gridSplits(10)
{

}

OpenGlDrawableSceneGrid::~OpenGlDrawableSceneGrid() {
}

void OpenGlDrawableSceneGrid::initializeGL() {

    _grid_vao.create();

    if (_grid_vao.isCreated()) {
        _grid_vao.bind();
    }

    generateGrid();

    _grid_vao.release();

    _gridProgram = new QOpenGLShaderProgram();
    _gridProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/sparseViewerPerspFloor.vert");
    _gridProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/sparseViewerGrid.frag");

}
void OpenGlDrawableSceneGrid::paintGL(QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView) {

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    _gridProgram->bind();
    _grid_vao.bind();
    _grid_buffer.bind();


    int vertexLocation =  _gridProgram->attributeLocation("in_location");
    _gridProgram->enableAttributeArray(vertexLocation);
    _gridProgram->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 2);

    _gridProgram->setUniformValue("matrixViewProjection", projectionView*modelView);

    f->glDrawArrays(GL_LINES, 0, 8*(_gridSplits + 1));

    _grid_vao.release();
    _grid_buffer.release();

    _gridProgram->disableAttributeArray(vertexLocation);
    _gridProgram->release();

}
void OpenGlDrawableSceneGrid::clearViewRessources() {

    if (_gridProgram != nullptr) {
        delete _gridProgram;
    }

    if (_grid_buffer.isCreated()) {
        _grid_buffer.destroy();
    }

    _grid_vao.destroy();

}

void OpenGlDrawableSceneGrid::generateGrid() {

    if (_grid_buffer.isCreated()) {
        _grid_buffer.destroy();
    }

    _grid_buffer.create();
    _grid_buffer.bind();

    size_t s = 16*_gridSplits + 8;
    std::vector<GLfloat> b(s);

    for(int i = 0; i < 2*_gridSplits + 1; i++) {
        b[4*i] = _gridDistance;
        b[4*i+1] = (i - _gridSplits)*_gridDistance/(_gridSplits);
        b[4*i+2] = -_gridDistance;
        b[4*i+3] = (i - _gridSplits)*_gridDistance/(_gridSplits);
    }

    for(int i = 2*_gridSplits + 1; i < 2*(2*_gridSplits + 1); i++) {
        int p = i - (2*_gridSplits + 1);
        b[4*i] = (p-_gridSplits)*_gridDistance/(_gridSplits);
        b[4*i+1] = _gridDistance;
        b[4*i+2] = (p-_gridSplits)*_gridDistance/(_gridSplits);
        b[4*i+3] = -_gridDistance;
    }

    int sw = 2*(2*_gridSplits + 1) - 1;
    int tw = _gridSplits;
    float tmp;

    tmp= b[4*sw];
    b[4*sw] = b[4*tw];
    b[4*tw] = tmp;

    tmp= b[4*sw+1];
    b[4*sw+1] = b[4*tw+1];
    b[4*tw+1] = tmp;

    tmp= b[4*sw+2];
    b[4*sw+2] = b[4*tw+2];
    b[4*tw+2] = tmp;

    tmp= b[4*sw+3];
    b[4*sw+3] = b[4*tw+3];
    b[4*tw+3] = tmp;

    _grid_buffer.allocate(b.data(), s*sizeof (GLfloat));
    _grid_buffer.setUsagePattern(QOpenGLBuffer::StaticDraw);

}

int OpenGlDrawableSceneGrid::gridSplits() const
{
    return _gridSplits;
}

void OpenGlDrawableSceneGrid::setGridSplits(int newGridSplits)
{
    if (newGridSplits != _gridSplits) {
        _gridSplits = newGridSplits;
        Q_EMIT updateRequested();
    }
}

float OpenGlDrawableSceneGrid::gridDistance() const
{
    return _gridDistance;
}

void OpenGlDrawableSceneGrid::setGridDistance(float newGridDistance)
{
    if (newGridDistance != _gridDistance) {
        _gridDistance = newGridDistance;
        Q_EMIT updateRequested();
    }
}

} // namespace StereoVisionApp
