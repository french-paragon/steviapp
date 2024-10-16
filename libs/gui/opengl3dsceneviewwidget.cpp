#include "opengl3dsceneviewwidget.h"

#include <cmath>

#include <QOpenGLFunctions>
#include <QOpenGLFunctions_4_5_Core>
#include <QOffscreenSurface>
#include <QOpenGLShaderProgram>
#include <QOpenGLFramebufferObjectFormat>

#include <QWheelEvent>
#include <QGuiApplication>

#include <QMessageBox>

#include <QFileDialog>

void GLAPIENTRY
MessageCallback( GLenum source,
                 GLenum type,
                 GLuint id,
                 GLenum severity,
                 GLsizei length,
                 const GLchar* message,
                 const void* userParam )
{
    const StereoVisionApp::OpenGl3DSceneViewWidget* widget_const =
            reinterpret_cast<const StereoVisionApp::OpenGl3DSceneViewWidget*>(userParam);

    StereoVisionApp::OpenGl3DSceneViewWidget* widget =
            const_cast<StereoVisionApp::OpenGl3DSceneViewWidget*>(widget_const);

    if (widget == nullptr) {
        fprintf( stderr, "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s\n",
           ( type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : "" ),
            type, severity, message );
    } else {

        fprintf( stderr, "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s\n",
           ( type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : "" ),
            type, severity, message );
    }
}

namespace StereoVisionApp {

OpenGlDrawable::OpenGlDrawable(OpenGl3DSceneViewWidget* parent) :
    QObject(parent)
{

}
OpenGlDrawable::~OpenGlDrawable() {

}

void OpenGlDrawable::initializeObjectIdMaskPart() {
    return;
}
void OpenGlDrawable::paintObjectIdMask(int drawableCode, QMatrix4x4 const& modelView, QMatrix4x4 const& projectionView) {
    Q_UNUSED(drawableCode)
    Q_UNUSED(modelView)
    Q_UNUSED(projectionView)
    return;
}
void OpenGlDrawable::clearObjectsIdsRessources() {
    return;
}

void OpenGlDrawable::processClick(int code) {
    Q_UNUSED(code);
    return;
}
void OpenGlDrawable::processHoover(int code) {
    Q_UNUSED(code);
    return;
}

OpenGl3DSceneViewWidget::OpenGl3DSceneViewWidget(QWidget *parent) :
    QOpenGLWidget(parent),
    _nextAvailableDrawableId(1),
    _isInitialized(false)
{

    _min_view_distance = 0.01;
    _max_view_distance = 100.;

    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);

    QSurfaceFormat offscreen_id_format = QSurfaceFormat::defaultFormat();
    offscreen_id_format.setSamples(1); //ensure we use a single sample for offscreen id rendering
    offscreen_id_format.setSwapBehavior(QSurfaceFormat::SingleBuffer); //we are not showing the rendered image, so double buffering not required.

    _obj_raycasting_context = new QOpenGLContext();
    _obj_raycasting_surface = new QOffscreenSurface();
    _obj_raycasting_surface->setFormat(offscreen_id_format);
    _obj_raycasting_fbo = nullptr;

    _id_img = nullptr;

    resetView();

}

OpenGl3DSceneViewWidget::~OpenGl3DSceneViewWidget() {

    makeCurrent();

    //delete the drawables before cleaning up, to ensure they still have the opengl context for their destructor
    for (OpenGlDrawable* oglDrawable : _drawables) {
        oglDrawable->clearViewRessources();
    }

    doneCurrent();

    if (_obj_raycasting_context != nullptr) {
        if (_obj_raycasting_surface != nullptr) {
            _obj_raycasting_context->makeCurrent(_obj_raycasting_surface);

            for (OpenGlDrawable* oglDrawable : _drawables) {
                oglDrawable->clearObjectsIdsRessources();
            }

            _obj_raycasting_context->doneCurrent();
        }

        delete _obj_raycasting_context;

    }

    if (_obj_raycasting_surface != nullptr) {
        delete _obj_raycasting_surface;
    }

    if (_id_img != nullptr) {
        delete _id_img;
    }
}

void OpenGl3DSceneViewWidget::addDrawable(OpenGlDrawable* drawable) {

    drawable->setParent(this); //take ownership of the drawable

    _drawables.push_back(drawable);

    _drawableCodeMap.insert(drawable, _nextAvailableDrawableId);
    _inverseDrawableCodeMap.insert(_nextAvailableDrawableId, drawable);

    _nextAvailableDrawableId++;

    connect(drawable, &OpenGlDrawable::updateRequested, this, [this] () {
        update();
    });

    if (_isInitialized) { //the drawable needs to be initialized manually
        makeCurrent();
        drawable->initializeGL();
        update();
    }
}
void OpenGl3DSceneViewWidget::removeDrawable(OpenGlDrawable* drawable) {

    bool contain = _drawables.contains(drawable);

    if (!contain) {
        return;
    }

    int idx = _drawableCodeMap.value(drawable);
    _drawableCodeMap.remove(drawable);
    _inverseDrawableCodeMap.remove(idx);
    _drawables.removeAll(drawable);

    drawable->deleteLater();

}


void OpenGl3DSceneViewWidget::zoomIn(float steps) {
    _view_distance = _view_distance*powf(0.9, steps);
    if (_view_distance < _min_view_distance) {
        _view_distance = _min_view_distance;
    }
    update();
}
void OpenGl3DSceneViewWidget::zoomOut(float steps) {
    _view_distance = _view_distance/powf(0.9, steps);
    if (_view_distance > _max_view_distance) {
        _view_distance = _max_view_distance;
    }
    update();
}

void OpenGl3DSceneViewWidget::rotateZenith(float degrees) {

    float newAngle = _zenith_angle + degrees;
    if (newAngle < 0) {
        newAngle = 0;
    } else if (newAngle > 180) {
        newAngle = 180;
    }

    if (newAngle != _zenith_angle) {
        _zenith_angle = newAngle;
        update();
    }

}
void OpenGl3DSceneViewWidget::rotateAzimuth(float degrees) {

    float newAngle = _azimuth_angle + degrees;

    if (newAngle < 0) {
        newAngle += (floor(fabs(newAngle)/360.) + 1)*360.;
    } else if (newAngle > 360) {
        newAngle -= floor(fabs(newAngle)/360.)*360.;
    }

    if (newAngle != _azimuth_angle) {
        _azimuth_angle = newAngle;
        update();
    }

}

void OpenGl3DSceneViewWidget::translateView(float deltaX, float deltaY) {

    _x_delta += deltaX;
    _y_delta += deltaY;

    update();
}

void OpenGl3DSceneViewWidget::saveViewpoint() {
    QString saveFile = QFileDialog::getSaveFileName(this, tr("Save viewpoint"), QString(), {tr("text file (*.txt)")});

    if (saveFile.isEmpty()) {
        return;
    }

    saveViewpoint(saveFile);
}
void OpenGl3DSceneViewWidget::saveViewpoint(QString file) {

    QFile out(file);

    if (!out.open(QFile::WriteOnly)) {
        return;
    }

    QTextStream stream(&out);

    stream << _view_distance << ' ';

    stream << _zenith_angle << ' ';
    stream << _azimuth_angle << ' ';

    stream << _x_delta << ' ';
    stream << _y_delta << ' ';

    stream.flush();

    out.close();

}

void OpenGl3DSceneViewWidget::loadViewpoint() {
    QString loadFile = QFileDialog::getOpenFileName(this, tr("Load viewpoint"), QString(), {tr("text file (*.txt)")});

    if (loadFile.isEmpty()) {
        return;
    }

    loadViewpoint(loadFile);
}

void OpenGl3DSceneViewWidget::loadViewpoint(QString file) {

    QFile in(file);

    if (!in.open(QFile::ReadOnly)) {
        return;
    }

    QTextStream stream(&in);

    stream >> _view_distance;

    stream >> _zenith_angle;
    stream >> _azimuth_angle;

    stream >> _x_delta;
    stream >> _y_delta;

    in.close();

    update();

}

void OpenGl3DSceneViewWidget::initializeGL() {

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    f->glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    f->glEnable(GL_MULTISAMPLE);
    f->glEnable(GL_POINT_SMOOTH);
    f->glEnable(GL_PROGRAM_POINT_SIZE);
    f->glEnable(GL_DEPTH_TEST);

    f->glEnable( GL_DEBUG_OUTPUT );

    QOpenGLFunctions_4_5_Core *f45;
    f45 = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_4_5_Core>();
    f45->glDebugMessageCallback( MessageCallback, this );

    //initialize all drawables.
    for (OpenGlDrawable* oglDrawable : _drawables) {
        oglDrawable->initializeGL();
    }

    initializeObjectIdMaskPart();

    _isInitialized = true;
}
void OpenGl3DSceneViewWidget::paintGL() {

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    f->glClearColor(1,1,1,1);
    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    f->glLineWidth(1.);

    setView(width(), height());

    //paints all drawables.
    for (OpenGlDrawable* oglDrawable : _drawables) {
        oglDrawable->paintGL(_modelView, _projectionView);
    }

    paintObjectIdMask();
}
void OpenGl3DSceneViewWidget::resizeGL(int w, int h) {
    setView(w, h);
}

void OpenGl3DSceneViewWidget::initializeObjectIdMaskPart() {

    if (_obj_raycasting_context == nullptr or _obj_raycasting_surface == nullptr) {
        return; //no object id context mean no object id masks
    }

    _obj_raycasting_context->makeCurrent(_obj_raycasting_surface);

    //initialize all drawables.
    for (OpenGlDrawable* oglDrawable : _drawables) {
        oglDrawable->initializeObjectIdMaskPart();
    }

    _obj_raycasting_context->doneCurrent();
}
void OpenGl3DSceneViewWidget::paintObjectIdMask() {

    GLenum err = glGetError();
    while(err != GL_NO_ERROR) {
      // loop to remove errors
        err = glGetError();
    }

    if (_obj_raycasting_context == nullptr or _obj_raycasting_surface == nullptr) {
        return; //no object id context mean no object id masks
    }

    setView(width(), height());

    _obj_raycasting_context->makeCurrent(_obj_raycasting_surface);

    QOpenGLFramebufferObjectFormat fbo_format;
    fbo_format.setSamples(0);
    fbo_format.setInternalTextureFormat(GL_RGB32F); //this can store an int64 using two floats, plus additional information.

    if (_obj_raycasting_fbo == nullptr) {
        _obj_raycasting_fbo = new QOpenGLFramebufferObject(width(), height(), fbo_format);
    } else if (_obj_raycasting_fbo->size() != size()) {
        delete _obj_raycasting_fbo;
        _obj_raycasting_fbo = new QOpenGLFramebufferObject(width(), height(), fbo_format);
    }

    QOpenGLFunctions *f = _obj_raycasting_context->functions();

    _obj_raycasting_fbo->bind();
    f->glViewport(0, 0, width(), height());

    f->glClearColor(0,0,0,0);
    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //paints all drawables.
    for (OpenGlDrawable* oglDrawable : _drawables) {
        int code = _drawableCodeMap.value(oglDrawable, -1);

        if (code < 0) {
            continue; //drawable has no code
        }

        oglDrawable->paintObjectIdMask(code, _modelView, _projectionView);
    }

    if (_id_img == nullptr) {

        _id_img = new Multidim::Array<float, 3>({height(), width(), 3}, {3*width(), 3, 1});

    } else if (_id_img->shape()[0] != height() or _id_img->shape()[1] != width()) {

        delete _id_img;
        _id_img = new Multidim::Array<float, 3>({height(), width(), 3}, {3*width(), 3, 1});
    }

    // tell openGL we want to read from the back buffer
    //f->glReadBuffer(GL_BACK);
    f->glReadPixels(0, 0, width(), height(), GL_RGB, GL_FLOAT, &(_id_img->atUnchecked(0)));

    err = glGetError();
    if (err != GL_NO_ERROR) {
        qDebug() << "OpenGL error while reading  id pass: " << err;
    }

    _obj_raycasting_fbo->release();
    _obj_raycasting_context->doneCurrent();
}

void OpenGl3DSceneViewWidget::wheelEvent(QWheelEvent *e) {

    QPoint d = e->angleDelta();
    if (d.y() == 0){
        e->ignore();
        return;
    }

    QGuiApplication* gapp = qGuiApp;
    Qt::KeyboardModifiers kmods;

    if (gapp != nullptr) {
        kmods = gapp->keyboardModifiers();
    }

    if (e->buttons() == Qt::NoButton) {

        float step = d.y()/50.;

        if (step < 0) {
            zoomOut(-step);
        } else {
            zoomIn(step);
        }

        e->accept();
    } else {
        e->ignore();
    }
}
void OpenGl3DSceneViewWidget::keyPressEvent(QKeyEvent *e)  {

    int k = e->key();
    int m = e->modifiers();

    if (k == Qt::Key_Down and m & Qt::KeypadModifier) {
        rotateZenith(10.);
    } else if (k == Qt::Key_Up and m & Qt::KeypadModifier) {
        rotateZenith(-10.);
    } else if (k == Qt::Key_Left and m & Qt::KeypadModifier) {
        rotateAzimuth(10.);
    } else if (k == Qt::Key_Right and m & Qt::KeypadModifier) {
        rotateAzimuth(-10.);
    } else if (k == Qt::Key_Plus and m & Qt::KeypadModifier) {
        zoomIn(1.);
    } else if (k == Qt::Key_Minus and m & Qt::KeypadModifier) {
        zoomOut(1.);
    }

    else if (k == Qt::Key_E and m & Qt::ControlModifier) {
        saveViewpoint();
    }
    else if (k == Qt::Key_R and m & Qt::ControlModifier) {
        loadViewpoint();
    }

    else {
        QWidget::keyPressEvent(e);
    }
}
void OpenGl3DSceneViewWidget::mousePressEvent(QMouseEvent *e)  {

    _previously_pressed = e->buttons();

    _motion_origin_pos = e->pos();

    if (_previously_pressed == Qt::MiddleButton or
            _previously_pressed == Qt::LeftButton or
            _previously_pressed == Qt::RightButton) {

        e->accept();
    } else {
        e->ignore();
    }
}
void OpenGl3DSceneViewWidget::mouseReleaseEvent(QMouseEvent *e)  {

    if (_previously_pressed == Qt::MiddleButton) {

        e->ignore();

    } else if (_previously_pressed == Qt::LeftButton) {

        int x = e->pos().x();
        int y = e->pos().y();

        IdPassInfos cid = idPassAtPos(x, y);

        OpenGlDrawable* drawable = _inverseDrawableCodeMap.value(cid.itemType, nullptr);

        if (drawable != nullptr) {
            drawable->processClick(cid.itemId);
        } else {
            processVoidClick();
        }

        e->accept();

        return;
    }
}
void OpenGl3DSceneViewWidget::mouseMoveEvent(QMouseEvent *e){

    int b = e->buttons();

    if (b == Qt::MiddleButton) {
        QPoint nP = e->pos();

        QPoint t = nP - _motion_origin_pos;
        _motion_origin_pos = nP;

        rotateZenith(-t.y()/3.);
        rotateAzimuth(t.x()/3.);

        e->accept();

    } else if (b == Qt::RightButton) {

        QPoint nP = e->pos();

        QPoint t = nP - _motion_origin_pos;
        _motion_origin_pos = nP;

        translateView(qreal(t.x())/width(), -qreal(t.y())/height());

        e->accept();

    } else if (b == Qt::NoButton) {

        int x = e->pos().x();
        int y = e->pos().y();

        IdPassInfos cid = idPassAtPos(x, y);

        OpenGlDrawable* drawable = _inverseDrawableCodeMap.value(cid.itemType, nullptr);

        if (drawable != nullptr) {
            //drawable->processHoover(cid.itemId); //TODO: implement hoover interaction
        }

        e->accept();

        return;
    }
}

void OpenGl3DSceneViewWidget::setView(int w, int h) {
    QMatrix4x4 model;
    QMatrix4x4 view;
    QMatrix4x4 proj;
    //_projectionView.rotate(90.,0.,1.,0.);
    model.rotate(_zenith_angle, -1.0, 0.0);
    model.rotate(_azimuth_angle, 0.0, 0.0, 1.0);

    view.lookAt(QVector3D(0,0,_view_distance),
                QVector3D(0,0,0),
                QVector3D(0,1.0,0));

    proj.perspective(70., w/static_cast<float>(h), 0.1f, 1000.0f);
    proj.translate(_x_delta*w/12, _y_delta*h/12);

    _modelView = view*model;
    _projectionView = proj;
}

void OpenGl3DSceneViewWidget::resetView() {
    resetView(width(), height());
}

void OpenGl3DSceneViewWidget::resetView(int w, int h) {
    _view_distance = 10.;

    _zenith_angle = 0;
    _azimuth_angle = 0;

    _x_delta = 0;
    _y_delta = 0;

    setView(w,h);
}


OpenGl3DSceneViewWidget::IdPassInfos OpenGl3DSceneViewWidget::idPassAtPos(int x, int y) {

    IdPassInfos ret = { -1, -1};

    if (_obj_raycasting_context == nullptr or _obj_raycasting_surface == nullptr) {
        return ret; //no object id context mean no object id masks
    }

    if (_obj_raycasting_fbo == nullptr) {
        return ret;
    }

    if (x < 0 or x > width()-1) {
        return ret;
    }

    if (y < 0 or y > height()-1) {
        return ret;
    }

    if (_id_img != nullptr) {

        int y_idx = height() - y - 1;

        float red = _id_img->atUnchecked(y_idx, x, 0);
        float green = _id_img->atUnchecked(y_idx, x, 1);
        float blue = _id_img->atUnchecked(y_idx, x, 2);

        //qDebug() << "red: " << red << " green: " << green << " blue: " << blue;

        int r;
        int g;

        //get the bytes of the float pixel values as integer
        std::memcpy(&r, &red, sizeof r);
        std::memcpy(&g, &green, sizeof g);

        qint64 data = static_cast<qint64>(g) << 32 | r;

        ret.itemId = data;

        int b;
        std::memcpy(&b, &blue, sizeof g);
        ret.itemType = b;

        return ret;
    }

    return ret;
}

void OpenGl3DSceneViewWidget::processVoidClick() {
    return;
}

} // namespace StereoVisionApp
