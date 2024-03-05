#include "imagelandmarksoverlay.h"

#include <StereoVision/QImageDisplayWidget/imagewidget.h>

#include "datablocks/image.h"
#include "datablocks/landmark.h"

#include <QTransform>
#include <QImageReader>

#include <QEvent>
#include <QMouseEvent>
#include <QGuiApplication>

namespace StereoVisionApp {

ImageLandmarksOverlay::ImageLandmarksOverlay(QWidget *parent) :
    QImageDisplay::Overlay(parent),
    _currentImageDataBlock(nullptr)
{

}

void ImageLandmarksOverlay::setImageDatablock(Image* img) {

    if (_currentImageDataBlock != nullptr) {
        unregisterAllLandmark();

        disconnect(_currentImageDataBlock, &QObject::destroyed, this, &ImageLandmarksOverlay::imageDeleted);
        disconnect(_currentImageDataBlock, &Image::pointAdded, this, &ImageLandmarksOverlay::imageLandmarkAdded);
        disconnect(_currentImageDataBlock, &Image::pointRemoved, this, &ImageLandmarksOverlay::imageLandmarkDeleted);
    }

    _currentImageDataBlock = img;

    if (img != nullptr) {

        for (qint64 lmid : img->listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName)) {
            registerLandmark(lmid);
        }

        connect(_currentImageDataBlock, &QObject::destroyed, this, &ImageLandmarksOverlay::imageDeleted);
        connect(_currentImageDataBlock, &Image::pointAdded, this, &ImageLandmarksOverlay::imageLandmarkAdded);
        connect(_currentImageDataBlock, &Image::pointRemoved, this, &ImageLandmarksOverlay::imageLandmarkDeleted);
    }

    requestFullRepainting();
}

void ImageLandmarksOverlay::setActivePoint(const qint64 &ap)
{
    qint64 activePoint = ap;

    if (_currentImageDataBlock == nullptr) {
        activePoint = -1;
    } else if (qobject_cast<ImageLandmark*>(_currentImageDataBlock->getById(activePoint)) == nullptr) {
        activePoint = -1;
    }

    if (activePoint != _activePoint) {
        _activePoint = activePoint;
        requestFullRepainting();
    }
}

void ImageLandmarksOverlay::registerLandmark(qint64 id) {
    ImageLandmark* imlm = _currentImageDataBlock->getImageLandmark(id);
    Landmark* lm = imlm->attachedLandmark();

    QMetaObject::Connection c = connect(imlm, &ImageLandmark::coordsChanged, this, &ImageLandmarksOverlay::requestFullRepainting);
    QMetaObject::Connection n = connect(lm, &QObject::objectNameChanged, this, &ImageLandmarksOverlay::requestFullRepainting);

    _landmarkConnections.insert(id, {c, n});
}

void ImageLandmarksOverlay::unregisterLandmark(qint64 id) {
    for (QMetaObject::Connection & co : _landmarkConnections[id]) {
        disconnect(co);
    }

    _landmarkConnections.remove(id);
}

void ImageLandmarksOverlay::unregisterAllLandmark() {

    QList<qint64> keys = _landmarkConnections.keys();

    for (qint64 id : keys) {
        unregisterLandmark(id);
    }
}

void ImageLandmarksOverlay::imageDeleted() {
    unregisterAllLandmark();
    _currentImageDataBlock = nullptr;
    requestFullRepainting();
}
void ImageLandmarksOverlay::imageLandmarkAdded(qint64 id) {
    registerLandmark(id);
    Q_EMIT repaintingRequested(QRect());
}
void ImageLandmarksOverlay::imageLandmarkDeleted(qint64 id) {
    unregisterLandmark(id);
    Q_EMIT repaintingRequested(QRect());
}

void ImageLandmarksOverlay::requestFullRepainting() {
    Q_EMIT repaintingRequested(QRect());
}

void ImageLandmarksOverlay::paintItemImpl(QPainter* painter) const {

    if (_currentImageDataBlock == nullptr) {
        return;
    }

    QString imageName = _currentImageDataBlock->getImageFile();
    QImageReader reader(imageName);

    QSize extend = reader.size();

    ImageLandmark* active = nullptr;

    QVector<qint64> pointsids = _currentImageDataBlock->listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName);

    for(qint64 id : pointsids) {
        ImageLandmark* lm = _currentImageDataBlock->getImageLandmark(id);

        if (id == _activePoint) {
            active = lm;
            continue;
        }

        if (_drawOnlyPoint >= 0 and _drawOnlyPoint != lm->attachedLandmarkid()) {
            continue;
        }

        auto coords = lm->imageCoordinates();

        if (coords.x() < 0 or coords.y() < 0 or
                coords.x() > extend.width() or coords.y() > extend.height() ) {
            drawOuterLandmark(painter, coords, false, lm->attachedLandmarkName());
        } else {
            drawLandmark(painter, lm->imageCoordinates(), false, lm->attachedLandmarkName());
        }

    }

    if (active != nullptr) {

        bool ok = false;

        auto coords = active->imageCoordinates();

        if (_drawOnlyPoint >= 0 and _drawOnlyPoint == active->attachedLandmarkid()) {
            ok = true;
        } else if (_drawOnlyPoint < 0) {
            ok = true;
        }

        if (ok) {
            if (coords.x() < 0 or coords.y() < 0 or
                    coords.x() > extend.width() or coords.y() > extend.height() ) {
                drawOuterLandmark(painter, coords, true, active->attachedLandmarkName());
            } else {
                drawLandmark(painter, coords, true, active->attachedLandmarkName());
            }
        }
    }

}

void ImageLandmarksOverlay::drawLandmark(QPainter* painter, QPointF const& imagePoint, bool isActive, const QString &ptn) const {

    QTransform img2paint = imageToPaintArea();

    QString ptName = ptn;

    if (ptName.isEmpty()) {
        ptName = "No name";
    }

    QPointF wCoord = img2paint.map(imagePoint);
    int hPw = 12;
    QPointF hExtend(hPw, hPw);

    QColor pColor(106, 193, 74);
    QColor tColor(255, 255, 255);
    QColor black(0, 0, 0);

    if (isActive) {
        tColor = pColor;
        pColor = QColor(255, 255, 255);
    }

    QBrush fill;
    QPen line(black);
    line.setWidth(4);

    painter->setFont(QFont("sans", 8, QFont::Normal));
    int hMTw = 50;
    QRectF maxWidth(0, 0, 2*hMTw, 2*hMTw);
    QRectF b = painter->boundingRect(maxWidth, Qt::AlignHCenter|Qt::AlignTop, ptName);

    QRectF textBg(wCoord + QPointF(-hMTw, -hPw-b.height()) + b.topLeft(),
                  wCoord + QPointF(-hMTw, -hPw-b.height()) + b.bottomRight());

    QRectF outterRect(wCoord - hExtend, wCoord + hExtend);

    painter->setBrush(fill);
    painter->setPen(line);

    painter->drawRect(outterRect);
    painter->drawRect(textBg);
    painter->drawPoint(wCoord);

    line.setColor(pColor);
    line.setWidth(2);
    painter->setPen(line);

    painter->drawRect(outterRect);
    painter->drawPoint(wCoord);

    fill = QBrush(pColor);
    painter->setBrush(fill);

    painter->drawRect(textBg);

    line.setColor(tColor);
    painter->setPen(line);

    painter->drawText(textBg, Qt::AlignCenter|Qt::AlignTop, ptName);
}

void ImageLandmarksOverlay::drawOuterLandmark(QPainter* painter, QPointF const& imagePoint, bool isActive, const QString &ptn) const {

    QTransform img2paint = imageToPaintArea();
    QSize size = paintAreaSize();

    QString ptName = ptn;

    if (ptName.isEmpty()) {
        ptName = "No name";
    }

    QPointF wCoord = img2paint.map(imagePoint);

    QPointF constrained = wCoord;

    if (constrained.x() < 0) {
        constrained.setX(0);
    }
    if (constrained.y() < 0) {
        constrained.setY(0);
    }
    if (constrained.x() > size.width()) {
        constrained.setX(size.width());
    }
    if (constrained.y() > size.height()) {
        constrained.setY(size.height());
    }

    int hPw = 12;

    QPointF hExtend(hPw, hPw);

    QColor pColor(191, 51, 26);
    QColor tColor(255, 255, 255);
    QColor black(0, 0, 0);

    if (isActive) {
        tColor = pColor;
        pColor = QColor(255, 255, 255);
    }

    QBrush fill;
    QPen line(black);
    line.setWidth(4);

    painter->setFont(QFont("sans", 8, QFont::Normal));
    int hMTw = 50;
    QRectF maxWidth(0, 0, 2*hMTw, 2*hMTw);
    QRectF b = painter->boundingRect(maxWidth, Qt::AlignHCenter|Qt::AlignTop, ptName);

    QPolygonF arrow;

    QRectF textBg;

    if (constrained.x() == size.width() and constrained.y() == size.height()) {
        arrow << QPointF(constrained)
              << QPointF(size.width() - hPw, size.height())
              << QPointF(size.width(), size.height() - hPw);

        textBg = QRectF(constrained + QPointF(-2*hMTw -hPw , -hPw-b.height()) + b.topLeft(),
                          constrained + QPointF(-2*hMTw -hPw, -hPw-b.height()) + b.bottomRight());

    } else if (constrained.x() == size.width() and constrained.y() == 0) {
        arrow << QPointF(constrained)
              << QPointF(size.width() - hPw, 0)
              << QPointF(size.width(), hPw);

        textBg = QRectF(constrained + QPointF(-2*hMTw -hPw , -b.height()) + b.topLeft(),
                          constrained + QPointF(-2*hMTw -hPw, -b.height()) + b.bottomRight());

    } else if (constrained.x() == 0 and constrained.y() == size.height()) {
        arrow << QPointF(constrained)
              << QPointF(hPw, size.height())
              << QPointF(0, size.height() - hPw);

        textBg = QRectF(constrained + QPointF(hPw , -hPw-b.height()) + b.topLeft(),
                          constrained + QPointF(hPw, -hPw-b.height()) + b.bottomRight());

    }  else if (constrained.x() == 0 and constrained.y() == 0) {
        arrow << QPointF(constrained)
              << QPointF(hPw, 0)
              << QPointF(0, hPw);

        textBg = QRectF(constrained + QPointF(hPw , hPw-b.height()) + b.topLeft(),
                          constrained + QPointF(hPw, hPw-b.height()) + b.bottomRight());

    } else if (constrained.x() == size.width()) {
        arrow << QPointF(constrained)
              << QPointF(size.width() - hPw, constrained.y() - hPw)
              << QPointF(size.width() - hPw, constrained.y() + hPw);

        textBg = QRectF(constrained + QPointF(-2*hMTw -hPw , -b.height()) + b.topLeft(),
                          constrained + QPointF(-2*hMTw -hPw, -b.height()) + b.bottomRight());

    } else if (constrained.y() == size.height()) {
        arrow << QPointF(constrained)
              << QPointF(constrained.x() - hPw, size.height() - hPw)
              << QPointF(constrained.x() + hPw, size.height() - hPw);

        textBg = QRectF(constrained + QPointF(-hMTw -hPw , -hPw-b.height()) + b.topLeft(),
                          constrained + QPointF(-hMTw -hPw, -hPw-b.height()) + b.bottomRight());

    } else if (constrained.x() == 0) {
        arrow << QPointF(constrained)
              << QPointF(hPw, constrained.y() - hPw)
              << QPointF(hPw, constrained.y() + hPw);

        textBg = QRectF(constrained + QPointF(hPw , -hPw-b.height()) + b.topLeft(),
                          constrained + QPointF(hPw, -hPw-b.height()) + b.bottomRight());

    } else if (constrained.y() == 0) {
        arrow << QPointF(constrained)
              << QPointF(constrained.x() - hPw, hPw)
              << QPointF(constrained.x() + hPw, hPw);

        textBg = QRectF(constrained + QPointF(-hMTw -hPw , hPw) + b.topLeft(),
                          constrained + QPointF(-hMTw -hPw, hPw) + b.bottomRight());
    }



    painter->setBrush(fill);
    painter->setPen(line);

    painter->drawConvexPolygon(arrow);
    painter->drawRect(textBg);
    painter->drawPoint(constrained);

    line.setColor(pColor);
    line.setWidth(2);
    painter->setPen(line);

    painter->drawConvexPolygon(arrow);
    painter->drawPoint(constrained);

    fill = QBrush(pColor);
    painter->setBrush(fill);

    painter->drawRect(textBg);

    line.setColor(tColor);
    painter->setPen(line);

    painter->drawText(textBg, Qt::AlignCenter|Qt::AlignTop, ptName);
}

bool ImageLandmarksOverlay::eventFilter(QObject *watched, QEvent *event) {

    QImageDisplay::ImageWidget* widget = qobject_cast<QImageDisplay::ImageWidget*>(watched);

    if (widget != nullptr) {

        if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent* mouseEvent = dynamic_cast<QMouseEvent*>(event);
            return mousePressEvent(widget, mouseEvent);
        }

        if (event->type() == QEvent::MouseButtonRelease) {
            QMouseEvent* mouseEvent = dynamic_cast<QMouseEvent*>(event);
            return mouseReleaseEvent(widget, mouseEvent);
        }

        if (event->type() == QEvent::MouseMove) {
            QMouseEvent* mouseEvent = dynamic_cast<QMouseEvent*>(event);
            return mouseMoveEvent(widget, mouseEvent);
        }
    }

    return false;
}



bool ImageLandmarksOverlay::mousePressEvent(QImageDisplay::ImageWidget* interactionWidget, QMouseEvent *e) {

    QGuiApplication* gapp = qGuiApp;

    if (gapp != nullptr) {
        Qt::KeyboardModifiers kmods = gapp->keyboardModifiers();

        if (kmods & Qt::ControlModifier) {
            _control_pressed_when_clicked = true;
        } else {
            _control_pressed_when_clicked = false;
        }
    }

    _previously_pressed = e->buttons();

    if (_previously_pressed == Qt::LeftButton) {

        _motion_origin_pos = e->pos();
        _motion_origin_t = interactionWidget->translation();

        qint64 pt = pointAt(interactionWidget, e->pos());
        setActivePoint(pt);

        if (pt >= 0) {
            _control_pressed_when_clicked = false;
            return true;
        }

    } else if (_previously_pressed == Qt::RightButton) {
        qint64 pt = pointAt(interactionWidget, e->pos());
        setActivePoint(pt);

        if (pt >= 0) {
            _control_pressed_when_clicked = false;
            Q_EMIT menuPointClick(pt, interactionWidget, e->pos());
            return true;
        }

    }

    return false;

}

bool ImageLandmarksOverlay::mouseReleaseEvent(QImageDisplay::ImageWidget* interactionWidget, QMouseEvent *e) {
    if (_previously_pressed == Qt::LeftButton) {
        if (_control_pressed_when_clicked) {
            emit newPointClick(interactionWidget->widgetToImageCoordinates(e->pos()));
            return true;
        }
    }

    return false;
}
bool ImageLandmarksOverlay::mouseMoveEvent(QImageDisplay::ImageWidget* interactionWidget, QMouseEvent *e) {

    if (_currentImageDataBlock == nullptr) {
        return false;
    }

    if (_previously_pressed == Qt::LeftButton) {
        if (!_control_pressed_when_clicked) {
            if (_previously_pressed == Qt::LeftButton and _activePoint >= 0) {
                ImageLandmark* ilm = _currentImageDataBlock->getImageLandmark(_activePoint);
                if (ilm != nullptr) {
                    ilm->setImageCoordinates(interactionWidget->widgetToImageCoordinates(e->pos()));
                    requestFullRepainting();
                    return true;
                }
            }
        }
    }

    return false;
}


qint64 ImageLandmarksOverlay::pointAt(QImageDisplay::ImageWidget* interactionWidget, QPoint const& widgetCoordinate) const {

    if (interactionWidget == nullptr) {
        return -1;
    }

    if (_currentImageDataBlock == nullptr) {
        return -1;
    }

    QPointF imCoord = interactionWidget->widgetToImageCoordinates(widgetCoordinate);
    return _currentImageDataBlock->getImageLandMarkAt(imCoord, 600./interactionWidget->zoom());
}

} // namespace StereoVisionApp
