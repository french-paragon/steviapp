#include "labelledpointsoverlay.h"

#include <qImageDisplayWidget/imagewidget.h>

#include <QEvent>
#include <QMouseEvent>
#include <QGuiApplication>

namespace StereoVisionApp {

LabelledPointsOverlay::LabelledPointsOverlay(QWidget *parent) :
    QImageDisplay::Overlay(parent),
    _activePoint(-1)
{

}

void LabelledPointsOverlay::setPointSet(QVector<QPointF> const& set) {
    _pointsSet = set;
    _labels.clear();
    setActivePoint(-1);
    requestFullRepainting();
}

void LabelledPointsOverlay::setColorMap(const QVector<QColor> &colorMap) {
    _pointsColor = colorMap;
    requestFullRepainting();
}

bool LabelledPointsOverlay::eventFilter(QObject *watched, QEvent *event) {

    QImageDisplay::ImageWidget* widget = qobject_cast<QImageDisplay::ImageWidget*>(watched);

    if (widget != nullptr) {

        if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent* mouseEvent = dynamic_cast<QMouseEvent*>(event);
            return mousePressEvent(widget, mouseEvent);
        }
    }

    return false;
}

void LabelledPointsOverlay::setActivePoint(const qint64 &ap) {

    if (ap != _activePoint) {
        _activePoint = ap;
        activePointChanged(_activePoint);
        requestFullRepainting();
    }
}


bool LabelledPointsOverlay::mousePressEvent(QImageDisplay::ImageWidget* interactionWidget, QMouseEvent *e) {

    Qt::MouseButtons previously_pressed = e->buttons();

    if (previously_pressed == Qt::LeftButton) {

        qint64 pt = pointAt(interactionWidget, e->pos());
        setActivePoint(pt);

        if (pt >= 0) {
            Q_EMIT pointClicked(pt);
        }

    }

    return false;

}

void LabelledPointsOverlay::paintItemImpl(QPainter* painter) const {

    bool allActive = _activePoint < 0;

    for(int i = 0; i < _pointsSet.size(); i++) {


        if (i == _activePoint) {
            continue;
        }

        QString label = QString("%1").arg(i);

        if (i < _labels.size()) {
            label = _labels[i];
        }

        drawDecoratedPoint(painter, _pointsSet[i], allActive, label);

    }

    if (!allActive) {

        QString label = QString("%1").arg(_activePoint);

        if (_activePoint >= 0 and _activePoint < _labels.size()) {
            label = _labels[_activePoint];
        }

        drawDecoratedPoint(painter, _pointsSet[_activePoint], true, label);
    }
}

void LabelledPointsOverlay::drawDecoratedPoint(QPainter* painter, QPointF point, bool isActive, QString label) const {

    QTransform img2paint = imageToPaintArea();

    QPointF wCoord = img2paint.map(point);
    int hPw = 12;
    QPointF hExtend(hPw, hPw);

    QColor pColor(170, 170, 170);
    QColor tColor(255, 255, 255);
    QColor black(0, 0, 0);

    if (isActive) {
        pColor = QColor(255, 150, 20);
    }

    QBrush fill;
    QPen line(black);
    line.setWidth(4);

    painter->setFont(QFont("sans", 8, QFont::Normal));
    int hMTw = 50;
    QRectF maxWidth(0, 0, 2*hMTw, 2*hMTw);
    QRectF b = painter->boundingRect(maxWidth, Qt::AlignHCenter|Qt::AlignTop, label);

    QRectF textBg(wCoord - QPointF(hMTw, 2*hPw) + b.topLeft(),
                  wCoord - QPointF(hMTw, 2*hPw) + b.bottomRight());

    painter->setBrush(fill);
    painter->setPen(line);

    painter->drawRect(textBg);
    painter->drawPoint(wCoord);

    line.setColor(pColor);
    line.setWidth(2);
    painter->setPen(line);

    painter->drawPoint(wCoord);

    fill = QBrush(pColor);
    painter->setBrush(fill);

    painter->drawRect(textBg);

    line.setColor(tColor);
    painter->setPen(line);

    painter->drawText(textBg, Qt::AlignCenter|Qt::AlignTop, label);

}

qint64 LabelledPointsOverlay::pointAt(QImageDisplay::ImageWidget* interactionWidget, QPoint const& widgetCoordinate) const {

    if (interactionWidget == nullptr) {
        return -1;
    }

    QPointF imCoord = interactionWidget->widgetToImageCoordinates(widgetCoordinate);

    qint64 r = -1;
    float tol = 5;
    float d = std::abs(tol);

    for(int i = 0; i < _pointsSet.size(); i++) {
        QPointF delta = imCoord - _pointsSet[i];

        float dm = std::max(std::abs(delta.x()), std::abs(delta.y()));

        if (dm < d) {
            r = i;
            d = dm;
        }
    }

    return r;
}

} // namespace StereoVisionApp
