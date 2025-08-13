#include "imagewidget.h"

#include <QDebug>

#include <QMouseEvent>
#include <QTimer>

namespace StereoVisionApp {

ImageWidget::ImageWidget(QWidget *parent) :
    QImageDisplay::ImageWidget(parent),
    _mouseHandlerTriggerReady(false)
{

    setMouseTracking(true);

    _mouseMoveLimiterTimer = new QTimer(this);
    _mouseMoveLimiterTimer->callOnTimeout(this, [this] () {
        _mouseHandlerTriggerReady = true;
    });
    _mouseMoveLimiterTimer->start(100); //limit the rate at which the move handler can trigger to 1/10th of a second.

}

void ImageWidget::setMouseMoveHandler(std::function<bool(QMouseEvent *)> const& mouseMouveHandler) {
    _mouseMoveHandler = mouseMouveHandler;
}

void ImageWidget::mousePressEvent(QMouseEvent *e) {

    Qt::MouseButtons previously_pressed = e->buttons();

    if (previously_pressed == Qt::RightButton) {

        Q_EMIT menuNonPointClick(e->pos());
	} else {
        QImageDisplay::ImageWidget::mousePressEvent(e);
	}

}

void ImageWidget::mouseMoveEvent(QMouseEvent *event) {

    QImageDisplay::ImageWidget::mouseMoveEvent(event);

    if (!event->isAccepted() and event->buttons() == Qt::MouseButton::NoButton) {
        if (_mouseHandlerTriggerReady and _mouseMoveHandler) {
            _mouseMoveHandler(event);
            _mouseHandlerTriggerReady = false;
        }
    }

}


} // namespace StereoVisionApp
