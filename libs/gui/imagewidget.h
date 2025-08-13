#ifndef STEREOVISIONAPP_IMAGEWIDGET_H
#define STEREOVISIONAPP_IMAGEWIDGET_H

#include <StereoVision/QImageDisplayWidget/imagewidget.h>
#include <StereoVision/QImageDisplayWidget/imageadapter.h>

#include <QWidget>
#include <QMap>
#include <QMetaObject>
#include <QPen>
#include <QBrush>
#include <QPainter>
#include <QPointF>

#include <functional>

class QTimer;

namespace StereoVisionApp {

class Image;

class ImageWidget : public QImageDisplay::ImageWidget
{
    Q_OBJECT
public:
    explicit ImageWidget(QWidget *parent = nullptr);

    /*!
     * \brief setMouseMoveHandler install a special move handler when the mouse move without any click
     * \param mouseMouveHandler the mouse move handler function
     */
    void setMouseMoveHandler(std::function<bool(QMouseEvent *)> const& mouseMouveHandler);

Q_SIGNALS:

    void menuNonPointClick(QPoint const& pos);

protected:

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

    QTimer* _mouseMoveLimiterTimer;
    bool _mouseHandlerTriggerReady;
    std::function<bool(QMouseEvent *)> _mouseMoveHandler;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMAGEWIDGET_H
