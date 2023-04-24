#ifndef STEREOVISIONAPP_IMAGEWIDGET_H
#define STEREOVISIONAPP_IMAGEWIDGET_H

#include <qImageDisplayWidget/imagewidget.h>
#include <qImageDisplayWidget/imageadapter.h>

#include <QWidget>
#include <QMap>
#include <QMetaObject>
#include <QPen>
#include <QBrush>
#include <QPainter>

namespace StereoVisionApp {

class Image;

class ImageWidget : public QImageDisplay::ImageWidget
{
	Q_OBJECT
public:
    explicit ImageWidget(QWidget *parent = nullptr);

Q_SIGNALS:

    void menuNonPointClick(QPoint const& pos);

protected:

	void mousePressEvent(QMouseEvent *event) override;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMAGEWIDGET_H
