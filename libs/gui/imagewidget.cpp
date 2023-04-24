#include "imagewidget.h"

#include <QDebug>

#include <QMouseEvent>

namespace StereoVisionApp {

ImageWidget::ImageWidget(QWidget *parent) :
    QImageDisplay::ImageWidget(parent)
{

}

void ImageWidget::mousePressEvent(QMouseEvent *e) {

    Qt::MouseButtons previously_pressed = e->buttons();

    if (previously_pressed == Qt::RightButton) {

        Q_EMIT menuNonPointClick(e->pos());
	} else {
        QImageDisplay::ImageWidget::mousePressEvent(e);
	}

}


} // namespace StereoVisionApp
