#include "imageviewer.h"

#include <QVBoxLayout>
#include <QLabel>

#include <QMouseEvent>

namespace StereoVisionApp {

const QString ImageViewer::ImageViewerClassName = "StereoVisionApp::ImageViewer";


ClickEventFilter::ClickEventFilter(QObject* parent) :
    QObject(parent)
{

}

bool ClickEventFilter::eventFilter(QObject *object, QEvent *event)
{

    QImageDisplay::ImageWidget* imgWidgt = qobject_cast<QImageDisplay::ImageWidget*>(object);

    if (imgWidgt != nullptr && event->type() == QEvent::MouseButtonPress) {
        QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
        if (mouseEvent->button() == Qt::LeftButton) {

            QPoint clickPos = imgWidgt->widgetToImageCoordinates(mouseEvent->pos()).toPoint();

            auto descr = imgWidgt->currentImage()->getOriginalChannelsInfos(clickPos);

            QString info = QString("Pos %1 %2: ").arg(clickPos.y()).arg(clickPos.x());

            for (auto& channelDescr : descr) {
                info += channelDescr.channelName + " " + channelDescr.channelValue + " ";
            }

            Q_EMIT InfosDisplayRequested(info);

            return true;
        } else {
            return false;
        }
    }

    return false;
}

ImageViewer::ImageViewer(QWidget *parent) :
    Editor(parent),
    _manageImage(false)
{

    QVBoxLayout* layout = new QVBoxLayout();

    _img_widget = new QImageDisplay::ImageWidget(this);
    _img_widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    ClickEventFilter* filter = new ClickEventFilter(_img_widget);
    _img_widget->installEventFilter(filter);

    layout->addWidget(_img_widget);

    _label = new QLabel(this);
    _label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

    layout->addWidget(_label);

    connect(filter, &ClickEventFilter::InfosDisplayRequested, _label, &QLabel::setText);

    setLayout(layout);
}

void ImageViewer::setImage(QImageDisplay::ImageAdapter* imageAdapter, bool manageImage) {

    QImageDisplay::ImageAdapter* old = nullptr;

    if (_manageImage) {
        old = _img_widget->currentImage();
    }

    _manageImage = true;
    _img_widget->setImage(imageAdapter);

    if (old != nullptr) {
        old->deleteLater();
    }

}

ImageViewerFactory::ImageViewerFactory(QObject* parent) :
    EditorFactory(parent)
{

}

QString ImageViewerFactory::TypeDescrName() const {
    return tr("Image viewer");
}
QString ImageViewerFactory::itemClassName() const {
    return ImageViewer::ImageViewerClassName;
}
Editor* ImageViewerFactory::factorizeEditor(QWidget* parent) const {
    return new ImageViewer(parent);
}

} // namespace StereoVisionApp
