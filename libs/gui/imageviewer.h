#ifndef STEREOVISIONAPP_IMAGEVIEWER_H
#define STEREOVISIONAPP_IMAGEVIEWER_H



#include "./editor.h"

#include <StereoVision/QImageDisplayWidget/imageadapter.h>
#include <StereoVision/QImageDisplayWidget/imagewidget.h>

class QLabel;

namespace StereoVisionApp {

class ClickEventFilter : public QObject {
    Q_OBJECT

public:

    ClickEventFilter(QObject* parent = nullptr);

    bool eventFilter(QObject *object, QEvent *event);

Q_SIGNALS:

    void InfosDisplayRequested(QString);
};

class ImageViewer : public Editor
{
    Q_OBJECT

public:

    static const QString ImageViewerClassName;

    ImageViewer(QWidget *parent = nullptr);

    /*!
     * \brief setImage set the displayed image
     * \param imageAdapter the adapter for the image
     * \param manageImage if true the editor will destroy the image adapter if a new image is set
     */
    void setImage(QImageDisplay::ImageAdapter* imageAdapter, bool manageImage);

protected:

    bool _manageImage;
    QImageDisplay::ImageWidget* _img_widget;
    QLabel* _label;

};

class ImageViewerFactory : public EditorFactory
{
    Q_OBJECT
public:

    explicit ImageViewerFactory(QObject* parent = nullptr);

    virtual QString TypeDescrName() const;
    virtual QString itemClassName() const;
    virtual Editor* factorizeEditor(QWidget* parent) const;


};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMAGEVIEWER_H
