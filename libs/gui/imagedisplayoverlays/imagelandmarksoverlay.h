#ifndef STEREOVISION_IMAGELANDMARKSOVERLAY_H
#define STEREOVISION_IMAGELANDMARKSOVERLAY_H

#include <StereoVision/QImageDisplayWidget/overlay.h>

#include <QMap>
#include <QList>
#include <QMetaObject>

namespace QImageDisplay {
    class ImageWidget;
}

namespace StereoVisionApp {

class Image;

/*!
 * \brief The ImageLandmarksOverlay class display the image landmarks of an image
 *
 * The class is meant to be used both as an overlay for display and
 * as an event filter for interaction.
 */
class ImageLandmarksOverlay : public QImageDisplay::Overlay
{
    Q_OBJECT
public:
    ImageLandmarksOverlay(QWidget *parent = nullptr);

    inline bool hasImageDatablock() const {
        return _currentImageDataBlock != nullptr;
    }

    inline Image* currentImageDatablock() const {
        return _currentImageDataBlock;
    }
    void setImageDatablock(Image* img);

    bool eventFilter(QObject *watched, QEvent *event) override;

    inline qint64 activePoint() const {
        return _activePoint;
    }

    void setActivePoint(const qint64 &ap);

    inline void displaySinglePoint(qint64 pointId) {
        if (pointId != _drawOnlyPoint) {
            _drawOnlyPoint = pointId;
            requestFullRepainting();
        }
    }
    inline qint64 singleDisplayedPoint() const {
        return _drawOnlyPoint;
    }

Q_SIGNALS:

    void newPointClick(QPointF imagePos);
    void menuPointClick(qint64 id, QImageDisplay::ImageWidget* widget, QPoint const& pos);

    void activePointChanged(qint64 activePt);

protected:

    void requestFullRepainting();

    void paintItemImpl(QPainter* painter) const override;

    void registerLandmark(qint64 id);
    void unregisterLandmark(qint64 id);
    void unregisterAllLandmark();

    void imageDeleted();
    void imageLandmarkAdded(qint64 id);
    void imageLandmarkDeleted(qint64 id);

    void drawLandmark(QPainter* painter, QPointF const& imagePoint, bool isActive, const QString &ptName) const;
    void drawOuterLandmark(QPainter* painter, QPointF const& imagePoint, bool isActive, const QString &ptName) const;

    bool mousePressEvent(QImageDisplay::ImageWidget* interactionWidget, QMouseEvent *event);
    bool mouseReleaseEvent(QImageDisplay::ImageWidget* interactionWidget, QMouseEvent *event);
    bool mouseMoveEvent(QImageDisplay::ImageWidget* interactionWidget, QMouseEvent *event);

    qint64 pointAt(QImageDisplay::ImageWidget* interactionWidget, QPoint const& widgetCoordinate) const;

    Image* _currentImageDataBlock;

    QMap<qint64, QList<QMetaObject::Connection>> _landmarkConnections;

    qint64 _activePoint;
    qint64 _drawOnlyPoint;

    Qt::MouseButtons _previously_pressed;
    bool _control_pressed_when_clicked;

    QPoint _motion_origin_pos;
    QPoint _motion_origin_t;

};

} // namespace StereoVisionApp

#endif // STEREOVISION_IMAGELANDMARKSOVERLAY_H
