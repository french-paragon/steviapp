#ifndef STEREOVISIONAPP_LABELLEDPOINTSOVERLAY_H
#define STEREOVISIONAPP_LABELLEDPOINTSOVERLAY_H

#include <StereoVision/QImageDisplayWidget/overlay.h>

#include <QObject>

namespace QImageDisplay {
    class ImageWidget;
}

namespace StereoVisionApp {

class LabelledPointsOverlay : public QImageDisplay::Overlay
{
    Q_OBJECT
public:
    LabelledPointsOverlay(QWidget *parent = nullptr);

    void setPointSet(QVector<QPointF> const& set);

    void setColorMap(const QVector<QColor> &colorMap);

    bool eventFilter(QObject *watched, QEvent *event) override;

    inline qint64 activePoint() const {
        return _activePoint;
    }

    void setActivePoint(const qint64 &ap);

Q_SIGNALS:

    void pointClicked(int id);
    void activePointChanged(int id);

protected:

    inline void requestFullRepainting() {
        Q_EMIT repaintingRequested(QRect());
    }

    bool mousePressEvent(QImageDisplay::ImageWidget* interactionWidget, QMouseEvent *event);

    void paintItemImpl(QPainter* painter) const override;
    void drawDecoratedPoint(QPainter* painter, QPointF point, bool isActive, QString label, QColor color = QColor(255, 150, 20)) const;

    qint64 pointAt(QImageDisplay::ImageWidget* interactionWidget, QPoint const& widgetCoordinate) const;

    QVector<QPointF> _pointsSet;
    QVector<QString> _labels;
    QVector<QColor> _pointsColor;
    float _radius;

    qint64 _activePoint;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LABELLEDPOINTSOVERLAY_H
