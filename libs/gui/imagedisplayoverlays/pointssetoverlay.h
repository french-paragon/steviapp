#ifndef STEREOVISIONAPP_POINTSSETOVERLAY_H
#define STEREOVISIONAPP_POINTSSETOVERLAY_H

#include <qImageDisplayWidget/overlay.h>

#include <QObject>

namespace StereoVisionApp {

class PointsSetOverlay : public QImageDisplay::Overlay
{
    Q_OBJECT
public:
    PointsSetOverlay(QWidget *parent = nullptr);

    void setPointSet(QVector<QPointF> const& set);

    void setColor(const QColor &newColor);

    void setRadius(float newRadius);

    void hide();
    void show();

protected:

    inline void requestFullRepainting() {
        Q_EMIT repaintingRequested(QRect());
    }

    void paintItemImpl(QPainter* painter) const override;

    bool _show;

    QVector<QPointF> _pointsSet;
    QColor _color;
    float _radius;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_POINTSSETOVERLAY_H
