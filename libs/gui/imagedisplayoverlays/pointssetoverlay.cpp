#include "pointssetoverlay.h"

namespace StereoVisionApp {

PointsSetOverlay::PointsSetOverlay(QWidget *parent) :
    QImageDisplay::Overlay(parent),
    _pointsSet(),
    _color(255, 0, 0),
    _radius(3)
{

}

void PointsSetOverlay::setPointSet(QVector<QPointF> const& set) {
    _pointsSet = set;
    requestFullRepainting();
}

void PointsSetOverlay::paintItemImpl(QPainter* painter) const {

    drawPoints(painter, QPolygonF(_pointsSet), _color, _radius);

}

void PointsSetOverlay::setRadius(float newRadius)
{
    _radius = newRadius;
    requestFullRepainting();
}

void PointsSetOverlay::setColor(const QColor &newColor)
{
    _color = newColor;
    requestFullRepainting();
}

void PointsSetOverlay::hide() {
    setDisplay(false);
}
void PointsSetOverlay::show() {
    setDisplay(true);
}

} // namespace StereoVisionApp
