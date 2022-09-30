#ifndef STEREOVISIONAPP_IMAGEWIDGET_H
#define STEREOVISIONAPP_IMAGEWIDGET_H

#include <QWidget>
#include <QMap>
#include <QMetaObject>
#include <QPen>
#include <QBrush>
#include <QPainter>

namespace StereoVisionApp {

class Image;
class ImageWidgetDrawable;

class ImageWidget : public QWidget
{
	Q_OBJECT
public:
	explicit ImageWidget(QWidget *parent = nullptr);

	bool hasImage() const;

	int zoom() const;
	void setZoom(int zoom_percent);

	Image* currentImage() const;
	void setImage(Image* img);

	QPoint translation() const;
	void setTranslation(const QPoint &translation);

	qint64 activePoint() const;
	void setActivePoint(const qint64 &activePoint);

	qint64 pointAt(QPoint const& widgetCoordinate);

	void drawOnlyPoint(qint64 only);
	qint64 drawOnlyPoint() const;

	void displayPoints(bool display);
	bool displayingPoints() const;

	void addDrawable(ImageWidgetDrawable* drawable);

Q_SIGNALS:

	void zoomChanged(int zoom_percent);
	void translationChanged(QPoint translation);

	void newPointClick(QPointF imagePos);
	void menuPointClick(qint64 id, QPoint const& pos);
	void menuNonPointClick(QPoint const& pos);

	void activePointChanged(qint64 activePt);

protected:

	QPointF widgetToImageCoordinates(QPoint const& widget_pos);
	QPointF imageToWidgetCoordinates(QPointF const& image_pos);

	void paintEvent(QPaintEvent *event) override;
	void resizeEvent(QResizeEvent *event) override;
	void mousePressEvent(QMouseEvent *event) override;
	void mouseReleaseEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event) override;
	void cachedZoomed();

	void imageDeleted();
	void addLandmark(qint64 id);
	void deleteLandmark(qint64 id);
	void deleteAllLandmark();
	void imageLandmarkAdded(qint64 id);
	void imageLandmarkDeleted(qint64 id);

	static int clipZoom(int rawZoom);

	void drawLandmark(QPainter* painter, QPointF const& imagePoint, bool isActive, const QString &ptName);
	void drawOuterLandmark(QPainter* painter, QPointF const& imagePoint, bool isActive, const QString &ptName);

	QPoint _translation;
	int _zoom;

	Image* _currentImageDataBlock;
	QPixmap _img;
	QPixmap _c_img;

	qint64 _activePoint;
	qint64 _drawOnlyPoint;
	bool _showPoints;

	QVector<ImageWidgetDrawable*> _drawables;

private:

	Qt::MouseButtons _previously_pressed;
	bool _control_pressed_when_clicked;

	QPoint _motion_origin_pos;
	QPoint _motion_origin_t;

	QMap<qint64, QList<QMetaObject::Connection>> _landmarkConnections;

};

class ImageWidgetDrawable : public QObject
{
	Q_OBJECT
public:
	explicit ImageWidgetDrawable(QWidget *parent = nullptr);

	inline void paintItem(QPainter* painter, QTransform const& imageToPaintArea) const {
		_imageToPaintArea = imageToPaintArea;
		paintItemImpl(painter);
	}

protected:

	virtual void paintItemImpl(QPainter* painter) const = 0;

	inline void drawPoint(QPainter* painter, QPointF const& point, QColor color, float radius) const {
		QPen pen;
		pen.setColor(color);
		pen.setWidthF(radius);
		painter->setPen(pen);
		painter->drawPoint(_imageToPaintArea.map(point));
	}
	inline void drawLine(QPainter* painter, QLineF const& line, QColor color, float width) const {
		QPen pen;
		pen.setColor(color);
		pen.setWidthF(width);
		painter->setPen(pen);
		painter->drawLine(_imageToPaintArea.map(line));
	}

	inline void drawPoints(QPainter* painter, QPolygonF const& points, QColor color, float radius) const {
		QPen pen;
		pen.setColor(color);
		pen.setWidthF(radius);
		painter->setPen(pen);
		painter->drawPoints(_imageToPaintArea.map(points));
	}
	inline void drawLines(QPainter* painter, QList<QLineF> const& lines, QColor color, float width) const {
		QPen pen;
		pen.setColor(color);
		pen.setWidthF(width);
		painter->setPen(pen);

		QVector<QLineF> transformedLines;
		transformedLines.reserve(lines.size());

		for (QLineF const& line : lines) {
			transformedLines.push_back(_imageToPaintArea.map(line));
		}

		painter->drawLines(transformedLines);
	}
	inline void drawLines(QPainter* painter, QPolygonF const& points, QColor color, float width) const {
		QPen pen;
		pen.setColor(color);
		pen.setWidthF(width);
		painter->setPen(pen);
		painter->drawPolyline(_imageToPaintArea.map(points));
	}

private:

	mutable QTransform _imageToPaintArea;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMAGEWIDGET_H
