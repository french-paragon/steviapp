#include "imagewidget.h"

#include <QPainter>
#include <QWheelEvent>
#include <QGuiApplication>

#include <cmath>

#include "datablocks/image.h"
#include "datablocks/landmark.h"

#include <QDebug>

namespace StereoVisionApp {


ImageWidgetDrawable::ImageWidgetDrawable(QWidget *parent) :
	QObject(parent)
{

}

ImageWidget::ImageWidget(QWidget *parent) :
	QWidget(parent),
	_translation(0, 0),
	_zoom(100),
	_currentImageDataBlock(nullptr),
	_img(),
	_c_img(),
	_activePoint(-1),
	_drawOnlyPoint(-1),
	_showPoints(true),
	_control_pressed_when_clicked(false)
{

}
bool ImageWidget::hasImage() const {
	return !_img.isNull();
}

int ImageWidget::zoom() const {
	return _zoom;
}

void ImageWidget::cachedZoomed() {

	if (_img.isNull() or _zoom > 100) {
		_c_img = QPixmap();
		return;
	}

	int w = _img.width()*_zoom/100;
	int h = _img.height()*_zoom/100;

	_c_img = _img.scaled(w, h, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);

}

void ImageWidget::imageDeleted() {
	_img = QPixmap();
	_c_img = QPixmap();
}

void ImageWidget::addLandmark(qint64 id) {
	ImageLandmark* imlm = _currentImageDataBlock->getImageLandmark(id);
	Landmark* lm = imlm->attachedLandmark();

	QMetaObject::Connection c = connect(imlm, &ImageLandmark::coordsChanged, this, static_cast<void (QWidget::*)()>(&QWidget::update));
	QMetaObject::Connection n = connect(lm, &QObject::objectNameChanged, this, static_cast<void (QWidget::*)()>(&QWidget::update));

	_landmarkConnections.insert(id, {c, n});
}
void ImageWidget::deleteLandmark(qint64 id) {

	for (QMetaObject::Connection & co : _landmarkConnections[id]) {
		disconnect(co);
	}

	_landmarkConnections.remove(id);
}
void ImageWidget::deleteAllLandmark() {
	for (qint64 id : _landmarkConnections.keys()) {
		deleteLandmark(id);
	}
}
void ImageWidget::imageLandmarkAdded(qint64 id) {
	addLandmark(id);
	update();
}
void ImageWidget::imageLandmarkDeleted(qint64 id) {

	deleteLandmark(id);
	update();

}

int ImageWidget::clipZoom(int rawZoom) {

	int zoom_percent = rawZoom;

	if (zoom_percent <= 0) {
		zoom_percent = 1;
	}

	if (zoom_percent > 1000) {
		zoom_percent = 1000;
	}

	return zoom_percent;
}

void ImageWidget::drawLandmark(QPainter* painter, QPointF const& imagePoint, bool isActive, QString const& ptn) {

	QString ptName = ptn;

	if (ptName.isEmpty()) {
		ptName = "No name";
	}

	QPointF wCoord = imageToWidgetCoordinates(imagePoint);
	int hPw = 12;
	QPointF hExtend(hPw, hPw);

	QColor pColor(106, 193, 74);
	QColor tColor(255, 255, 255);
	QColor black(0, 0, 0);

	if (isActive) {
		tColor = pColor;
		pColor = QColor(255, 255, 255);
	}

	QBrush fill;
	QPen line(black);
	line.setWidth(4);

	painter->setFont(QFont("sans", 8, QFont::Normal));
	int hMTw = 50;
	QRectF maxWidth(0, 0, 2*hMTw, 2*hMTw);
	QRectF b = painter->boundingRect(maxWidth, Qt::AlignHCenter|Qt::AlignTop, ptName);

	QRectF textBg(wCoord + QPointF(-hMTw, -hPw-b.height()) + b.topLeft(),
				  wCoord + QPointF(-hMTw, -hPw-b.height()) + b.bottomRight());

	QRectF outterRect(wCoord - hExtend, wCoord + hExtend);

	painter->setBrush(fill);
	painter->setPen(line);

	painter->drawRect(outterRect);
	painter->drawRect(textBg);
	painter->drawPoint(wCoord);

	line.setColor(pColor);
	line.setWidth(2);
	painter->setPen(line);

	painter->drawRect(outterRect);
	painter->drawPoint(wCoord);

	fill = QBrush(pColor);
	painter->setBrush(fill);

	painter->drawRect(textBg);

	line.setColor(tColor);
	painter->setPen(line);

	painter->drawText(textBg, Qt::AlignCenter|Qt::AlignTop, ptName);
}
void ImageWidget::drawOuterLandmark(QPainter* painter, QPointF const& imagePoint, bool isActive, const QString &ptn) {

	QString ptName = ptn;

	if (ptName.isEmpty()) {
		ptName = "No name";
	}

	QPointF wCoord = imageToWidgetCoordinates(imagePoint);

	QPointF constrained = wCoord;

	if (constrained.x() < 0) {
		constrained.setX(0);
	}
	if (constrained.y() < 0) {
		constrained.setY(0);
	}
	if (constrained.x() > size().width()) {
		constrained.setX(size().width());
	}
	if (constrained.y() > size().height()) {
		constrained.setY(size().height());
	}

	int hPw = 12;

	QPointF hExtend(hPw, hPw);

	QColor pColor(191, 51, 26);
	QColor tColor(255, 255, 255);
	QColor black(0, 0, 0);

	if (isActive) {
		tColor = pColor;
		pColor = QColor(255, 255, 255);
	}

	QBrush fill;
	QPen line(black);
	line.setWidth(4);

	painter->setFont(QFont("sans", 8, QFont::Normal));
	int hMTw = 50;
	QRectF maxWidth(0, 0, 2*hMTw, 2*hMTw);
	QRectF b = painter->boundingRect(maxWidth, Qt::AlignHCenter|Qt::AlignTop, ptName);

	QPolygonF arrow;

	QRectF textBg;

	if (constrained.x() == size().width() and constrained.y() == size().height()) {
		arrow << QPointF(constrained)
			  << QPointF(size().width() - hPw, size().height())
			  << QPointF(size().width(), size().height() - hPw);

		textBg = QRectF(constrained + QPointF(-2*hMTw -hPw , -hPw-b.height()) + b.topLeft(),
						  constrained + QPointF(-2*hMTw -hPw, -hPw-b.height()) + b.bottomRight());

	} else if (constrained.x() == size().width() and constrained.y() == 0) {
		arrow << QPointF(constrained)
			  << QPointF(size().width() - hPw, 0)
			  << QPointF(size().width(), hPw);

		textBg = QRectF(constrained + QPointF(-2*hMTw -hPw , -b.height()) + b.topLeft(),
						  constrained + QPointF(-2*hMTw -hPw, -b.height()) + b.bottomRight());

	} else if (constrained.x() == 0 and constrained.y() == size().height()) {
		arrow << QPointF(constrained)
			  << QPointF(hPw, size().height())
			  << QPointF(0, size().height() - hPw);

		textBg = QRectF(constrained + QPointF(hPw , -hPw-b.height()) + b.topLeft(),
						  constrained + QPointF(hPw, -hPw-b.height()) + b.bottomRight());

	}  else if (constrained.x() == 0 and constrained.y() == 0) {
		arrow << QPointF(constrained)
			  << QPointF(hPw, 0)
			  << QPointF(0, hPw);

		textBg = QRectF(constrained + QPointF(hPw , hPw-b.height()) + b.topLeft(),
						  constrained + QPointF(hPw, hPw-b.height()) + b.bottomRight());

	} else if (constrained.x() == size().width()) {
		arrow << QPointF(constrained)
			  << QPointF(size().width() - hPw, constrained.y() - hPw)
			  << QPointF(size().width() - hPw, constrained.y() + hPw);

		textBg = QRectF(constrained + QPointF(-2*hMTw -hPw , -b.height()) + b.topLeft(),
						  constrained + QPointF(-2*hMTw -hPw, -b.height()) + b.bottomRight());

	} else if (constrained.y() == size().height()) {
		arrow << QPointF(constrained)
			  << QPointF(constrained.x() - hPw, size().height() - hPw)
			  << QPointF(constrained.x() + hPw, size().height() - hPw);

		textBg = QRectF(constrained + QPointF(-hMTw -hPw , -hPw-b.height()) + b.topLeft(),
						  constrained + QPointF(-hMTw -hPw, -hPw-b.height()) + b.bottomRight());

	} else if (constrained.x() == 0) {
		arrow << QPointF(constrained)
			  << QPointF(hPw, constrained.y() - hPw)
			  << QPointF(hPw, constrained.y() + hPw);

		textBg = QRectF(constrained + QPointF(hPw , -hPw-b.height()) + b.topLeft(),
						  constrained + QPointF(hPw, -hPw-b.height()) + b.bottomRight());

	} else if (constrained.y() == 0) {
		arrow << QPointF(constrained)
			  << QPointF(constrained.x() - hPw, hPw)
			  << QPointF(constrained.x() + hPw, hPw);

		textBg = QRectF(constrained + QPointF(-hMTw -hPw , hPw) + b.topLeft(),
						  constrained + QPointF(-hMTw -hPw, hPw) + b.bottomRight());
	}



	painter->setBrush(fill);
	painter->setPen(line);

	painter->drawConvexPolygon(arrow);
	painter->drawRect(textBg);
	painter->drawPoint(constrained);

	line.setColor(pColor);
	line.setWidth(2);
	painter->setPen(line);

	painter->drawConvexPolygon(arrow);
	painter->drawPoint(constrained);

	fill = QBrush(pColor);
	painter->setBrush(fill);

	painter->drawRect(textBg);

	line.setColor(tColor);
	painter->setPen(line);

	painter->drawText(textBg, Qt::AlignCenter|Qt::AlignTop, ptName);
}

qint64 ImageWidget::activePoint() const
{
	return _activePoint;
}

void ImageWidget::setActivePoint(const qint64 &ap)
{
	qint64 activePoint = ap;

	if (_currentImageDataBlock == nullptr) {
		activePoint = -1;
	} else if (qobject_cast<ImageLandmark*>(_currentImageDataBlock->getById(activePoint)) == nullptr) {
		activePoint = -1;
	}

	if (activePoint != _activePoint) {
		_activePoint = activePoint;
		update();
	}
}

qint64 ImageWidget::pointAt(QPoint const& widgetCoordinate) {

	if (_currentImageDataBlock == nullptr) {
		return -1;
	}

	QPointF imCoord = widgetToImageCoordinates(widgetCoordinate);
	return _currentImageDataBlock->getImageLandMarkAt(imCoord, 600./_zoom);
}

void ImageWidget::drawOnlyPoint(qint64 only) {
	if (only != _drawOnlyPoint) {
		_drawOnlyPoint = only;
		update();
	}
}
qint64 ImageWidget::drawOnlyPoint() const {
	return _drawOnlyPoint;
}

void ImageWidget::displayPoints(bool display) {
	if (display != _showPoints) {
		_showPoints = display;
		update();
	}
}
bool ImageWidget::displayingPoints() const {
	return _showPoints;
}

void ImageWidget::addDrawable(ImageWidgetDrawable* drawable) {
	_drawables.push_back(drawable);
}

void ImageWidget::setZoom(int zp) {

	int zoom_percent = clipZoom(zp);

	if (zoom_percent != _zoom) {
		_zoom = zoom_percent;
		emit zoomChanged(zoom_percent);

		if (hasImage()) {
			cachedZoomed();
			setTranslation(translation());
			update();
		}
	}
}

Image* ImageWidget::currentImage() const {
	return _currentImageDataBlock;
}

void ImageWidget::setImage(Image *img) {

	if (_currentImageDataBlock != nullptr) {
		deleteAllLandmark();

		disconnect(_currentImageDataBlock, &QObject::destroyed, this, &ImageWidget::imageDeleted);
		disconnect(_currentImageDataBlock, &Image::pointAdded, this, &ImageWidget::imageLandmarkAdded);
		disconnect(_currentImageDataBlock, &Image::pointRemoved, this, &ImageWidget::imageLandmarkDeleted);
	}

	_currentImageDataBlock = img;

	if (img != nullptr) {

		for (qint64 lmid : img->listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName)) {
			addLandmark(lmid);
		}

		connect(_currentImageDataBlock, &QObject::destroyed, this, &ImageWidget::imageDeleted);
		connect(_currentImageDataBlock, &Image::pointAdded, this, &ImageWidget::imageLandmarkAdded);
		connect(_currentImageDataBlock, &Image::pointRemoved, this, &ImageWidget::imageLandmarkDeleted);

		QString p = img->getImageFile();
		if (!p.isEmpty()) {
			_img = QPixmap(p);
		} else if (!_img.isNull()) {
			_img = QPixmap();
		}
	} else {
		_img = QPixmap();
	}
	cachedZoomed();
	update();
}

QPoint ImageWidget::translation() const
{
	return _translation;
}

void ImageWidget::setTranslation(const QPoint &translation)
{
	QPoint n_t = translation;

	QSize s = rect().size();
	QSize is = (_c_img.isNull()) ? QSize(_img.width()*_zoom/100, _img.height()*_zoom/100) : _c_img.size();

	int max_w = std::abs(s.width() - is.width())/2;
	int max_h = std::abs(s.height() - is.height())/2;

	if (n_t.x() < -max_w) {
		n_t.setX(-max_w);
	} else if (n_t.x() > max_w) {
		n_t.setX(max_w);
	}

	if (n_t.y() < -max_h) {
		n_t.setY(-max_h);
	} else if (n_t.y() > max_h) {
		n_t.setY(max_h);
	}

	if (_translation != n_t) {
		_translation = n_t;
		update();
	}
}

QPointF ImageWidget::widgetToImageCoordinates(QPoint const& widget_pos) {
	QPoint middle = QPoint(rect().size().width(), rect().size().height())/2;
	QPointF src_img_coord = widget_pos - middle - _translation;
	src_img_coord /= _zoom;
	src_img_coord *= 100;

	QPointF middle_img = QPoint(_img.size().width(), _img.size().height())/2;
	src_img_coord += middle_img;
	return src_img_coord;
}
QPointF ImageWidget::imageToWidgetCoordinates(QPointF const& image_pos) {
	QPointF middle_img = QPoint(_img.size().width(), _img.size().height())/2;
	QPointF local_img_coord = image_pos - middle_img;

	local_img_coord *= _zoom;
	local_img_coord /= 100;

	QPoint middle = QPoint(rect().size().width(), rect().size().height())/2;
	QPointF wCoord = local_img_coord + middle + _translation;

	return wCoord;
}

void ImageWidget::paintEvent(QPaintEvent *) {

	QPainter painter(this);

	QSize s = rect().size();
	painter.fillRect(0, 0, s.width(), s.height(), QColor(255, 255, 255));

	if (_img.isNull()) {
		return;
	}

	QSize is = (_c_img.isNull()) ? QSize(_img.width()*_zoom/100, _img.height()*_zoom/100) : _c_img.size();

	int pw_x = (is.width() >= s.width()) ? 0 : (s.width() - is.width())/2 + _translation.x();
	int pw_y = (is.height() >= s.height()) ? 0 : (s.height() - is.height())/2 + _translation.y();

	int pi_x = (is.width() < s.width()) ? 0 : (is.width() - s.width())/2 - _translation.x();
	int pi_y = (is.height() < s.height()) ? 0 : (is.height() - s.height())/2 - _translation.y();

	int pi_w = (is.width() < s.width()) ? is.width() : s.width();
	int pi_h = (is.height() < s.height()) ? is.height() : s.height();

	QRectF imageCover = (_c_img.isNull()) ? QRectF(pi_x*100./_zoom, pi_y*100./_zoom, pi_w*100./_zoom, pi_h*100./_zoom) : QRectF(pi_x, pi_y, pi_w, pi_h);

	painter.drawPixmap(QRectF(pw_x, pw_y, pi_w, pi_h), (_c_img.isNull()) ? _img : _c_img, imageCover);

	QVector<qint64> pointsids = _currentImageDataBlock->listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName);

	ImageLandmark* active = nullptr;

	if (_showPoints) {

		for(qint64 id : pointsids) {
			ImageLandmark* lm = _currentImageDataBlock->getImageLandmark(id);

			if (id == activePoint()) {
				active = lm;
				continue;
			}

			if (_drawOnlyPoint >= 0 and _drawOnlyPoint != lm->attachedLandmarkid()) {
				continue;
			}

			auto coords = lm->imageCoordinates();
			auto extend = _img.size();

			if (coords.x() < 0 or coords.y() < 0 or
					coords.x() > extend.width() or coords.y() > extend.height() ) {
				drawOuterLandmark(&painter, coords, false, lm->attachedLandmarkName());
			} else {
				drawLandmark(&painter, lm->imageCoordinates(), false, lm->attachedLandmarkName());
			}
		}

		if (active != nullptr) {

			bool ok = false;

			auto coords = active->imageCoordinates();
			auto extend = _img.size();

			if (_drawOnlyPoint >= 0 and _drawOnlyPoint == active->attachedLandmarkid()) {
				ok = true;
			} else if (_drawOnlyPoint < 0) {
				ok = true;
			}

			if (ok) {
				if (coords.x() < 0 or coords.y() < 0 or
						coords.x() > extend.width() or coords.y() > extend.height() ) {
					drawOuterLandmark(&painter, coords, true, active->attachedLandmarkName());
				} else {
					drawLandmark(&painter, coords, true, active->attachedLandmarkName());
				}
			}
		}
	}

	QTransform img2widget;
	img2widget.translate(_translation.x()-(is.width() - s.width())/2, _translation.y()-(is.height() - s.height())/2);
	img2widget.scale(_zoom/100., _zoom/100.);

	for (ImageWidgetDrawable* d : _drawables) {
		d->paintItem(&painter, img2widget);
	}

}

void ImageWidget::resizeEvent(QResizeEvent *) {
	setTranslation(_translation);
	update();
}

void ImageWidget::mousePressEvent(QMouseEvent *e) {

	QGuiApplication* gapp = qGuiApp;

	if (gapp != nullptr) {
		Qt::KeyboardModifiers kmods = gapp->keyboardModifiers();

		if (kmods & Qt::ControlModifier) {
			_control_pressed_when_clicked = true;
		} else {
			_control_pressed_when_clicked = false;
		}
	}

	_previously_pressed = e->buttons();

	if (_previously_pressed == Qt::LeftButton or _previously_pressed == Qt::MiddleButton) {
		_motion_origin_pos = e->pos();
		_motion_origin_t = translation();

		if (_previously_pressed == Qt::LeftButton) {
			setActivePoint(pointAt(e->pos()));
		}

		e->accept();
	} else if (_previously_pressed == Qt::RightButton) {
		qint64 pt = pointAt(e->pos());
		setActivePoint(pt);

		if (pt >= 0) {
			Q_EMIT menuPointClick(pt, e->pos());
		} else {
			Q_EMIT menuNonPointClick(e->pos());
		}
	} else {
		e->ignore();
	}

}
void ImageWidget::mouseReleaseEvent(QMouseEvent *e) {

	if (_previously_pressed == Qt::LeftButton) {
		if (_control_pressed_when_clicked) {
			emit newPointClick(widgetToImageCoordinates(e->pos()));
		}
	}
}

void ImageWidget::mouseMoveEvent(QMouseEvent *e) {

	if (_currentImageDataBlock == nullptr) {
		return;
	}

	if (_previously_pressed == Qt::LeftButton or _previously_pressed == Qt::MiddleButton) {
		if (!_control_pressed_when_clicked) {
			if (_previously_pressed == Qt::LeftButton and _activePoint >= 0) {
				ImageLandmark* ilm = _currentImageDataBlock->getImageLandmark(_activePoint);
				if (ilm != nullptr) {
					ilm->setImageCoordinates(widgetToImageCoordinates(e->pos()));
					update();
				}
			} else {
				QPoint delta_t = e->pos() - _motion_origin_pos;
				setTranslation(_motion_origin_t + delta_t);
			}
			e->accept();
		}
	} else {
		e->ignore();
	}

}

void ImageWidget::wheelEvent(QWheelEvent *e) {

	QPoint d = e->angleDelta();
	if (d.y() == 0){
		e->ignore();
		return;
	}

	if (e->buttons() == Qt::NoButton) {
		int delta = d.y()*zoom()/500;
		if (delta == 0) {
			delta = (d.y() > 0) - (d.y() < 0);
		}
		int old_zoom = zoom();
		int n_zoom = clipZoom(old_zoom + delta);

		QPoint mousePos = e->position().toPoint();
		QPoint oldTranslation = translation();

		QPoint wCenter(width()/2, height()/2);

		QPoint imPt = mousePos - wCenter - oldTranslation;

		imPt.rx() *= n_zoom;
		imPt.rx() /= old_zoom;

		imPt.ry() *= n_zoom;
		imPt.ry() /= old_zoom;

		_translation = mousePos - wCenter -imPt;

		setZoom(n_zoom);

		e->accept();
	} else {
		e->ignore();
	}

}


} // namespace StereoVisionApp
