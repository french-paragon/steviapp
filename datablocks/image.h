#ifndef STEREOVISIONAPP_IMAGE_H
#define STEREOVISIONAPP_IMAGE_H

#include "./project.h"
#include "./floatparameter.h"

#include <QPointF>
#include <QSet>

namespace StereoVisionApp {

class Landmark;
class ImageLandmark;

class Image : public DataBlock
{
	Q_OBJECT
public:
	Image(Project* parent = nullptr);

	qint64 assignedCamera() const;
	void assignCamera(qint64 camId);

	QString getImageFile() const;
	void setImageFile(const QString &imageFile);

	floatParameter xCoord() const;
	void setXCoord(const floatParameter &x);

	floatParameter yCoord() const;
	void setYCoord(const floatParameter &y);

	floatParameter zCoord() const;
	void setZCoord(const floatParameter &z);

	floatParameter xRot() const;
	void setXRot(const floatParameter &rx);

	floatParameter yRot() const;
	void setYRot(const floatParameter &ry);

	floatParameter zRot() const;
	void setZRot(const floatParameter &rz);


	floatParameter optXCoord() const;
	void setOptXCoord(const floatParameter &o_x);
	void clearOptXCoord();

	floatParameter optYCoord() const;
	void setOptYCoord(const floatParameter &o_y);
	void clearOptYCoord();

	floatParameter optZCoord() const;
	void setOptZCoord(const floatParameter &o_z);
	void clearOptZCoord();

	floatParameter optXRot() const;
	void setOptXRot(const floatParameter &o_rx);
	void clearOptXRot();

	floatParameter optYRot() const;
	void setOptYRot(const floatParameter &o_ry);
	void clearOptYRot();

	floatParameter optZRot() const;
	void setOptZRot(const floatParameter &o_rz);
	void clearOptZRot();

	qint64 addImageLandmark(const QPointF &coordinates, bool uncertain = false, qreal sigma_pos = 1.0);
	qint64 addImageLandmark(const QPointF &coordinates, qint64 attacheLandmarkId, bool uncertain = false, qreal sigma_pos = 1.0);
	ImageLandmark* getImageLandmark(qint64 id) const;
	ImageLandmark* getImageLandmarkByLandmarkId(qint64 id);
	void clearImageLandmark(qint64 id);
	qint64 getImageLandMarkAt(QPointF const& coord, float tol = 3);
	QVector<qint64> getAttachedLandmarksIds() const;

	int countPointsRefered(QSet<qint64> const& excluded = {}) const;
	int countPointsRefered(QVector<qint64> const& excluded) const;

	void clearOptimized() override;
	bool hasOptimizedParameters() const override;

Q_SIGNALS:

	void assignedCameraChanged(qint64 id);

	void pointAdded(qint64 pt);
	void pointRemoved(qint64 pt);

	void imageFileChanged(QString fName);

	void xCoordChanged(floatParameter);
	void yCoordChanged(floatParameter);
	void zCoordChanged(floatParameter);

	void xRotChanged(floatParameter);
	void yRotChanged(floatParameter);
	void zRotChanged(floatParameter);


	void optXCoordChanged(floatParameter);
	void optYCoordChanged(floatParameter);
	void optZCoordChanged(floatParameter);

	void optXRotChanged(floatParameter);
	void optYRotChanged(floatParameter);
	void optZRotChanged(floatParameter);


protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	void extendDataModel();

	qint64 _assignedCamera;

	QString _imageFile;

	floatParameter _x;
	floatParameter _y;
	floatParameter _z;

	floatParameter _rx;
	floatParameter _ry;
	floatParameter _rz;


	floatParameter _o_x;
	floatParameter _o_y;
	floatParameter _o_z;

	floatParameter _o_rx;
	floatParameter _o_ry;
	floatParameter _o_rz;
};

class ImageLandmark : public DataBlock
{
	Q_OBJECT
public:

	static const QString ImageLandmarkClassName;

	explicit ImageLandmark(Image* parent = nullptr);

	qint64 attachedLandmarkid() const;
	void setAttachedLandmark(qint64 id);
	QString attachedLandmarkName() const;
	Landmark* attachedLandmark() const;

	floatParameter x() const;
	void setX(const floatParameter &x);
	void setX(float x);

	floatParameter y() const;
	void setY(const floatParameter &y);
	void setY(float y);

	void setImageCoordinates(QPointF const& point);
	QPointF imageCoordinates() const;


Q_SIGNALS:

	void attachedLandmarkidChanged(qint64 id);

	void xCoordChanged(floatParameter);
	void yCoordChanged(floatParameter);

	void coordsChanged();

protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	void referedCleared(QVector<qint64> const& referedId) override;

	qint64 _attachedLandmarkId;

	floatParameter _x;
	floatParameter _y;

	friend class Image;

};

class ImageFactory : public DataBlockFactory
{
	Q_OBJECT
public:
	explicit ImageFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual FactorizableFlags factorizable() const;
	virtual DataBlock* factorizeDataBlock(Project *parent = nullptr) const;

	virtual QString itemClassName() const;
	static QString imageClassName();
};
} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMAGE_H
