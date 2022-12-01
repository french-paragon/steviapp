#ifndef STEREOVISIONAPP_LOCALCOORDINATESYSTEM_H
#define STEREOVISIONAPP_LOCALCOORDINATESYSTEM_H

#include "./project.h"
#include "./rigidbody.h"
#include "./point3d.h"

namespace StereoVisionApp {

class Landmark;
class LandmarkLocalCoordinates;

class LocalCoordinateSystem : public RigidBody
{
	Q_OBJECT
public:
	explicit LocalCoordinateSystem(Project *parent = nullptr);

	qint64 addLandmarkLocalCoordinates(qint64 attachedLandmarkId,
									   floatParameter priorX = floatParameter(),
									   floatParameter priorY = floatParameter(),
									   floatParameter priorZ = floatParameter());
	LandmarkLocalCoordinates* getLandmarkLocalCoordinates(qint64 local_coordinates_id) const;
	LandmarkLocalCoordinates* getLandmarkLocalCoordinatesByLandmarkId(qint64 landmark_id) const;
	void clearLandmarkLocalCoordinates(qint64 attacheLandmarkId);
	QVector<qint64> getAttachedLandmarksIds() const;

	int countPointsRefered(QSet<qint64> const& excluded = {}) const;
	int countPointsRefered(QVector<qint64> const& excluded) const;

	QJsonObject getJsonRepresentation() const override;
	void setParametersFromJsonRepresentation(QJsonObject const& rep) override;

Q_SIGNALS:

	void pointAdded(qint64 pt);
	void pointRemoved(qint64 pt);


protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	void extendDataModel();

};

class LandmarkLocalCoordinates : public Point3D
{
	Q_OBJECT
public:

	explicit LandmarkLocalCoordinates(LocalCoordinateSystem* parent = nullptr);

	qint64 attachedLandmarkid() const;
	void setAttachedLandmark(qint64 id);
	QString attachedLandmarkName() const;
	Landmark* attachedLandmark() const;


Q_SIGNALS:

	void attachedLandmarkidChanged(qint64 id);

protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	void referedCleared(QVector<qint64> const& referedId) override;

	qint64 _attachedLandmarkId;

	friend class LocalCoordinateSystem;

};


class LocalCoordinateSystemFactory : public DataBlockFactory
{
	Q_OBJECT
public:
	explicit LocalCoordinateSystemFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual FactorizableFlags factorizable() const;
	virtual DataBlock* factorizeDataBlock(Project *parent = nullptr) const;

	virtual QString itemClassName() const;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LOCALCOORDINATESYSTEM_H
