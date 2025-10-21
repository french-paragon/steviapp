#ifndef STEREOVISIONAPP_LOCALCOORDINATESYSTEM_H
#define STEREOVISIONAPP_LOCALCOORDINATESYSTEM_H

#include "./project.h"
#include "./rigidbody.h"
#include "./point3d.h"

namespace StereoVisionApp {

class Landmark;
class LandmarkLocalCoordinates;
class Trajectory;
class Mounting;

class LocalCoordinateSystem : public RigidBody
{
	Q_OBJECT
public:
	explicit LocalCoordinateSystem(Project *parent = nullptr);

    qint64 assignedTrajectory() const;
    Trajectory* getAssignedTrajectory() const;
    void assignTrajectory(qint64 trajId);
    QString getAssignedTrajectoryName() const;

    qint64 assignedMounting() const;
    Mounting* getAssignedMounting() const;
    void assignMounting(qint64 mountId);
    QString getAssignedMountingName() const;

	qint64 addLandmarkLocalCoordinates(qint64 attachedLandmarkId,
									   floatParameter priorX = floatParameter(),
									   floatParameter priorY = floatParameter(),
                                       floatParameter priorZ = floatParameter(),
                                       double time = 0);
	LandmarkLocalCoordinates* getLandmarkLocalCoordinates(qint64 local_coordinates_id) const;
	LandmarkLocalCoordinates* getLandmarkLocalCoordinatesByLandmarkId(qint64 landmark_id) const;
	void clearLandmarkLocalCoordinates(qint64 attacheLandmarkId);
	QVector<qint64> getAttachedLandmarksIds() const;

	int countPointsRefered(QSet<qint64> const& excluded = {}) const;
	int countPointsRefered(QVector<qint64> const& excluded) const;

	QJsonObject getJsonRepresentation() const override;
	void setParametersFromJsonRepresentation(QJsonObject const& rep) override;

Q_SIGNALS:

    void assignedTrajectoryChanged();
    void assignedMountingChanged();

	void pointAdded(qint64 pt);
	void pointRemoved(qint64 pt);


protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	void extendDataModel();

    qint64 _assignedTrajectory; //the trajectory the local coordinate system follow (if enabled, local landmarks coordinates have to get a time attached).
    qint64 _assignedMounting; //the lever arm between the local coordinate system and its trajectory.

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

    double time() const;
    void setTime(double time);

Q_SIGNALS:

    void timeChanged(double time);
	void attachedLandmarkidChanged(qint64 id);

protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	void referedCleared(QVector<qint64> const& referedId) override;

	qint64 _attachedLandmarkId;
    double _time;

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
