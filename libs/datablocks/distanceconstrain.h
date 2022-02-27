#ifndef STEREOVISIONAPP_DISTANCECONSTRAIN_H
#define STEREOVISIONAPP_DISTANCECONSTRAIN_H

#include "./floatparameter.h"
#include "./project.h"

#include "./landmarkscollection.h"

namespace StereoVisionApp {

class DistanceLandmarksPair;

class DistanceConstrain : public DataBlock
{
	Q_OBJECT
public:
	DistanceConstrain(Project* parent);

	floatParameter distanceValue() const;
	void setDistanceValue(const floatParameter &distanceValue);

	floatParameter optimizedDistanceValue() const;
	void setOptimizedDistanceValue(const floatParameter &opt_distanceValue);

	void clearOptimized() override;
	bool hasOptimizedParameters() const override;

	DistanceLandmarksPair* getLandmarksPair(qint64 id) const;
	qint64 insertLandmarksPair(qint64 lm1Id, qint64 lm2Id);

Q_SIGNALS:

	void distanceChanged(floatParameter angle);
	void optDistanceChanged();

	void landmarksPairAdded(qint64 id);

protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	void extendDataModel();

	floatParameter _distanceValue;
	floatParameter _opt_distanceValue;
};

class DistanceLandmarksPair : public LandmarksPair
{
	Q_OBJECT
public:
	DistanceLandmarksPair(DistanceConstrain* parent) : LandmarksPair(parent) {

	}
protected:
	friend class DistanceConstrain;
};

class DistanceConstrainFactory : public DataBlockFactory
{
	Q_OBJECT
public:
	explicit DistanceConstrainFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual FactorizableFlags factorizable() const;
	virtual DataBlock* factorizeDataBlock(Project *parent = nullptr) const;

	virtual QString itemClassName() const;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_DISTANCECONSTRAIN_H
