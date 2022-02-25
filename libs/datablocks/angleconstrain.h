#ifndef STEREOVISIONAPP_ANGLECONSTRAIN_H
#define STEREOVISIONAPP_ANGLECONSTRAIN_H

#include "./project.h"
#include "./floatparameter.h"

#include "landmarkscollection.h"

namespace StereoVisionApp {

class AngleLandmarksTriplets;

class AngleConstrain : public DataBlock
{
	Q_OBJECT
public:
	AngleConstrain(Project* parent);

	floatParameter angleValue() const;
	void setAngleValue(const floatParameter &angleValue);

	floatParameter optimizedAngleValue() const;
	void setOptimizedAngleValue(const floatParameter &opt_angleValue);

	void clearOptimized() override;
	bool hasOptimizedParameters() const override;

	AngleLandmarksTriplets* getLandmarksTriplet(qint64 id) const;
	qint64 insertLandmarksTriplet(qint64 lm1Id, qint64 lm2Id, qint64 lm3Id);

Q_SIGNALS:

	void angleChanged(floatParameter angle);
	void optAngleChanged();

	void landmarksTripletAdded(qint64 id);

protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	void extendDataModel();

	floatParameter _angleValue;
	floatParameter _opt_angleValue;
};

class AngleLandmarksTriplets : public LandmarksTriplet
{
	Q_OBJECT
public:
	AngleLandmarksTriplets(AngleConstrain* parent) : LandmarksTriplet(parent) {

	}
protected:
	friend  class AngleConstrain;
};

class AngleConstrainFactory : public DataBlockFactory
{
	Q_OBJECT
public:
	explicit AngleConstrainFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual FactorizableFlags factorizable() const;
	virtual DataBlock* factorizeDataBlock(Project *parent = nullptr) const;

	virtual QString itemClassName() const;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_ANGLECONSTRAIN_H
