#ifndef STEREOVISIONAPP_LANDMARK_H
#define STEREOVISIONAPP_LANDMARK_H

#include "./project.h"
#include "./floatparameter.h"

#include <QSet>

namespace StereoVisionApp {

class Landmark : public DataBlock
{
	Q_OBJECT
public:

	Q_PROPERTY(floatParameter xCoord READ xCoord WRITE setXCoord NOTIFY xCoordChanged)
	Q_PROPERTY(floatParameter yCoord READ yCoord WRITE setYCoord NOTIFY yCoordChanged)
	Q_PROPERTY(floatParameter zCoord READ zCoord WRITE setZCoord NOTIFY zCoordChanged)

	Q_PROPERTY(floatParameter optimizedX READ optimizedX WRITE setOptimisedX NOTIFY optXCoordChanged)
	Q_PROPERTY(floatParameter optimizedY READ optimizedY WRITE setOptimisedY NOTIFY optYCoordChanged)
	Q_PROPERTY(floatParameter optimizedZ READ optimizedZ WRITE setOptimisedZ NOTIFY optZCoordChanged)

	explicit Landmark(Project* parent = nullptr);

	floatParameter xCoord() const;
	void setXCoord(const floatParameter &x);

	floatParameter yCoord() const;
	void setYCoord(const floatParameter &y);

	floatParameter zCoord() const;
	void setZCoord(const floatParameter &z);


	floatParameter optimizedX() const;
	void setOptimisedX(const floatParameter &o_x);
	void clearOptimisedX();

	floatParameter optimizedY() const;
	void setOptimisedY(const floatParameter &o_x);
	void clearOptimisedY();

	floatParameter optimizedZ() const;
	void setOptimisedZ(const floatParameter &o_x);
	void clearOptimisedZ();

	int countImagesRefering(QSet<qint64> const& excluded = {}) const;
	int countImagesRefering(QVector<qint64> const& excluded) const;

Q_SIGNALS:

	void xCoordChanged(floatParameter);
	void yCoordChanged(floatParameter);
	void zCoordChanged(floatParameter);

	void optXCoordChanged(floatParameter);
	void optYCoordChanged(floatParameter);
	void optZCoordChanged(floatParameter);

protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	void extendDataModel();

	floatParameter _x;
	floatParameter _y;
	floatParameter _z;

	floatParameter _o_x;
	floatParameter _o_y;
	floatParameter _o_z;
};

class LandmarkFactory : public DataBlockFactory
{
	Q_OBJECT
public:
	explicit LandmarkFactory(QObject* parent = nullptr);

	virtual QString TypeDescrName() const;
	virtual FactorizableFlags factorizable() const;
	virtual DataBlock* factorizeDataBlock(Project *parent = nullptr) const;

	virtual QString itemClassName() const;
	static QString landmarkClassName();
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LANDMARK_H
