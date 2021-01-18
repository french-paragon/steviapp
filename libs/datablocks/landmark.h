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

	Q_PROPERTY(floatParameterGroup<3> optimizedPos READ optPos WRITE setOptPos NOTIFY optPosChanged)

	explicit Landmark(Project* parent = nullptr);

	floatParameter xCoord() const;
	void setXCoord(const floatParameter &x);

	floatParameter yCoord() const;
	void setYCoord(const floatParameter &y);

	floatParameter zCoord() const;
	void setZCoord(const floatParameter &z);


	floatParameterGroup<3> optPos() const;
	void setOptPos(floatParameterGroup<3> const& o_pos);
	void clearOptPos();

	int countImagesRefering(QSet<qint64> const& excluded = {}) const;
	int countImagesRefering(QVector<qint64> const& excluded) const;

	int countImagesReferingInList(QSet<qint64> const& included) const;
	int countImagesReferingInList(QVector<qint64> const& included) const;

	QSet<qint64> getViewingImgInList(QSet<qint64> const& included) const;

	void clearOptimized() override;
	bool hasOptimizedParameters() const override;

Q_SIGNALS:

	void xCoordChanged(floatParameter);
	void yCoordChanged(floatParameter);
	void zCoordChanged(floatParameter);

	void optPosChanged();

protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	void extendDataModel();

	floatParameter _x;
	floatParameter _y;
	floatParameter _z;

	floatParameterGroup<3> _o_pos;

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
