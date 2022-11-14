#ifndef STEREOVISIONAPP_POINT3D_H
#define STEREOVISIONAPP_POINT3D_H

#include "./project.h"
#include "./floatparameter.h"

namespace StereoVisionApp {

class Point3D : public DataBlock
{
	Q_OBJECT
public:

	Q_PROPERTY(floatParameter xCoord READ xCoord WRITE setXCoord NOTIFY xCoordChanged)
	Q_PROPERTY(floatParameter yCoord READ yCoord WRITE setYCoord NOTIFY yCoordChanged)
	Q_PROPERTY(floatParameter zCoord READ zCoord WRITE setZCoord NOTIFY zCoordChanged)

	Q_PROPERTY(floatParameterGroup<3> optimizedPos READ optPos WRITE setOptPos NOTIFY optPosChanged)

	explicit Point3D(Project *parent = nullptr);
	explicit Point3D(DataBlock *parent = nullptr);

	floatParameter xCoord() const;
	void setXCoord(const floatParameter &x);

	floatParameter yCoord() const;
	void setYCoord(const floatParameter &y);

	floatParameter zCoord() const;
	void setZCoord(const floatParameter &z);


	floatParameterGroup<3> optPos() const;
	void setOptPos(floatParameterGroup<3> const& o_pos);
	void clearOptPos();

	float optXCoord() const;
	void setOptXCoord(const float &x);

	float optYCoord() const;
	void setOptYCoord(const float &y);

	float optZCoord() const;
	void setOptZCoord(const float &z);

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

	floatParameter _x;
	floatParameter _y;
	floatParameter _z;

	floatParameterGroup<3> _o_pos;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_POINT3D_H
