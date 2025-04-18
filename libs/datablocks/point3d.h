#ifndef STEREOVISIONAPP_POINT3D_H
#define STEREOVISIONAPP_POINT3D_H

#include "./project.h"
#include "./floatparameter.h"

#include <Eigen/Core>
#include <optional>

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

    floatParameter optXCoord() const;
    void setOptXCoord(const floatParameter &x);

    floatParameter optYCoord() const;
    void setOptYCoord(const floatParameter &y);

    floatParameter optZCoord() const;
    void setOptZCoord(const floatParameter &z);

	/*!
	 * \brief getPointVec is an accessor for the point coordinates
	 * \return the 3d vector with the non optimized position parameters.
	 */
	std::optional<Eigen::Vector3f> getPointVec() const;
	/*!
	 * \brief getOptPointVec is an accessor for the point coordinates
	 * \return the 3d vector with the optimized position parameters.
	 */
	std::optional<Eigen::Vector3f> getOptPointVec() const;

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
