#ifndef STEREOVISIONAPP_RIGIDBODY_H
#define STEREOVISIONAPP_RIGIDBODY_H

#include "./project.h"
#include "./floatparameter.h"
#include "geometry/core.h"

#include <optional>

namespace StereoVisionApp {

class RigidBody : public DataBlock
{
	Q_OBJECT
public:
	explicit RigidBody(Project *parent = nullptr);
	explicit RigidBody(DataBlock *parent = nullptr);

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

	floatParameterGroup<3> optPos() const;
	void setOptPos(floatParameterGroup<3> const& o_pos);
	void clearOptPos();

	float optXCoord() const;
	void setOptXCoord(const float &x);

	float optYCoord() const;
	void setOptYCoord(const float &y);

	float optZCoord() const;
	void setOptZCoord(const float &z);

	floatParameterGroup<3> optRot() const;
	void setOptRot(floatParameterGroup<3> const& o_rot);
	void clearOptRot();

	float optXRot() const;
	void setOptXRot(const float &rx);

	float optYRot() const;
	void setOptYRot(const float &ry);

	float optZRot() const;
	void setOptZRot(const float &rz);

	/*!
	 * \brief getTransform is an accessor for the transform encoded by the rigid body
	 * \return the transform from the non optimized parameters, or nullopt if some parameters are missing.
	 */
	std::optional<StereoVision::Geometry::AffineTransform> getTransform() const;
	/*!
	 * \brief getOptTransform is an accessor for the transform encoded by the rigid body
	 * \return the transform from the optimized parameters, or nullopt if some parameters are missing.
	 */
	std::optional<StereoVision::Geometry::AffineTransform> getOptTransform() const;

Q_SIGNALS:

	void xCoordChanged(floatParameter);
	void yCoordChanged(floatParameter);
	void zCoordChanged(floatParameter);

	void xRotChanged(floatParameter);
	void yRotChanged(floatParameter);
	void zRotChanged(floatParameter);

	void optPosChanged();
	void optRotChanged();

	void optXCoordChanged(floatParameter);
	void optYCoordChanged(floatParameter);
	void optZCoordChanged(floatParameter);

	void optXRotChanged(floatParameter);
	void optYRotChanged(floatParameter);
	void optZRotChanged(floatParameter);

protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	floatParameter _x;
	floatParameter _y;
	floatParameter _z;

	floatParameter _rx;
	floatParameter _ry;
	floatParameter _rz;

	floatParameterGroup<3> _o_pos;
	floatParameterGroup<3> _o_rot;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_RIGIDBODY_H
