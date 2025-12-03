#ifndef STEREOVISIONAPP_RIGIDBODY_H
#define STEREOVISIONAPP_RIGIDBODY_H

#include "./project.h"
#include "./floatparameter.h"

#include <StereoVision/geometry/core.h>

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

    floatParameter optXCoord() const;
    void setOptXCoord(const floatParameter &x);

    floatParameter optYCoord() const;
    void setOptYCoord(const floatParameter &y);

    floatParameter optZCoord() const;
    void setOptZCoord(const floatParameter &z);

	floatParameterGroup<3> optRot() const;
	void setOptRot(floatParameterGroup<3> const& o_rot);
	void clearOptRot();

    floatParameter optXRot() const;
    void setOptXRot(const floatParameter &rx);

    floatParameter optYRot() const;
    void setOptYRot(const floatParameter &ry);

    floatParameter optZRot() const;
        void setOptZRot(const floatParameter &rz);

        /*!
         * \brief getTransform is an accessor for the transform encoded by the rigid body
         * \return the transform from the non optimized parameters, or nullopt if some parameters are missing.
         */
        std::optional<StereoVision::Geometry::AffineTransform<float>> getTransform() const;
        /*!
         * \brief getOptTransform is an accessor for the transform encoded by the rigid body
         * \return the transform from the optimized parameters, or nullopt if some parameters are missing.
         */
        std::optional<StereoVision::Geometry::AffineTransform<float>> getOptTransform() const;


        /*!
         * \brief setTransform set the non optimized transform of the rigid body
         * \param transform the transform to use.
         *
         * Note that this function do not change the uncertainty of the parameters.
         */
        void setTransform(StereoVision::Geometry::AffineTransform<float> const& transform);
        /*!
         * \brief setOptTransform set the optimized transform of the rigid body
         * \param transform the transform to use.
         *
         * Note that this function reset the uncertainty of the parameters.
         */
        void setOptTransform(StereoVision::Geometry::AffineTransform<float> const& transform);


        bool hasOptimizedParameters() const override;
        void clearOptimized() override;

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
