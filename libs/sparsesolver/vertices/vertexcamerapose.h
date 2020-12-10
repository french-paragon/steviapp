#ifndef VERTEXCAMERAPOSE_H
#define VERTEXCAMERAPOSE_H

#include "camerapose.h"
#include "g2o/core/base_vertex.h"

/*!
 * \brief The VertexCameraPose class represent an element of R^3 x SO(3), meaning the position and orientation of a camera.
 */
class VertexCameraPose : public g2o::BaseVertex<6, CameraPose>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	VertexCameraPose();

	virtual bool read(std::istream& is);

	virtual bool write(std::ostream& os) const;

	virtual void setToOriginImpl();

	virtual void oplusImpl(const double* update_);
};

#endif // VERTEXCAMERAPOSE_H
