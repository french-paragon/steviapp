#ifndef VERTEXCAMERAPOSE_H
#define VERTEXCAMERAPOSE_H

#include "camerapose.h"
#include "g2o/core/base_vertex.h"

#include "../../datablocks/project.h"

namespace StereoVisionApp {

/*!
 * \brief The VertexCameraPose class represent an element of R^3 x SO(3), encoding the position and orientation of a camera.
 */
class VertexCameraPose : public g2o::BaseVertex<6, CameraPose>, public DataBlockReference
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	VertexCameraPose();

	virtual bool read(std::istream& is);

	virtual bool write(std::ostream& os) const;

	virtual void setToOriginImpl();

	virtual void oplusImpl(const double* update_);
};

} // namespace StereoVisionApp

#endif // VERTEXCAMERAPOSE_H
