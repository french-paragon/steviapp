#ifndef STEREOVISIONAPP_VERTEXRIGIDBODYPOSE_H
#define STEREOVISIONAPP_VERTEXRIGIDBODYPOSE_H

#include "camerapose.h"
#include "g2o/core/base_vertex.h"

#include "../../datablocks/project.h"


namespace StereoVisionApp {

class VertexRigidBodyPose: public g2o::BaseVertex<6, CameraPose>, public DataBlockReference
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	VertexRigidBodyPose();

	virtual bool read(std::istream& is);

	virtual bool write(std::ostream& os) const;

	virtual void setToOriginImpl();

	virtual void oplusImpl(const double* update_);
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_VERTEXRIGIDBODYPOSE_H
