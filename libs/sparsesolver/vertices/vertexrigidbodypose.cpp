#include "vertexrigidbodypose.h"

namespace StereoVisionApp {

VertexRigidBodyPose::VertexRigidBodyPose()
{

}

bool VertexRigidBodyPose::read(std::istream& is) {
	is >> _estimate;
	return true;
}

bool VertexRigidBodyPose::write(std::ostream& os) const {
	os << _estimate;
	return os.good();
}

void VertexRigidBodyPose::setToOriginImpl() {
	_estimate = CameraPose();
}

void VertexRigidBodyPose::oplusImpl(const double* update_)  {
  Eigen::Map<const CameraPose::Vector6d> update(update_);
  setEstimate(CameraPose::exp(update)*estimate());
}

} // namespace StereoVisionApp
