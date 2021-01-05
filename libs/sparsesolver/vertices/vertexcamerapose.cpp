#include "vertexcamerapose.h"

namespace StereoVisionApp {

VertexCameraPose::VertexCameraPose()
{

}

bool VertexCameraPose::read(std::istream& is) {
	is >> _estimate;
	return true;
}

bool VertexCameraPose::write(std::ostream& os) const {
	os << _estimate;
	return os.good();
}

void VertexCameraPose::setToOriginImpl() {
	_estimate = CameraPose();
  }

void VertexCameraPose::oplusImpl(const double* update_)  {
  Eigen::Map<const CameraPose::Vector6d> update(update_);
  setEstimate(CameraPose::exp(update)*estimate());
}

} // namespace StereoVisionApp
