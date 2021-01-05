#include "vertexcameraparam.h"

namespace StereoVisionApp {

const int CameraInnerVertexCollection::VerticesPerCam = 4;

VertexCameraParam::VertexCameraParam()
{

}

bool VertexCameraParam::read(std::istream& is) {
	is >> _estimate;
	return true;
}

bool VertexCameraParam::write(std::ostream& os) const {
	os << _estimate << ' ';
	return os.good();
}

void VertexCameraParam::setToOriginImpl() {
	_estimate = CamParam();
}

void VertexCameraParam::oplusImpl(const double* update_) {
	_estimate += update_;
}

} // namespace StereoVisionApp
