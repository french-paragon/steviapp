#include "vertexcameratangentialdistortion.h"

namespace StereoVisionApp {

VertexCameraTangentialDistortion::VertexCameraTangentialDistortion()
{

}

bool VertexCameraTangentialDistortion::read(std::istream& is) {

	is >> _estimate(0);
	is >> _estimate(1);

	return true;
}

bool VertexCameraTangentialDistortion::write(std::ostream& os) const {

	os << _estimate(0) << " " << _estimate(1) << " ";
	return os.good();
}

void VertexCameraTangentialDistortion::setToOriginImpl() {
	_estimate = Eigen::Vector2d::Zero();
}

void VertexCameraTangentialDistortion::oplusImpl(const double* update_) {
	Eigen::Map<const Eigen::Vector2d> update(update_);
	_estimate += update;
}

} // namespace StereoVisionApp
