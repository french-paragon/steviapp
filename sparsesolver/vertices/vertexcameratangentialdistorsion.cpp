#include "vertexcameratangentialdistorsion.h"

VertexCameraTangentialDistorsion::VertexCameraTangentialDistorsion()
{

}

bool VertexCameraTangentialDistorsion::read(std::istream& is) {

	is >> _estimate(0);
	is >> _estimate(1);

	return true;
}

bool VertexCameraTangentialDistorsion::write(std::ostream& os) const {

	os << _estimate(0) << " " << _estimate(1) << " ";
	return os.good();
}

void VertexCameraTangentialDistorsion::setToOriginImpl() {
	_estimate = Eigen::Vector2d::Zero();
}

void VertexCameraTangentialDistorsion::oplusImpl(const double* update_) {
	Eigen::Map<const Eigen::Vector2d> update(update_);
	_estimate += update;
}
