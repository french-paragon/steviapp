#include "vertexcameraradialdistorsion.h"

#include <iostream>

VertexCameraRadialDistorsion::VertexCameraRadialDistorsion()
{

}

bool VertexCameraRadialDistorsion::read(std::istream& is) {

	is >> _estimate(0);
	is >> _estimate(1);
	is >> _estimate(2);

	return true;
}

bool VertexCameraRadialDistorsion::write(std::ostream& os) const {
	os << _estimate(0) << " " << _estimate(1) << " " << _estimate(2) << " ";
	return os.good();
}

void VertexCameraRadialDistorsion::setToOriginImpl() {
	_estimate = Eigen::Vector3d::Zero();
}

void VertexCameraRadialDistorsion::oplusImpl(const double* update_) {
	Eigen::Map<const Eigen::Vector3d> update(update_);
	_estimate += update;
}
