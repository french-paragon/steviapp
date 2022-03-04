#include "edgecamerase3leverarm.h"

namespace StereoVisionApp {

EdgeCameraSE3LeverArm::EdgeCameraSE3LeverArm()
{

}

bool EdgeCameraSE3LeverArm::read(std::istream& is) {
	Eigen::Vector3d p;
	is >> p[0] >> p[1] >> p[2];
	Eigen::AngleAxisd a;
	is >> a.axis()[0] >> a.axis()[1] >> a.axis()[2];
	Measurement q(Eigen::Quaterniond(a), p);
	setMeasurement(q);
	for (int i = 0; i < 6; ++i)
		for (int j = i; j < 6; ++j) {
			is >> information()(i, j);
			if (i != j)
				information()(j, i) = information()(i, j);
		}
	return true;
}
bool EdgeCameraSE3LeverArm::write(std::ostream& os) const {
	Measurement q = measurement();
	Eigen::Vector3d p = q.translation();
	Eigen::AngleAxisd a(q.rotation());
	os << p[0] << " " << p[1] << " " << p[2] << " ";
	os << a.axis()[0] << " " << a.axis()[1] << " " << a.axis()[1];
	for (int i = 0; i < 6; ++i)
		for (int j = i; j < 6; ++j)
			os << " " << information()(i, j);
	return os.good();
}

void EdgeCameraSE3LeverArm::computeError() {

	const VertexXiType* v1 = static_cast<const VertexXiType*>(_vertices[0]);
	VertexXiType::EstimateType cam1toWorld = v1->estimate();
	const VertexXjType* v2 = static_cast<const VertexXjType*>(_vertices[1]);
	VertexXjType::EstimateType cam2toWorld = v2->estimate();

	VertexXiType::EstimateType cam1tocam2 = cam1toWorld*cam2toWorld.inverse();

	VertexXiType::EstimateType q = cam1tocam2 * _measurement;
	_error = q.log(); //implementation using Rodriguez formula.
}

} // namespace StereoVisionApp
