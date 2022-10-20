#include "edgecamerase3leverarm.h"

#include <QDebug>

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

	VertexXiType::EstimateType cam1tocam2 = cam2toWorld.inverseSE3().applySE3(cam1toWorld);

	VertexXiType::EstimateType q = cam1tocam2.applySE3(_measurement);
	_error = q.log(); //implementation using Rodriguez formula.
}

EdgeCameraParametrizedSE3LeverArm::EdgeCameraParametrizedSE3LeverArm() {
	resize(3);

	_measurement.setR(Eigen::Matrix3d::Identity());
	_measurement.setT(Eigen::Vector3d::Zero());
}

bool EdgeCameraParametrizedSE3LeverArm::read(std::istream& is) {
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
bool EdgeCameraParametrizedSE3LeverArm::write(std::ostream& os) const {
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

void EdgeCameraParametrizedSE3LeverArm::computeError() {

	const VertexCameraPose* v1 = static_cast<const VertexCameraPose*>(_vertices[0]);
	VertexCameraPose::EstimateType cam1toWorld = v1->estimate();
	const VertexCameraPose* v2 = static_cast<const VertexCameraPose*>(_vertices[1]);
	VertexCameraPose::EstimateType cam2toWorld = v2->estimate();

	const VertexCameraPose* v3 = static_cast<const VertexCameraPose*>(_vertices[2]);
	VertexCameraPose::EstimateType relativePose = v3->estimate();

	VertexCameraPose::EstimateType cam1tocam2 = cam2toWorld.inverseSE3().applySE3(cam1toWorld);

	VertexCameraPose::EstimateType q = cam1tocam2.applySE3(relativePose).applySE3(_measurement);
	_error = q.log(); //implementation using Rodriguez formula.

}


} // namespace StereoVisionApp
