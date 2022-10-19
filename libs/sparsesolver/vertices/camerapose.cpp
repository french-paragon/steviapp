#include "camerapose.h"
#include <eigen3/Eigen/src/Geometry/AngleAxis.h>
#include "g2o/types/slam3d/se3_ops.h"
#include "geometry/rotations.h"

namespace StereoVisionApp {

CameraPose::CameraPose(Eigen::Matrix3d const& R,
					   Eigen::Vector3d const& t) :
	_r(R),
	_t(t)
{

}

CameraPose::CameraPose(Eigen::Quaterniond const& q,
					   Eigen::Vector3d const& t):
	CameraPose(q.toRotationMatrix(), t)
{

}

CameraPose::CameraPose(const Eigen::AngleAxisd &a,
					   Eigen::Vector3d const& t) :
	CameraPose(a.toRotationMatrix(), t)
{

}

CameraPose::CameraPose(CameraPose const& other) :
	CameraPose(other.r(), other.t())
{

}

CameraPose& CameraPose::operator= (CameraPose const& other) {
	_r = other.r();
	_t = other.t();
	return *this;
}

CameraPose& CameraPose::operator*= (CameraPose const& other) {
	_r *= other.r();
	_t += other.t();
	return *this;
}

CameraPose CameraPose::operator* (CameraPose const& other) {

	return CameraPose(_r*other.r(), _t + other.t());
}

CameraPose CameraPose::applySE3 (CameraPose const& other) {
	return CameraPose(_r*other.r(), _t + _r*other.t());
}

Eigen::Matrix3d CameraPose::r() const
{
	return _r;
}

void CameraPose::setR(const Eigen::Matrix3d &r)
{
	_r = r;
}

Eigen::Vector3d CameraPose::t() const
{
	return _t;
}

void CameraPose::setT(const Eigen::Vector3d &t)
{
	_t = t;
}

CameraPose CameraPose::inverse() const {
	return CameraPose(_r.transpose(), -_t);
}
CameraPose CameraPose::inverseSE3() const {
	return CameraPose(_r.transpose(), _r.transpose()*(-_t));
}


CameraPose::Vector6d CameraPose::log() const {

	Eigen::Vector3d omega = StereoVision::Geometry::inverseRodriguezFormulaD(_r);

	Vector6d v;
	v << omega, _t;

	return v;
}

CameraPose CameraPose::exp(Vector6d const& log) {

	Eigen::Vector3d r = log.block<3, 1>(0, 0);
	Eigen::Vector3d t = log.block<3, 1>(3, 0);

	return CameraPose(StereoVision::Geometry::rodriguezFormulaD(r), t);
}

std::istream & operator>> (std::istream & in, StereoVisionApp::CameraPose & cam) {
	Eigen::Matrix3d r;
	Eigen::Vector3d t;

	for(int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			in >> r(i, j);
		}
	}

	for (int i = 0; i < 3; i++) {
		in >> t[i];
	}

	cam.setR(r);
	cam.setT(t);

	return in;
}

std::ostream & operator<< (std::ostream & out, StereoVisionApp::CameraPose const& cam) {

	for(int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			out << cam._r(i, j);
		}
	}

	for (int i = 0; i < 3; i++) {
		out << cam._t[i] << " ";
	}

	return out;
}

} // namespace StereoVisionApp
