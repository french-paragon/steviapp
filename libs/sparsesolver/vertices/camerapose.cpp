#include "camerapose.h"
#include <eigen3/Eigen/src/Geometry/AngleAxis.h>
#include "g2o/types/slam3d/se3_ops.h"

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

std::istream & operator>> (std::istream & in,  CameraPose & cam) {
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

std::ostream & operator<< (std::ostream & out, CameraPose const& cam) {

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


CameraPose::Vector6d CameraPose::log() const {

	double d =  0.5*(_r(0,0)+_r(1,1)+_r(2,2)-1);
	Eigen::Vector3d omega;

	Eigen::Vector3d dR = g2o::deltaR(_r);

	if (d>0.99999)
	{
	  omega=0.5*dR;
	}
	else if (dR.norm() < 1e-6) {
		double theta = acos(d);
		Eigen::Vector3d d = _r.diagonal();
		omega = theta*(d - Eigen::Vector3d::Ones()*d.minCoeff())/(1 - d.minCoeff());
	}
	else
	{
	  double theta = acos(d);
	  omega = theta/(2*sqrt(1-d*d))*dR;
	}

	Vector6d v;
	v << omega, _t;

	return v;
}

CameraPose CameraPose::exp(Vector6d const& log) {

	Eigen::Vector3d r = log.block<3, 1>(0, 0);
	Eigen::Vector3d t = log.block<3, 1>(3, 0);
	Eigen::Matrix3d m = g2o::skew(r);

	double theta = r.norm();

	Eigen::Matrix3d em;
	if (theta > 1e-6) {
		em = Eigen::Matrix3d::Identity() + sin(theta)/theta*m + (1 - cos(theta))/(theta*theta)*m*m;
	} else {
		em = Eigen::Matrix3d::Identity() + m + 0.5*m*m;
	}

	return CameraPose(em, t);
}
