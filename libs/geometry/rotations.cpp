#include "rotations.h"

namespace StereoVisionApp {


ShapePreservingTransform::ShapePreservingTransform(Eigen::Vector3f r, Eigen::Vector3f t, float s) :
	t(t), r(r), s(s)
{

}

ShapePreservingTransform::ShapePreservingTransform() :
	t(Eigen::Vector3f::Zero()), r(Eigen::Vector3f::Zero()), s(1.)
{

}

Eigen::Vector3f ShapePreservingTransform::operator*(Eigen::Vector3f const& pt) const {
	return s*rodriguezFormula(r)*pt + t;
}
Eigen::Matrix3Xf ShapePreservingTransform::operator*(Eigen::Matrix3Xf const& pts) const {
	return applyOnto(pts.array()).matrix();
}
Eigen::Array3Xf ShapePreservingTransform::operator*(Eigen::Array3Xf const& pts) const {
	return applyOnto(pts);
}

AffineTransform ShapePreservingTransform::toAffineTransform() const {
	return AffineTransform(s*rodriguezFormula(r), t);
}

Eigen::Array3Xf ShapePreservingTransform::applyOnto(Eigen::Array3Xf const& pts) const {

	Eigen::Array3Xf transformedPts;
	transformedPts.resize(3, pts.cols());

	for (int i = 0; i < transformedPts.cols(); i++) {
		transformedPts.col(i) = s*rodriguezFormula(r)*(pts.col(i).matrix()) + t;
	}

	return transformedPts;
}

Eigen::Matrix3f rodriguezFormula(Eigen::Vector3f const& r)
{
	float theta = r.norm();
	Eigen::Matrix3f m = skew(r);

	Eigen::Matrix3f R;

	if (theta > 1e-6) {
		R = Eigen::Matrix3f::Identity() + sin(theta)/theta*m + (1 - cos(theta))/(theta*theta)*m*m;
	} else {
		R = Eigen::Matrix3f::Identity() + m + 0.5*m*m;
	}

	return R;
}

Eigen::Vector3f inverseRodriguezFormula(Eigen::Matrix3f const& R) {


	float d =  0.5*(R(0,0) + R(1,1) + R(2,2) - 1);
	Eigen::Vector3f omega;

	Eigen::Vector3f dR = unskew(R - R.transpose());

	float nDr = dR.norm();

	if (d>0.999)
	{

	  omega=0.5*dR;
	}
	else if (nDr < 1e-3) {
		float theta = acos(d);
		Eigen::Vector3f d = R.diagonal();
		omega = theta*(d - Eigen::Vector3f::Ones()*d.minCoeff())/(1 - d.minCoeff());
	}
	else
	{
	  float theta = acos(d);
	  omega = theta/(2*sqrt(1-d*d))*dR;
	}

	return omega;
}

Eigen::Matrix3f diffRodriguezLieAlgebra(Eigen::Vector3f const& r)
{
	float theta = r.norm();
	Eigen::Matrix3f m = skew(r);

	Eigen::Matrix3f dR;

	float a;
	float b;
	float c;

	if (theta > 1e-6) {
		a = sin(theta)/theta;
		b = (1 - cos(theta))/(theta*theta);
		c = (1 - a)/(theta*theta);
	} else {
		a = 1;
		b = 1./2.;
		c = 1./6.;
	}

	dR = a*Eigen::Matrix3f::Identity() + b*m + c*m*m.transpose();

	return dR;

}
Eigen::Matrix3f diffRodriguez(Eigen::Vector3f const& r, Axis direction) {
	return rodriguezFormula(r)*diffRodriguezLieAlgebra(r)*skew(pathFromDiff(direction));
}

} // namespace StereoVisionApp
