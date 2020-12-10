#include "rotations.h"

namespace StereoVisionApp {

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
