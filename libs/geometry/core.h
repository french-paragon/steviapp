#ifndef CORE_H
#define CORE_H

#include <eigen3/Eigen/Core>

namespace StereoVisionApp {

Eigen::Matrix3f skew(Eigen::Vector3f const& v);
Eigen::Vector3f unskew(Eigen::Matrix3f const& m);

enum class Axis : char {
	X,
	Y,
	Z
};


enum class IterativeTermination : char {
	Error,
	Converged,
	MaxStepReached
};

Eigen::Vector3f pathFromDiff(Axis dir);

class AffineTransform
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	AffineTransform(Eigen::Matrix3f R, Eigen::Vector3f t);
	AffineTransform();

	Eigen::Vector3f operator*(Eigen::Vector3f const& pt) const;
	Eigen::Matrix3Xf operator*(Eigen::Matrix3Xf const& pts) const;
	Eigen::Array3Xf operator*(Eigen::Array3Xf const& pts) const;

	Eigen::Vector3f t;
	Eigen::Matrix3f R;

protected:

	Eigen::Array3Xf applyOnto(Eigen::Array3Xf const& pts) const;
};

} // namespace StereoVisionApp

#endif // CORE_H
