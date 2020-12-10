#ifndef POINTCLOUDALIGNMENT_H
#define POINTCLOUDALIGNMENT_H

#include <eigen3/Eigen/Core>

#include <vector>


namespace StereoVisionApp {

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

	Eigen::Vector3f t;
	Eigen::Matrix3f R;
};

Eigen::Matrix3f skew(Eigen::Vector3f const& v);

Eigen::Matrix3f rodriguezFormula(Eigen::Vector3f const& r);

Eigen::Matrix3f diffRodriguezLieAlgebra(Eigen::Vector3f const& r);
Eigen::Matrix3f diffRodriguez(Eigen::Vector3f const& r, Axis direction);

AffineTransform estimateShapePreservingMap(Eigen::VectorXf const& obs,
										   Eigen::Matrix3Xf const& pts,
										   std::vector<int> const& idxs,
										   std::vector<Axis> const& coordinate,
										   IterativeTermination * status,
										   int n_steps = 250,
										   float incrLimit = 1e-8,
										   float damping = 5e-1,
										   float dampingScale = 1e-1,
										   float initial_s = 0.,
										   Eigen::Vector3f const& initial_r = Eigen::Vector3f(0,0,0),
										   Eigen::Vector3f const& initial_t = Eigen::Vector3f(0,0,0));

}; //namespace StereoVisionApp

#endif // POINTCLOUDALIGNMENT_H
