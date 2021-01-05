#ifndef CAMERAPOSE_H
#define CAMERAPOSE_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <eigen3/Eigen/src/Geometry/AngleAxis.h>

namespace StereoVisionApp {

/*!
 * \brief The CameraPose class represent the position and orientation of a camera.
 *
 * It is mainly 1 SO(3) term, stored as rotation matrix and a translation term.
 * The benefit of dissociating both part is to simplify the derivation of
 * analytic gradients, since the closed form formulas for SO(3) can be used
 * instead of the expression for SE(3).
 *
 * When used in our optimisation graph, rotation is assumed to be from
 * body to mapping, as well as translation.
 */
class CameraPose
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	CameraPose(const Eigen::Matrix3d& R = Eigen::Matrix3d::Identity(),
			   const Eigen::Vector3d& t = Eigen::Vector3d::Zero() );
	CameraPose(const Eigen::Quaterniond& q,
			   const Eigen::Vector3d& t = Eigen::Vector3d::Zero() );
	CameraPose(const Eigen::AngleAxisd& a,
			   const Eigen::Vector3d& t = Eigen::Vector3d::Zero() );

	typedef Eigen::Matrix<double, 6, 1, Eigen::ColMajor> Vector6d;

	CameraPose(CameraPose const& other);
	CameraPose& operator= (CameraPose const& other);
	CameraPose& operator*= (CameraPose const& other);
	CameraPose operator* (CameraPose const& other);

	friend std::istream & operator>> (std::istream & in, CameraPose & cam);
	friend std::ostream & operator<< (std::ostream & out, CameraPose const& cam);

	Eigen::Matrix3d r() const;
	inline Eigen::Matrix3d rotation() const { return r(); }
	void setR(const Eigen::Matrix3d &r);

	Eigen::Vector3d t() const;
	inline Eigen::Vector3d translation() const { return t(); }
	void setT(const Eigen::Vector3d &t);

	CameraPose inverse() const;

	Vector6d log() const;
	static CameraPose exp(Vector6d const& log);

protected:

	//! \brief store the rotation matrix in a full dense matrix, this is the best for speed.
	Eigen::Matrix3d _r;
	//! \brief store the translation therm.
	Eigen::Vector3d _t;
};

std::istream & operator>> (std::istream & in, StereoVisionApp::CameraPose & cam);
std::ostream & operator<< (std::ostream & out, StereoVisionApp::CameraPose const& cam);

} // namespace StereoVisionApp

#endif // CAMERAPOSE_H
