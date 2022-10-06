#ifndef EDGEPARAMETRIZEDXYZ2UV_H
#define EDGEPARAMETRIZEDXYZ2UV_H

#include <eigen3/Eigen/Core>
#include "g2o/core/base_multi_edge.h"

#include "vertices/camerapose.h"
#include "vertices/camparam.h"

namespace g2o {
	class VertexPointXYZ;
}

namespace StereoVisionApp {

class VertexCameraPose;
class VertexCameraParam;
class VertexCameraRadialDistortion;
class VertexCameraTangentialDistortion;
class VertexCameraSkewDistortion;

/*!
 * \brief project fumnction represent the camera model for general point projection from reference to pixel coordinate.
 * \param pose the pose of the camera in the reference system
 * \param point the point coordinate in the reference system
 * \param cam the intrisic parameters of the camera
 * \param r_dist the radial distortion coefficients (optional)
 * \param t_dist the tengential distortion coefficients (optional)
 * \param s_dist the skewness distortion coefficients (optional)
 * \return
 */
Eigen::Vector2d project(const CameraPose & pose,
						const Eigen::Vector3d & point,
						const CamParam & cam,
						const std::optional<Eigen::Vector3d> &r_dist = std::nullopt,
						const std::optional<Eigen::Vector2d> &t_dist = std::nullopt,
						const std::optional<Eigen::Vector2d> &s_dist = std::nullopt);

/*!
 * \brief The EdgeParametrizedXYZ2UV class represent a measurement from a cam to a tie points with accounting for camera parameters (principal points and focal length) and lens parameters (both radial and tangential distortion).
 */
class EdgeParametrizedXYZ2UV : public  g2o::BaseMultiEdge<2, Eigen::Vector2d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeParametrizedXYZ2UV();

	virtual bool read  (std::istream& is);
	virtual bool write (std::ostream& os) const;
	virtual void computeError ();
	virtual void linearizeOplus ();

};

/*!
 * \brief The EdgeStereoRigXYZ2UV class represent a measurement from a cam to a tie points in a stereo rig setup.
 *
 * The class does some simplifications to be used to pre-caliber a simple stereo rig (two camera oriented in the same direction),
 * like not using distiortion and using a vertex for intrisict parameters where only the focal length is learned
 */
class EdgeStereoRigXYZ2UV : public  g2o::BaseMultiEdge<2, Eigen::Vector2d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeStereoRigXYZ2UV();

	virtual bool read  (std::istream& is);
	virtual bool write (std::ostream& os) const;
	virtual void computeError ();

};

} // namespace StereoVisionApp

#endif // EDGEPARAMETRIZEDXYZ2UV_H
