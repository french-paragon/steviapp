#ifndef EDGEPARAMETRIZEDXYZ2UV_H
#define EDGEPARAMETRIZEDXYZ2UV_H

#include <eigen3/Eigen/Core>
#include "g2o/core/base_multi_edge.h"

class VertexCameraPose;
class VertexCameraParam;
class VertexCameraRadialDistorsion;
class VertexCameraTangentialDistorsion;

namespace g2o {
	class VertexSBAPointXYZ;
}

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

	static Eigen::Vector2d project(const VertexCameraPose *pose,
								   const g2o::VertexSBAPointXYZ *point,
								   const VertexCameraParam *cam,
								   const VertexCameraRadialDistorsion *r_dist,
								   const VertexCameraTangentialDistorsion *t_dist);

protected:


};

#endif // EDGEPARAMETRIZEDXYZ2UV_H
