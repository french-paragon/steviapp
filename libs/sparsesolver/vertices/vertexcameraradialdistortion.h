#ifndef CAMERARADIALDISTORSIONVERTEX_H
#define CAMERARADIALDISTORSIONVERTEX_H

#include <Eigen/Core>
#include "g2o/core/base_vertex.h"

namespace StereoVisionApp {

/*!
 * \brief The VertexCameraRadialDistorsion class represent the radial distorsion coefficients of a lens (R1, R2, R3).
 */
class VertexCameraRadialDistortion : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	VertexCameraRadialDistortion();

	virtual bool read(std::istream& is);

	virtual bool write(std::ostream& os) const;

	virtual void setToOriginImpl();

	virtual void oplusImpl(const double* update_);
};

} // namespace StereoVisionApp

#endif // CAMERARADIALDISTORSIONVERTEX_H
