#ifndef EDGESE3XYZPRIOR_H
#define EDGESE3XYZPRIOR_H

#include <eigen3/Eigen/Core>
#include "g2o/core/base_unary_edge.h"

#include "sparsesolver/vertices/vertexcamerapose.h"

namespace StereoVisionApp {

/*!
 * \brief The EdgeSE3xyzPrior class represent an a priori measurement of a camera position.
 */
class EdgeSE3xyzPrior : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexCameraPose>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeSE3xyzPrior();

	virtual bool read(std::istream& is);
	virtual bool write(std::ostream& os) const;

	virtual void computeError();
};

} // namespace StereoVisionApp

#endif // EDGESE3XYZPRIOR_H
