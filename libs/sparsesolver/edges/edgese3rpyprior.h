#ifndef EDGESE3RPYPRIOR_H
#define EDGESE3RPYPRIOR_H

#include <eigen3/Eigen/Core>
#include "g2o/core/base_unary_edge.h"

#include "sparsesolver/vertices/vertexcamerapose.h"

namespace StereoVisionApp {

/*!
 * \brief The EdgeSE3rpyPrior class represent an apriori measurement of a camera orientation.
 */
class EdgeSE3rpyPrior : public g2o::BaseUnaryEdge<3, Eigen::Matrix3d, VertexCameraPose>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeSE3rpyPrior();

	virtual bool read(std::istream& is);
	virtual bool write(std::ostream& os) const;

	virtual void computeError();
};

} // namespace StereoVisionApp

#endif // EDGESE3RPYPRIOR_H
