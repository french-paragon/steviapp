#ifndef EDGESE3FULLPRIOR_H
#define EDGESE3FULLPRIOR_H

#include "g2o/core/base_unary_edge.h"

#include "sparsesolver/vertices/vertexcamerapose.h"
#include "sparsesolver/vertices/camerapose.h"

namespace StereoVisionApp {

/*!
 * \brief The EdgeSE3FullPrior class represent an a priori measurement of a camera both position and orientation-
 */
class EdgeSE3FullPrior : public g2o::BaseUnaryEdge<6, CameraPose, VertexCameraPose>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeSE3FullPrior();

	virtual bool read(std::istream& is);
	virtual bool write(std::ostream& os) const;

	virtual void computeError();
};

} // namespace StereoVisionApp

#endif // EDGESE3FULLPRIOR_H
