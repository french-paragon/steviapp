#ifndef EDGEXYZPRIOR_H
#define EDGEXYZPRIOR_H

#include <eigen3/Eigen/Core>
#include "g2o/core/base_unary_edge.h"

#include "g2o/core/base_unary_edge.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

namespace StereoVisionApp {

/*!
 * \brief The EdgeXyzPrior class allow to get an apriori
 */
class EdgeXyzPrior : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeXyzPrior();

	virtual bool read(std::istream& is);
	virtual bool write(std::ostream& os) const;

	virtual void computeError();
};

} // namespace StereoVisionApp

#endif // EDGEXYZPRIOR_H
