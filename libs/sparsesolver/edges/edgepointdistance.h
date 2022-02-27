#ifndef STEREOVISIONAPP_EDGEPOINTDISTANCE_H
#define STEREOVISIONAPP_EDGEPOINTDISTANCE_H

#include <eigen3/Eigen/Core>
#include "g2o/core/base_binary_edge.h"
#include "g2o/types/sba/types_sba.h"


namespace StereoVisionApp {

/*!
 * \brief The EdgePointDistance class is a edge representing the measure of a distance between two points in the scene.
 */
class EdgePointDistance : public g2o::BaseBinaryEdge<1, double, g2o::VertexSBAPointXYZ, g2o::VertexSBAPointXYZ>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgePointDistance();

	bool read(std::istream& is) override;
	bool write(std::ostream& os) const override;

	void computeError() override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_EDGEPOINTDISTANCE_H
