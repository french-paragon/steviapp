#ifndef STEREOVISIONAPP_EDGEPOINTSANGLE_H
#define STEREOVISIONAPP_EDGEPOINTSANGLE_H


#include <eigen3/Eigen/Core>
#include "g2o/core/base_multi_edge.h"

namespace StereoVisionApp {

class EdgePointsAngle : public  g2o::BaseMultiEdge<1, double>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgePointsAngle();

	bool read(std::istream& is) override;
	bool write(std::ostream& os) const override;

	void computeError() override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_EDGEPOINTSANGLE_H
