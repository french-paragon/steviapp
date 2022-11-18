#ifndef STEREOVISIONAPP_EDGELOCALPOINTCOORDINATES_H
#define STEREOVISIONAPP_EDGELOCALPOINTCOORDINATES_H

#include <eigen3/Eigen/Core>
#include "g2o/core/base_binary_edge.h"
#include "../vertices/vertexrigidbodypose.h"
#include "./vertices/vertexlandmarkpos.h"


namespace StereoVisionApp {

/*!
 * \brief The EdgeLocalPointCoordinates class is a edge representing the measure of a point position in a local coordinate systems.
 */
class EdgeLocalPointCoordinates : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexRigidBodyPose, VertexLandmarkPos>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeLocalPointCoordinates();

	bool read(std::istream& is) override;
	bool write(std::ostream& os) const override;

	void computeError() override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_EDGELOCALPOINTCOORDINATES_H
