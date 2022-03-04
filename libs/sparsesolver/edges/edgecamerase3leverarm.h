#ifndef STEREOVISIONAPP_EDGECAMERASE3DISTANCE_H
#define STEREOVISIONAPP_EDGECAMERASE3DISTANCE_H

#include <eigen3/Eigen/Core>
#include "g2o/core/base_binary_edge.h"
#include "../vertices/vertexcamerapose.h"


namespace StereoVisionApp {

/*!
 * \brief The EdgePointDistance class is a edge representing the measure of a distance between two points in the scene.
 */
class EdgeCameraSE3LeverArm : public g2o::BaseBinaryEdge<6, CameraPose, VertexCameraPose, VertexCameraPose>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeCameraSE3LeverArm();

	bool read(std::istream& is) override;
	bool write(std::ostream& os) const override;

	void computeError() override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_EDGECAMERASE3DISTANCE_H
