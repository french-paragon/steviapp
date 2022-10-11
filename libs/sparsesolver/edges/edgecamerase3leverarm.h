#ifndef STEREOVISIONAPP_EDGECAMERASE3DISTANCE_H
#define STEREOVISIONAPP_EDGECAMERASE3DISTANCE_H

#include <eigen3/Eigen/Core>
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "../vertices/vertexcamerapose.h"


namespace StereoVisionApp {

/*!
 * \brief The EdgeCameraSE3LeverArm class is a edge representing the measure of a lever arm between two cameras in the scene.
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

/*!
 * \brief The EdgeCameraParametrizedSE3LeverArm class is a edge representing the measure of the lever arm difference between two cameras and a reference lever arm.
 *
 * The first vertex is camera 1
 * The second vertex is camera 2
 * The thirs vertex is the expected transform from caqmera 2 to camera 1
 * The measurement is the measured difference (in most application it should stay 0).
 */
class EdgeCameraParametrizedSE3LeverArm : public g2o::BaseMultiEdge<6, CameraPose>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeCameraParametrizedSE3LeverArm();

	bool read(std::istream& is) override;
	bool write(std::ostream& os) const override;

	void computeError() override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_EDGECAMERASE3DISTANCE_H
