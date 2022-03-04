#ifndef STEREOVISIONAPP_EDGECAMERALANDMARKDIRECTIONALALIGNEMENT_H
#define STEREOVISIONAPP_EDGECAMERALANDMARKDIRECTIONALALIGNEMENT_H


#include <eigen3/Eigen/Core>
#include "g2o/core/base_binary_edge.h"

#include "../vertices/vertexcamerapose.h"
#include "../vertices/vertexlandmarkpos.h"

namespace StereoVisionApp {

class MinCamDistanceParam : public g2o::Parameter
{
  public:
	MinCamDistanceParam();
	MinCamDistanceParam(double pMinDist);

	double minCameraDistance () const;

	virtual bool read(std::istream& is);

	virtual bool write(std::ostream& os) const;

	double minDist;
};

class EdgeCameraLandmarkDirectionalAlignement : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexCameraPose, VertexLandmarkPos>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeCameraLandmarkDirectionalAlignement();

	bool read(std::istream& is) override;
	bool write(std::ostream& os) const override;

	void setMeasurementFromNormalizedCoordinates(Eigen::Vector2d const& normalized);

	void computeError() override;

	MinCamDistanceParam* _minCamDistParam;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_EDGECAMERALANDMARKDIRECTIONALALIGNEMENT_H
