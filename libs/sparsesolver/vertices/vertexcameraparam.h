#ifndef VERTEXCAMERAPARAM_H
#define VERTEXCAMERAPARAM_H

#include "camparam.h"
#include "g2o/core/base_vertex.h"

#include "./vertexcameraradialdistortion.h"
#include "./vertexcameratangentialdistortion.h"
#include "./vertexcameraskewdistortion.h"

namespace StereoVisionApp {

/*!
 * \brief The VertexCameraParam class store the focal lenght and principal point of a camera.
 */
class VertexCameraParam : public g2o::BaseVertex<3, CamParam>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	VertexCameraParam();

	virtual bool read(std::istream& is);

	virtual bool write(std::ostream& os) const;

	virtual void setToOriginImpl();

	virtual void oplusImpl(const double* update_);
};

struct CameraInnerVertexCollection {
	static const int VerticesPerCam;
	VertexCameraParam* param;
	VertexCameraRadialDistortion* radialDist;
	VertexCameraTangentialDistortion* tangeantialDist;
	VertexCameraSkewDistortion* skewDist;
};

} // namespace StereoVisionApp

#endif // VERTEXCAMERAPARAM_H
