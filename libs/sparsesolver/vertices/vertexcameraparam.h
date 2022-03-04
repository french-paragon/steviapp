#ifndef VERTEXCAMERAPARAM_H
#define VERTEXCAMERAPARAM_H

#include "camparam.h"
#include "g2o/core/base_vertex.h"

#include "./vertexcameraradialdistortion.h"
#include "./vertexcameratangentialdistortion.h"
#include "./vertexcameraskewdistortion.h"

#include "../../datablocks/project.h"

namespace StereoVisionApp {

/*!
 * \brief The VertexCameraParam class store the focal lenght and principal point of a camera.
 */
class VertexCameraParam : public g2o::BaseVertex<3, CamParam>, public DataBlockReference
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	VertexCameraParam();

	virtual bool read(std::istream& is);

	virtual bool write(std::ostream& os) const;

	virtual void setToOriginImpl();

	virtual void oplusImpl(const double* update_);
};

/*!
 * \brief The VertexCameraFocal class store the focal lenght of a camera.
 *
 * Aditionaly the principal point can be stored as well.
 */
class VertexCameraFocal : public g2o::BaseVertex<1, double>, public DataBlockReference
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	VertexCameraFocal();

	virtual bool read(std::istream& is);

	virtual bool write(std::ostream& os) const;

	virtual void setToOriginImpl();

	virtual void oplusImpl(const double* update_);

	void setPP(Eigen::Vector2d pp);
	void clearPP();
	void setExtend(Eigen::Vector2d ext);
	void clearExtend();
	std::optional<CamParam> getCamParam() const;

protected:
	std::optional<Eigen::Vector2d> _pp;
	std::optional<Eigen::Vector2d> _extend;
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
