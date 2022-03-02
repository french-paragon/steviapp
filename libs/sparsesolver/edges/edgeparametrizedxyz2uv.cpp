#include "edgeparametrizedxyz2uv.h"

#include "sparsesolver/vertices/vertexcamerapose.h"
#include "sparsesolver/vertices/vertexcameraparam.h"
#include "sparsesolver/vertices/vertexcameraradialdistortion.h"
#include "sparsesolver/vertices/vertexcameratangentialdistortion.h"
#include "sparsesolver/vertices/vertexcameraskewdistortion.h"

#include "g2o/types/sba/types_sba.h"
#include "g2o/types/slam3d/se3_ops.h"

#include "geometry/alignement.h"
#include "geometry/lensdistortion.h"

#include <cmath>

namespace StereoVisionApp {

Eigen::Vector2d project(const CameraPose & pose,
						const Eigen::Vector3d & point,
						const CamParam & cam,
						const std::optional<Eigen::Vector3d> & r_dist,
						const std::optional<Eigen::Vector2d> & t_dist,
						const std::optional<Eigen::Vector2d> & s_dist) {

	Eigen::Vector3d Pbar = pose.r().transpose()*(point - pose.t());
	if (Pbar[2] < 0.0) {
		return {NAN, NAN};
	}

	Eigen::Vector2d proj = StereoVision::Geometry::projectPointsD(Pbar);

	Eigen::Vector2d dRadial = Eigen::Vector2d::Zero();
	Eigen::Vector2d dTangential = Eigen::Vector2d::Zero();

	if (r_dist.has_value()) {
		dRadial = StereoVision::Geometry::radialDistortionD(proj, r_dist.value());
	}

	if (t_dist.has_value()) {
		dTangential = StereoVision::Geometry::tangentialDistortionD(proj, t_dist.value());
	}

	proj += dRadial + dTangential;

	if (s_dist.has_value()) {
		return StereoVision::Geometry::skewDistortionD(proj, s_dist.value(), cam.f(), cam.pp());
	}

	return cam.f()*proj + cam.pp();

}

EdgeParametrizedXYZ2UV::EdgeParametrizedXYZ2UV()
{
	resize(6);
}

bool EdgeParametrizedXYZ2UV::read  (std::istream& is) {

	Eigen::Vector2d p;
	is >> p[0] >> p[1];
	setMeasurement(p);
	for (int i = 0; i < 2; ++i)
		for (int j = i; j < 2; ++j) {
			is >> information()(i, j);
			if (i != j)
				information()(j, i) = information()(i, j);
		}
	return true;

}

bool EdgeParametrizedXYZ2UV::write (std::ostream& os) const {

	Eigen::Vector2d p = measurement();
	os << p[0] << " " << p[1] << " ";
	for (int i = 0; i < 2; ++i)
		for (int j = i; j < 2; ++j)
			os << " " << information()(i, j);
	return os.good();


}
void EdgeParametrizedXYZ2UV::computeError () {

	const VertexCameraPose * pose = static_cast<const VertexCameraPose*>(_vertices[0]);
	const g2o::VertexSBAPointXYZ* point = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[1]);
	const VertexCameraParam * cam = static_cast<const VertexCameraParam*>(_vertices[2]);
	const VertexCameraRadialDistortion * r_dist = (_vertices.size() > 3) ? static_cast<const VertexCameraRadialDistortion*>(_vertices[3]) : nullptr;
	const VertexCameraTangentialDistortion * t_dist = (_vertices.size() > 4) ?  static_cast<const VertexCameraTangentialDistortion*>(_vertices[4]) : nullptr;
	const VertexCameraSkewDistortion * s_dist = (_vertices.size() > 5) ? static_cast<const VertexCameraSkewDistortion*>(_vertices[5]) : nullptr;

	Eigen::Vector2d obs = _measurement;
	Eigen::Vector2d proj = project(pose->estimate(),
								   point->estimate(),
								   cam->estimate(),
								   r_dist->estimate(),
								   t_dist->estimate(),
								   s_dist->estimate());

	_error = proj - obs;

}

void EdgeParametrizedXYZ2UV::linearizeOplus () {

	#ifdef USE_ANALYTICAL_JACOBIAN

		const VertexCameraPose * pose = static_cast<const VertexCameraPose*>(_vertices[0]);
		const g2o::VertexSBAPointXYZ* point = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[1]);
		const VertexCameraParam * cam = static_cast<const VertexCameraParam*>(_vertices[2]);
		const VertexCameraRadialDistorsion * r_dist = (_vertices.size() > 3) ? static_cast<const VertexCameraRadialDistorsion*>(_vertices[3]) : nullptr;
		const VertexCameraTangentialDistorsion * t_dist = (_vertices.size() > 4) ?  static_cast<const VertexCameraTangentialDistorsion*>(_vertices[4]) : nullptr;
		const VertexCameraSkewDistortion * s_dist = (_vertices.size() > 5) ? static_cast<const VertexCameraSkewDistortion*>(_vertices[5]) : nullptr;

		//jacobian with respect the the pose.
		Eigen::Vector3d tmp = -pose->estimate().t() + point->estimate();
		Eigen::Vector3d Pbar = pose->estimate().r().transpose()*(tmp); //position of origin in camera frame.

		const double& x = Pbar[0];
		const double& y = Pbar[1];
		const double& z = Pbar[2];

		const double& f_l = cam->estimate().f();

		Eigen::Matrix<double,2,3,Eigen::ColMajor> Pi;
		Pi(0,0) = f_l;
		Pi(0,1) = 0;
		Pi(0,2) = -x/z*f_l;

		Pi(1,0) = 0;
		Pi(1,1) = f_l;
		Pi(1,2) = -y/z*f_l;

		Pi /= z;

		_jacobianOplus[0] = Eigen::Matrix<double,2,6,Eigen::ColMajor>::Zero();

		//we consider the first three values to be the rotation.
		_jacobianOplus[0].block<2, 3>(0, 0) = Pi * pose->estimate().r().transpose()*g2o::skew(tmp);
		//the three others to be the translation.
		_jacobianOplus[0].block<2, 3>(0, 3) = -Pi*pose->estimate().r().transpose();

		//jacobian with respect to the point position.
		_jacobianOplus[1] = Pi*pose->estimate().r().transpose();

		//jacobian with respect to the camera
		_jacobianOplus[2] = Eigen::Matrix<double,2,3,Eigen::ColMajor>::Zero();

		_jacobianOplus[2].block<2, 2>(0, 0) = -Eigen::Matrix<double,2,2,Eigen::ColMajor>::Identity();


		Eigen::Vector2d proj = cam->estimate().partialProjectZ(Pbar); //formule pix4

		_jacobianOplus[2].block<2, 1>(0, 2) = proj; //df

		//jacobian with respect to the radial distorsion, tangetial distortion and skew distortion.

		double r2 = proj(0)*proj(0) + proj(1)*proj(1);

		if (r_dist != nullptr) {

			Eigen::Vector3d rv;
			rv << r2, r2*r2, r2*r2*r2;

			_jacobianOplus[3] = proj * rv.transpose();

			_jacobianOplus[3] *= cam->estimate().f();

		}

		if (t_dist != nullptr) {
			int id = (r_dist != nullptr) ? 4 : 3;

			_jacobianOplus[id] = Eigen::Matrix<double,2,2,Eigen::ColMajor>::Zero();

			_jacobianOplus[id](0, 0) =  2*proj[0]*proj[1];
			_jacobianOplus[id](0, 1) = r2 + 2*proj[0]*proj[0];
			_jacobianOplus[id](1, 0) = r2 + 2*proj[1]*proj[1];
			_jacobianOplus[id](1, 1) = 2*proj[0]*proj[1];

			_jacobianOplus[id] *= cam->estimate().f();
		}


		if (s_dist != nullptr) {
			int id = (r_dist != nullptr) ? 4 : 3;
			if (t_dist != nullptr) { id++; }

			_jacobianOplus[id] = Eigen::Matrix<double,2,2,Eigen::ColMajor>::Zero();

			_jacobianOplus[id](0, 0) =  proj[0];
			_jacobianOplus[id](0, 1) = proj[1];
		}

	#else
		BaseMultiEdge::linearizeOplus();
	#endif

}

EdgeStereoRigXYZ2UV::EdgeStereoRigXYZ2UV() {
	resize(3);
}

bool EdgeStereoRigXYZ2UV::read  (std::istream& is) {

	Eigen::Vector2d p;
	is >> p[0] >> p[1];
	setMeasurement(p);
	for (int i = 0; i < 2; ++i)
		for (int j = i; j < 2; ++j) {
			is >> information()(i, j);
			if (i != j)
				information()(j, i) = information()(i, j);
		}
	return true;
}
bool EdgeStereoRigXYZ2UV::write (std::ostream& os) const {

	Eigen::Vector2d p = measurement();
	os << p[0] << " " << p[1] << " ";
	for (int i = 0; i < 2; ++i)
		for (int j = i; j < 2; ++j)
			os << " " << information()(i, j);
	return os.good();

}
void EdgeStereoRigXYZ2UV::computeError () {

	const VertexCameraPose * pose = static_cast<const VertexCameraPose*>(_vertices[0]);
	const g2o::VertexSBAPointXYZ* point = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[1]);
	const VertexCameraFocal * cam = static_cast<const VertexCameraFocal*>(_vertices[2]);

	Eigen::Vector2d obs = _measurement;

	CamParam p;

	if (cam->getCamParam().has_value()) {
		p = cam->getCamParam().value();
	} else {
		p.setF(cam->estimate());
	}

	Eigen::Vector2d proj = project(pose->estimate(), point->estimate(), p, std::nullopt, std::nullopt, std::nullopt);

	_error = proj - obs;
}

} // namespace StereoVisionApp
