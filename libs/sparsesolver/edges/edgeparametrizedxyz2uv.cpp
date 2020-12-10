#include "edgeparametrizedxyz2uv.h"

#include "sparsesolver/vertices/vertexcamerapose.h"
#include "sparsesolver/vertices/vertexcameraparam.h"
#include "sparsesolver/vertices/vertexcameraradialdistortion.h"
#include "sparsesolver/vertices/vertexcameratangentialdistortion.h"
#include "sparsesolver/vertices/vertexcameraskewdistortion.h"

#include "g2o/types/sba/types_sba.h"
#include "g2o/types/slam3d/se3_ops.h"

#include <cmath>

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

	_error = project(pose, point, cam, r_dist, t_dist, s_dist) - obs; //DONE, check the signs are correct. it's easier to compute the derivative with this formula.


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

		//jacobian with respect the the position.

		_jacobianOplus[1] = Pi*pose->estimate().r().transpose();

		//jacobian with respect to the camera
		_jacobianOplus[2] = Eigen::Matrix<double,2,3,Eigen::ColMajor>::Zero();

		_jacobianOplus[2].block<2, 2>(0, 0) = Eigen::Matrix<double,2,2,Eigen::ColMajor>::Identity();


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

Eigen::Vector2d EdgeParametrizedXYZ2UV::project(VertexCameraPose const* pose,
												g2o::VertexSBAPointXYZ const* point,
												VertexCameraParam const* cam,
												VertexCameraRadialDistortion const* r_dist,
												VertexCameraTangentialDistortion const* t_dist,
												VertexCameraSkewDistortion const* s_dist) {

	//TODO: assume cameras are not well fixed and find a way to init positions properly.

	Eigen::Vector3d Pbar = pose->estimate().r().transpose()*(-pose->estimate().t() + point->estimate()); //position of origin in camera frame. The convention for r use one matrix multiplication more, but would be easier to work with in case we add inertial measurements.
	if (Pbar[2] > 0.0) {
		return {NAN, NAN};
	}

	Eigen::Vector2d proj = cam->estimate().partialProjectZ(Pbar); //formule pix4

	double r2 = proj(0)*proj(0) + proj(1)*proj(1);

	if (r_dist != nullptr) {
		Eigen::Vector3d vr;
		vr << r2, r2*r2, r2*r2*r2;

		proj += r_dist->estimate().dot(vr)*proj;
	}

	if (t_dist != nullptr) {
		Eigen::Vector2d td;
		td << t_dist->estimate()(1)*(r2 + 2*proj(0)*proj(0)) + 2*t_dist->estimate()(0)*proj(0)*proj(1),
				t_dist->estimate()(0)*(r2 + 2*proj(1)*proj(1)) + 2*t_dist->estimate()(1)*proj(0)*proj(1);

		proj += td;

	}

	if (s_dist != nullptr) {
		const Eigen::Vector2d & td = s_dist->estimate();
		Eigen::Vector2d r = cam->estimate().f()*proj + cam->estimate().pp();
		r[0] += td[0]*proj[0] + td[1]*proj[1];
		return r;
	}

	return cam->estimate().f()*proj + cam->estimate().pp();

}
