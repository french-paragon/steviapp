#include "pointcloudalignment.h"

#include "rotations.h"

#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/QR>
#include <QDebug>

namespace StereoVisionApp {


AffineTransform estimateShapePreservingMap(Eigen::VectorXf const& obs,
										   Eigen::Matrix3Xf const& pts,
										   std::vector<int> const& idxs,
										   std::vector<Axis> const& coordinate,
										   IterativeTermination *status,
										   int n_steps,
										   float incrLimit,
										   float damping,
										   float dampingScale,
										   float initial_s,
										   Eigen::Vector3f const& initial_r,
										   Eigen::Vector3f const& initial_t) {

	typedef Eigen::Matrix<float, 7, 1, Eigen::ColMajor> ParamVector;
	typedef Eigen::Matrix<float, 7, 7> MatrixQxx ;

	float s = initial_s;
	Eigen::Vector3f r = initial_r;
	Eigen::Vector3f t = initial_t;

	AffineTransform transform;

	int n_obs = obs.rows();

	if (n_obs < 7) {//underdetermined
		if (status != nullptr) {
			*status = IterativeTermination::Error;
		}
		return transform;
	}

	ParamVector x;
	x << r, t, s;

	*status = IterativeTermination::MaxStepReached;

	Eigen::Matrix<float, Eigen::Dynamic, 7, Eigen::RowMajor> A;
	A.resize(n_obs, 7);

	Eigen::VectorXf e = obs;

	for(int i = 0; i < n_steps; i++) {

		r << x[0],x[1],x[2];
		t << x[3],x[4],x[5];
		s = x[6];

		if (fabs(s) > 5) {//limit the scale growth to avoid scale explosion
			s = (s > 0) ? 3. : -3;
			x[6] = s;
		}

		Eigen::Matrix3f R = rodriguezFormula(r);
		Eigen::Matrix3f DxR = diffRodriguez(r, Axis::X);
		Eigen::Matrix3f DyR = diffRodriguez(r, Axis::Y);
		Eigen::Matrix3f DzR = diffRodriguez(r, Axis::Z);

		for (int i = 0; i < static_cast<int>(idxs.size()); i++) {

			int id_row;

			if (coordinate[i] == Axis::X) {
				id_row = 0;
			}

			if (coordinate[i] == Axis::Y) {
				id_row = 1;
			}

			if (coordinate[i] == Axis::Z) {
				id_row = 2;
			}
			Eigen::Block<Eigen::Matrix3f,1,3> Rrow = R.block<1,3>(id_row,0);

			float l0i = exp(s)*(Rrow*pts.block<3,1>(0,idxs[i]))(0,0) + t[id_row];
			e[i] = obs[i] - l0i;

			A(i, 0) = exp(s)*(DxR.block<1,3>(id_row,0)*pts.block<3,1>(0,idxs[i]))(0,0); //Param rx;
			A(i, 1) = exp(s)*(DyR.block<1,3>(id_row,0)*pts.block<3,1>(0,idxs[i]))(0,0); //Param ry;
			A(i, 2) = exp(s)*(DzR.block<1,3>(id_row,0)*pts.block<3,1>(0,idxs[i]))(0,0); //Param rz;

			A(i, 3) = (coordinate[i] == Axis::X) ? 1 : 0; //Param x;
			A(i, 4) = (coordinate[i] == Axis::Y) ? 1 : 0; //Param y;
			A(i, 5) = (coordinate[i] == Axis::Z) ? 1 : 0; //Param z;

			A(i, 6) = exp(s)*(Rrow*pts.block<3,1>(0,idxs[i]))(0,0); //Param s;
		}

		qDebug() << i << ": " << e.norm();

		MatrixQxx invQxx = A.transpose()*A;
		auto QR = Eigen::ColPivHouseholderQR<MatrixQxx>(invQxx);

		if (!QR.isInvertible()) {
			if (status != nullptr) {
				*status = IterativeTermination::Error;
			}
			return transform;
		}

		ParamVector incr = QR.solve(A.transpose()*e);
		incr[6] *= dampingScale;

		x = x + damping*incr;

		float n = incr.norm();
		if (n < incrLimit) {
			if (status != nullptr) {
				*status = IterativeTermination::Converged;
			}
			break;
		}

	}

	r << x[0],x[1],x[2];
	t << x[3],x[4],x[5];
	s = x[6];

	transform.R = exp(s)*rodriguezFormula(r);
	transform.t = t;

	return transform;

}

}; //namespace StereoVisionApp
