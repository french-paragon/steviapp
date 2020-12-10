#include "alignement.h"

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <geometricexception.h>

namespace StereoVisionApp {

Eigen::Array2Xf projectPoints(Eigen::Array3Xf const& pts) {

	Eigen::Array2Xf proj;
	proj.resize(2, pts.cols());

	proj.row(0) = pts.row(0)/pts.row(2);
	proj.row(1) = pts.row(1)/pts.row(2);

	return proj;

}
Eigen::Array2Xf projectPoints(Eigen::Array3Xf const& pts, AffineTransform const& T) {
	Eigen::Array3Xf transformedPts = T*pts;
	return projectPoints(transformedPts);
}
Eigen::Array2Xf projectPoints(Eigen::Array3Xf const& pts, Eigen::Matrix3f const& R, Eigen::Vector3f const& t) {
	Eigen::Array3Xf transformedPts = AffineTransform(R,t)*pts;
	return projectPoints(transformedPts);
}

Eigen::Array3Xf reprojectPoints(AffineTransform const& T,
								Eigen::Array2Xf const& pt_cam_1,
								Eigen::Array2Xf const& pt_cam_2) {
	return reprojectPoints(T.R, T.t, pt_cam_1, pt_cam_2);
}

Eigen::Array3Xf reprojectPoints(Eigen::Matrix3f const& R,
								 Eigen::Vector3f const& t,
								 Eigen::Array2Xf const& pt_cam_1,
								 Eigen::Array2Xf const& pt_cam_2) {

	int nPts = pt_cam_1.cols();

	if (nPts < 8 or pt_cam_1.cols() != pt_cam_2.cols()) {
		throw GeometricException("Points arrays of different dimensions provided");
	}

	Eigen::Array3Xf reproj;

	reproj.resize(3,nPts);
	reproj.topRows(2) = pt_cam_1;
	reproj.bottomRows(1).setOnes();

	Eigen::ArrayXf x3_v1 = (pt_cam_2.row(0)*t.z() - t.x()) /
			(
				pt_cam_2.row(0)*(R(2,0)*pt_cam_1.row(0) + R(2,1)*pt_cam_1.row(1) + R(2,2)) -
				(R(0,0)*pt_cam_1.row(0) + R(0,1)*pt_cam_1.row(1) + R(0,2))
			);

	Eigen::ArrayXf x3_v2 = (pt_cam_2.row(1)*t.z() - t.y()) /
			(
				pt_cam_2.row(1)*(R(2,0)*pt_cam_1.row(0) + R(2,1)*pt_cam_1.row(1) + R(2,2)) -
				(R(1,0)*pt_cam_1.row(0) + R(1,1)*pt_cam_1.row(1) + R(1,2))
			);

	Eigen::ArrayXf x3 = (x3_v1 + x3_v2)/2.0;

	reproj.row(0) *= x3;
	reproj.row(1) *= x3;
	reproj.row(2) *= x3;

	return -reproj;

}

Eigen::Matrix3f estimateEssentialMatrix(const Eigen::Array2Xf &pt_cam_1, const Eigen::Array2Xf &pt_cam_2) {

	typedef Eigen::Matrix<float, 9, Eigen::Dynamic> MatrixFtype;

	int nPts = pt_cam_1.cols();

	if (nPts < 8 or pt_cam_1.cols() != pt_cam_2.cols()) {
		throw GeometricException("Points arrays of different dimensions provided");
	}

	MatrixFtype F;
	F.resize(9, nPts);

	F.row(0) = pt_cam_2.row(0)*pt_cam_1.row(0);
	F.row(1) = pt_cam_2.row(0)*pt_cam_1.row(1);
	F.row(2) = pt_cam_2.row(0);
	F.row(3) = pt_cam_2.row(1)*pt_cam_1.row(0);
	F.row(4) = pt_cam_2.row(1)*pt_cam_1.row(1);
	F.row(5) = pt_cam_2.row(1);
	F.row(6) = pt_cam_1.row(0);
	F.row(7) = pt_cam_1.row(1);
	F.row(8).setOnes();

	auto svd = F.jacobiSvd(Eigen::ComputeFullU);
	auto e = svd.matrixU().col(8);
	Eigen::Matrix3f E;
	E << e[0], e[1], e[2],
		 e[3], e[4], e[5],
		 e[6], e[7], e[8];

	return E;

}

std::pair<AffineTransform, AffineTransform> essentialMatrix2Transforms(Eigen::Matrix3f const& E) {

	auto svd = E.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

	Eigen::Matrix3f U = svd.matrixU();
	Eigen::Matrix3f V = svd.matrixV();

	//ensure both U and V are rotation matrices.
	if (U.determinant() < 0) {
		U = -U;
	}

	if (V.determinant() < 0) {
		V = -V;
	}

	Eigen::Matrix3f W = Eigen::Matrix3f::Zero();
	W(1,0) = -1;
	W(0,1) = 1;
	W(2,2) = 1;

	AffineTransform T1;
	AffineTransform T2;

	T1.R = U*W*V.transpose();
	T2.R = U*W.transpose()*V.transpose();

	W(2,2) = 0;

	T1.t = unskew(U*W*U.transpose());
	T2.t = -T1.t;

	return {T1, T2};
}


AffineTransform essentialMatrix2Transform(Eigen::Matrix3f const& E,
										  Eigen::Array2Xf const& pt_cam_1,
										  Eigen::Array2Xf const& pt_cam_2) {
	auto candidates = essentialMatrix2Transforms(E);
	return selectTransform(candidates.first, candidates.second, pt_cam_1, pt_cam_2);
}

AffineTransform selectTransform(AffineTransform const& T1,
								AffineTransform const& T2,
								const Eigen::Array2Xf &pt_cam_1,
								const Eigen::Array2Xf &pt_cam_2) {

	int nPts = pt_cam_1.cols();

	AffineTransform Rs[4];

	int count_good = 0;

	for (auto const& R_cand : {T1.R, T2.R}) {

		for (auto const& t_cand : {T1.t, T2.t}) {

			Eigen::Array3Xf reproj = reprojectPoints(R_cand, t_cand, pt_cam_1, pt_cam_2);

			if ((reproj.bottomRows(1) < 0.).all()) {
				reproj = reprojectPoints(R_cand.transpose(), -R_cand.transpose()*t_cand, pt_cam_2, pt_cam_1);

				if ((reproj.bottomRows(1) < 0.).all()) {

					Rs[count_good].R = R_cand;
					Rs[count_good].t = t_cand;

					count_good++;
				}
			}

		}
	}

	if (count_good == 0) {
		throw GeometricException("No valid transforms has been found");
	}

	if (count_good > 1) {

		int selected = -1;
		float s_error = std::numeric_limits<float>::infinity();

		for (int i = 0; i < count_good; i++) {

			Eigen::Array3Xf reproj = reprojectPoints(Rs[i], pt_cam_1, pt_cam_2);
			Eigen::Array2Xf onOtherIm = projectPoints(reproj, Rs[i]);

			float c_error = (onOtherIm - pt_cam_2).matrix().norm()/nPts;

			reproj = reprojectPoints(Rs[i].R.transpose(), -Rs[i].R.transpose()*Rs[i].t, pt_cam_2, pt_cam_1);
			onOtherIm = projectPoints(reproj, Rs[i].R.transpose(), -Rs[i].R.transpose()*Rs[i].t);

			c_error += (onOtherIm - pt_cam_1).matrix().norm()/nPts;

			if (c_error < s_error) {
				selected = i;
				s_error = c_error;
			}
		}

		if (selected < 0) {
			throw GeometricException("No valid transforms has been found");
		}

		return Rs[selected];

	}

	return Rs[0];
}



AffineTransform findTransform(Eigen::Array2Xf const& pt_cam_1,
							  Eigen::Array2Xf const& pt_cam_2) {

	Eigen::Matrix3f E = estimateEssentialMatrix(pt_cam_1, pt_cam_2);
	return essentialMatrix2Transform(E, pt_cam_1, pt_cam_2);
}

} // namespace StereoVisionApp
