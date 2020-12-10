#include "sbainitializer.h"

#include "datablocks/project.h"
#include "datablocks/image.h"
#include "datablocks/landmark.h"
#include "datablocks/camera.h"

#include <eigen3/Eigen/Core>
#include <Eigen/Geometry>

#include <QSet>
#include <QDebug>

#include <random>
#include <algorithm>

namespace StereoVisionApp {

SBAInitializer::~SBAInitializer() {

}


EightPointsSBAInitializer::EightPointsSBAInitializer(qint64 f1, qint64 f2, int triangulation_threshold) :
	_f1(f1),
	_f2(f2),
	_auto_triangulation_threshold(triangulation_threshold)
{

}

SBAInitializer::InitialSolution EightPointsSBAInitializer::computeInitialSolution(Project* p,
																				  QSet<qint64> const& s_pts,
																				  QSet<qint64> const& s_imgs) {

	InitialSolution r;
	r.cams.clear();
	r.points.clear();

	qint64 s_f1 = _f1;
	qint64 s_f2 = _f2;

	Image* f1 = nullptr;
	Image* f2 = nullptr;

	qint64 strat;

	int n_pt_selected = 7; //to select a pair images should have a least 8 points in common.
	float svd_score = std::numeric_limits<float>::infinity();
	QSet<qint64> s_intersection;

	QVector<qint64> c_img = s_imgs.values().toVector();

	std::random_device rd;
	std::default_random_engine engine(rd());
	std::uniform_real_distribution<float> uDist(-1., 1.);
	std::shuffle(c_img.begin(), c_img.end(), engine);

	if (s_f1 < 0 and s_f2 < 0) {
		strat = s_f1;

		for(auto it = c_img.begin(); it+1 != c_img.end(); ++it) {

			Image* ip = qobject_cast<Image*>(p->getById(*it));

			if (ip == nullptr) {
				continue;
			}

			QVector<qint64> lms1 = ip->getAttachedLandmarksIds();
			QSet<qint64> slms1(lms1.begin(), lms1.end());

			for (auto jt = it+1; jt != c_img.end(); ++jt) {

				Image* jp = qobject_cast<Image*>(p->getById(*jt));

				if (jp == nullptr) {
					continue;
				}

				QVector<qint64> lms2 = jp->getAttachedLandmarksIds();
				QSet<qint64> slms2(lms2.begin(), lms2.end());

				QSet<qint64> intersection = slms1.intersect(slms2);
				intersection = intersection.intersect(s_pts);

				if (intersection.size() > n_pt_selected) {

					if (strat == AutoPointNumber) {

						s_f1 = *it;
						s_f2 = *jt;
						n_pt_selected = intersection.size();
						s_intersection = intersection;

					} else if (strat == AutoMatrixQuality) {

						Eigen::Matrix3f Eapprox = ApproximateEssentialMatrix(ip, jp, intersection);

						auto svd = Eapprox.jacobiSvd();
						float score = std::log(svd.singularValues()[2]/svd.singularValues()[1]) + 6*std::log(svd.singularValues()[0]/svd.singularValues()[1]);
						score += uDist(engine)*0.1;

						if (score < svd_score) {

							s_f1 = *it;
							s_f2 = *jt;
							svd_score = score;
							s_intersection = intersection;
						}

					}
				}

			}

		}
	} else if (s_f1 < 0 or s_f2 < 0) {

		if (s_f2 >= 0 and s_f1 < 0) {
			strat = s_f1;
			s_f1 = s_f2;
			s_f2 = strat;
		}

		if (s_f2 < 0) {
			strat = s_f2;
		}

		Image* ip = qobject_cast<Image*>(p->getById(s_f1));

		if (ip == nullptr) {
			return r;
		}

		QVector<qint64> lms1 = ip->getAttachedLandmarksIds();
		QSet<qint64> slms1(lms1.begin(), lms1.end());

		for (auto jt = s_imgs.begin(); jt != s_imgs.end(); ++jt) {

			if (*jt == s_f1) {
				continue;
			}

			Image* jp = qobject_cast<Image*>(p->getById(*jt));

			if (jp == nullptr) {
				continue;
			}

			QVector<qint64> lms2 = jp->getAttachedLandmarksIds();
			QSet<qint64> slms2(lms2.begin(), lms2.end());

			QSet<qint64> intersection = slms1.intersect(slms2);

			if (intersection.size() > n_pt_selected) {

				if (strat == AutoPointNumber) {

					s_f2 = *jt;
					n_pt_selected = intersection.size();
					s_intersection = intersection;

				} else if (strat == AutoMatrixQuality) {

					Eigen::Matrix3f Eapprox = ApproximateEssentialMatrix(ip, jp, intersection);

					auto svd = Eapprox.jacobiSvd();
					float score = std::log(svd.singularValues()[2]/svd.singularValues()[1]) + 6*std::log(svd.singularValues()[0]/svd.singularValues()[1]);

					if (score < svd_score) {

						s_f2 = *jt;
						svd_score = score;
						s_intersection = intersection;
					}

				}
			}

		}

	}

	f1 = qobject_cast<Image*>(p->getById(s_f1));
	f2 = qobject_cast<Image*>(p->getById(s_f2));

	qDebug() << f1->objectName();
	qDebug() << f2->objectName();

	if (f1 == nullptr or f2 == nullptr) {
		return r;
	}

	if (s_intersection.empty()) {

		QVector<qint64> lms1 = f1->getAttachedLandmarksIds();
		QSet<qint64> slms1(lms1.begin(), lms1.end());

		QVector<qint64> lms2 = f2->getAttachedLandmarksIds();
		QSet<qint64> slms2(lms2.begin(), lms2.end());

		QSet<qint64> intersection = slms1.intersect(slms2);
	}

	Eigen::Matrix3f E = ApproximateEssentialMatrix(f1, f2, s_intersection);

	if (E.determinant() < 0) {
		E = -E;
	}

	Eigen::JacobiSVD<Eigen::Matrix3f, Eigen::NoQRPreconditioner> svd (E, Eigen::ComputeFullU | Eigen::ComputeFullV);

	Eigen::Matrix3f U = svd.matrixU();
	Eigen::Matrix3f V = svd.matrixV();

	Eigen::Matrix3f W = Eigen::Matrix3f::Zero();
	W(1,0) = -1;
	W(0,1) = 1;
	W(2,2) = 1;

	qDebug() << U.determinant();
	qDebug() << U.determinant();
	qDebug() << W.determinant();

	Eigen::Matrix3f R1 = U*W*V.transpose();
	Eigen::Matrix3f R2 = U*W.transpose()*V.transpose();

	//W(2,2) = 0;

	//Eigen::Matrix3f tCross = U*W*U.transpose();
	Eigen::Vector3f t1 = U.col(2);
	Eigen::Vector3f t2 = -t1;

	Eigen::Matrix3f R;
	Eigen::Vector3f t;

	bool ok;

	for (auto const& R_cand : {R1, R2}) {

		for (auto const& t_cand : {t1, t2}) {

			ok = true;

			for (qint64 lm_id : s_intersection) {

				ImageLandmark* imlm1 = f1->getImageLandmarkByLandmarkId(lm_id);
				ImageLandmark* imlm2 = f2->getImageLandmarkByLandmarkId(lm_id);

				Eigen::Vector3f t_p_c1 = triangulatePoint(getHomogeneousCoordinates(f1, {imlm1->x().value(), imlm1->y().value()}),
													   getHomogeneousCoordinates(f2, {imlm2->x().value(), imlm2->y().value()}),
													   R_cand,
													   t_cand);

				Eigen::Vector3f t_p_c2 = triangulatePoint(getHomogeneousCoordinates(f2, {imlm2->x().value(), imlm2->y().value()}),
													   getHomogeneousCoordinates(f1, {imlm1->x().value(), imlm1->y().value()}),
													   R_cand.transpose(),
													   -R_cand.transpose()*t_cand);

				if (t_p_c1.hasNaN() or t_p_c1.z() < 0) {
					ok = false;
					break;
				}

				if (t_p_c2.hasNaN() or t_p_c2.z() < 0) {
					ok = false;
					break;
				}

			}

			if (ok) {
				R = R_cand;
				t = t_cand;
				break;
			}

		}

		if (ok) {
			break;
		}

	}

	if (!ok) {
		r.cams.clear();
		return r;
	}

	r.cams.emplace(s_f1, Pt6D(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero()));
	r.cams.emplace(s_f2, Pt6D(R, t));

	for (qint64 lm_id : s_intersection) {

		ImageLandmark* imlm1 = f1->getImageLandmarkByLandmarkId(lm_id);
		ImageLandmark* imlm2 = f2->getImageLandmarkByLandmarkId(lm_id);

		Eigen::Vector3f t_p = triangulatePoint(getHomogeneousCoordinates(f1, {imlm1->x().value(), imlm1->y().value()}),
											   getHomogeneousCoordinates(f2, {imlm2->x().value(), imlm2->y().value()}),
											   R,
											   t);
		if (t_p.hasNaN()) {
			r.points.clear();
			r.cams.clear();
			return r;
		}

		r.points.emplace(lm_id, -t_p);

	}

	AffineTransform adjust = estimateTransform(r, p);

	//apply adjustement transform to all points

	for(auto & map_pt : r.points) {
		r.points[map_pt.first] = adjust.R*map_pt.second + adjust.t;
	}

	for(auto & map_cam : r.cams) {
		r.cams[map_cam.first].t = adjust.R*map_cam.second.t + adjust.t;
		r.cams[map_cam.first].R = adjust.R*map_cam.second.R;
	}

	return r;
}


Eigen::Matrix3f EightPointsSBAInitializer::ApproximateEssentialMatrix(Image* im1, Image* im2, QSet<qint64> const& intersection) {

	Eigen::Matrix<float, 9, Eigen::Dynamic> F;
	F.resize(9, intersection.size());

	int i = 0;
	for (qint64 lm_id : intersection) {

		ImageLandmark* imlm1 = im1->getImageLandmarkByLandmarkId(lm_id);
		ImageLandmark* imlm2 = im2->getImageLandmarkByLandmarkId(lm_id);

		auto hpt1 = getHomogeneousCoordinates(im1, {imlm1->x().value(), imlm1->y().value()});
		auto hpt2 = getHomogeneousCoordinates(im2, {imlm2->x().value(), imlm2->y().value()});

		F(0,i) = hpt2.x()*hpt1.x();
		F(1,i) = hpt2.x()*hpt1.y();
		F(2,i) = hpt2.x();
		F(3,i) = hpt2.y()*hpt1.x();
		F(4,i) = hpt2.y()*hpt1.y();
		F(5,i) = hpt2.y();
		F(6,i) = hpt1.x();
		F(7,i) = hpt1.y();
		F(8,i) = 1;

		i++;
	}

	auto svd = F.jacobiSvd(Eigen::ComputeFullU);
	auto e = svd.matrixU().col(8);
	Eigen::Matrix3f E;
	E << e[0], e[1], e[2],
		 e[3], e[4], e[5],
		 e[6], e[7], e[8];

	return E;
}

Eigen::Vector3f EightPointsSBAInitializer::triangulatePoint(Eigen::Vector2f const& pt1,
															Eigen::Vector2f const& pt2,
															Eigen::Matrix3f const& R,
															Eigen::Vector3f const& t) {

	Eigen::Vector3f y;
	y << pt1, 1;

	auto tmp1 = R.row(0) - pt2.x()*R.row(2);
	Eigen::Vector3f vec1;
	vec1 << tmp1[0], tmp1[1], tmp1[2];
	float x3_est1 = (pt2.x()*t[2] - t[0])/vec1.dot(y);

	auto tmp2 = R.row(1) - pt2.y()*R.row(2);
	Eigen::Vector3f vec2;
	vec2 << tmp2[0], tmp2[1], tmp2[2];
	float x3_est2 = (pt2.y()*t[2] - t[1])/vec2.dot(y);

	float x3_est = (x3_est1 + x3_est2)/2.0;

	return x3_est*y;

}

Eigen::Vector2f EightPointsSBAInitializer::getHomogeneousCoordinates(Image* im, Eigen::Vector2f const& pt) {

	if (!im->isInProject()) {
		return {NAN, NAN};
	}

	Camera* cam = qobject_cast<Camera*>(im->getProject()->getById(im->assignedCamera()));

	if (cam == nullptr) {
		return {NAN, NAN};
	}

	Eigen::Vector2f p(-pt.x(), pt.y());
	Eigen::Vector2f pp(-cam->opticalCenterX().value(), cam->opticalCenterY().value());

	return (p - pp)/cam->fLen().value();

}

AffineTransform EightPointsSBAInitializer::estimateTransform(InitialSolution const& solution, Project* p) {

	AffineTransform t;

	if (p == nullptr) {
		return t;
	}

	int countLm = 0;
	int countCam = 0;
	std::vector<qint64> lm_idxs;
	std::vector<qint64> cam_idxs;
	std::vector<Axis> axis;

	for (auto const& map_pt : solution.points) {
		qint64 id = map_pt.first;
		Landmark* lm = qobject_cast<Landmark*>(p->getById(id));

		if (lm != nullptr) {

			if (lm->xCoord().isSet()) {
				lm_idxs.push_back(id);
				axis.push_back(Axis::X);
			}

			if (lm->yCoord().isSet()) {
				lm_idxs.push_back(id);
				axis.push_back(Axis::Y);
			}

			if (lm->zCoord().isSet()) {
				lm_idxs.push_back(id);
				axis.push_back(Axis::Z);
			}

			if (lm->xCoord().isSet() or lm->yCoord().isSet() or lm->zCoord().isSet()) {
				countLm++;
			}
		}
	}

	for (auto const& map_pt : solution.cams) {
		qint64 id = map_pt.first;
		Image* im = qobject_cast<Image*>(p->getById(id));

		if (im != nullptr) {

			if (im->xCoord().isSet()) {
				cam_idxs.push_back(id);
				axis.push_back(Axis::X);
			}

			if (im->yCoord().isSet()) {
				cam_idxs.push_back(id);
				axis.push_back(Axis::Y);
			}

			if (im->zCoord().isSet()) {
				cam_idxs.push_back(id);
				axis.push_back(Axis::Z);
			}

			if (im->xCoord().isSet() or im->yCoord().isSet() or im->zCoord().isSet()) {
				countCam++;
			}
		}
	}

	int nobs = axis.size();

	if (nobs < 7) {
		return t;
	}

	Eigen::VectorXf obs;
	obs.resize(nobs);

	Eigen::Matrix3Xf pts;
	pts.resize(3,countLm + countCam);

	QMap<qint64, int> idsMap;
	std::vector<int> idxs;

	int n_ptId = 0;

	for (int i = 0; i < static_cast<int>(lm_idxs.size()); i++) {
		qint64 id = lm_idxs[i];
		Landmark* lm = qobject_cast<Landmark*>(p->getById(id));

		int ptId;

		if (idsMap.contains(id)) {
			ptId = idsMap[id];
		} else {
			ptId = n_ptId;
			n_ptId++;
			idsMap.insert(id, ptId);

			pts(0,ptId) = solution.points.at(id).x();
			pts(1,ptId) = solution.points.at(id).y();
			pts(2,ptId) = solution.points.at(id).z();
		}

		idxs.push_back(ptId);

		if (axis[i] == Axis::X) {
			obs[i] = lm->xCoord().value();
		} else if (axis[i] == Axis::Y) {
			obs[i] = lm->yCoord().value();
		} else if (axis[i] == Axis::Z) {
			obs[i] = lm->zCoord().value();
		}

	}

	for (int i = 0; i < static_cast<int>(cam_idxs.size()); i++) {
		int k = i + lm_idxs.size();

		qint64 id = cam_idxs[i];
		Image* im = qobject_cast<Image*>(p->getById(id));

		int ptId;

		if (idsMap.contains(id)) {
			ptId = idsMap[id];
		} else {
			ptId = n_ptId;
			n_ptId++;
			idsMap.insert(id, ptId);

			pts(0,ptId) = solution.cams.at(id).t.x();
			pts(1,ptId) = solution.cams.at(id).t.y();
			pts(2,ptId) = solution.cams.at(id).t.z();
		}

		idxs.push_back(ptId);

		if (axis[k] == Axis::X) {
			obs[k] = im->xCoord().value();
		} else if (axis[k] == Axis::Y) {
			obs[k] = im->yCoord().value();
		} else if (axis[k] == Axis::Z) {
			obs[k] = im->zCoord().value();
		}

	}

	IterativeTermination status;
	AffineTransform s = estimateShapePreservingMap(obs, pts, idxs, axis, &status, 1500, 1e-4, 2e-1, 1e-1);

	if (status == IterativeTermination::Error) {
		return t;
	}

	return s;

}


const int FrontCamSBAInitializer::MinimalNObs = 6;

FrontCamSBAInitializer::FrontCamSBAInitializer(int min_n_obs, int rot_adjustement_steps) :
	_min_n_obs(min_n_obs),
	_rot_adjustement_steps(rot_adjustement_steps)
{

}

SBAInitializer::InitialSolution FrontCamSBAInitializer::computeInitialSolution(Project* p, const QSet<qint64> &s_pts, const QSet<qint64> &s_imgs) {

	InitialSolution r;
	r.points.clear();
	r.cams.clear();

	if (_min_n_obs < MinimalNObs) {
		return r;
	}

	//first select the camera with the most observations.

	Image* im_start = nullptr;
	Camera* cam_start = nullptr;
	int n_obs = _min_n_obs;
	int n_world_obs = 0;

	for (qint64 id : s_imgs) {

		Image* candidate = qobject_cast<Image*>(p->getById(id));

		if (candidate == nullptr) {
			continue;
		}

		Camera* c_cam = qobject_cast<Camera*>(p->getById(candidate->assignedCamera()));

		if (c_cam == nullptr) {
			continue;
		}

		QVector<qint64> lms = candidate->listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName);
		int c_n_obs = lms.size()*2;
		int c_w_obs = 0;

		for (qint64 lm_id : lms) {
			ImageLandmark* ilm = qobject_cast<ImageLandmark*>(candidate->getById(lm_id));

			if (ilm == nullptr) {
				continue;
			}

			Landmark* lm = ilm->attachedLandmark();

			if (lm == nullptr) {
				continue;
			}

			c_w_obs += (lm->xCoord().isSet()) ? 1 : 0;
			c_w_obs += (lm->yCoord().isSet()) ? 1 : 0;
			c_w_obs += (lm->zCoord().isSet()) ? 1 : 0;
		}

		if (c_w_obs >= n_world_obs) {
			if (c_n_obs > n_obs and c_n_obs > MinimalNObs) {
				im_start = candidate;
				cam_start = c_cam;
				n_obs = c_n_obs;
				n_world_obs = c_w_obs;
			}
		}
	}

	if (im_start == nullptr) {
		return r;
	}

	//Init the points in the image in front of the camera, assuming the camera is at (0,0,0) facing down.

	QSet<qint64> presents;
	Eigen::Matrix3f rot;
	Eigen::Matrix3f R0 = Eigen::Matrix3f::Identity();
	Eigen::Matrix3f Rx180 = -Eigen::Matrix3f::Identity();
	Rx180(0,0) = 1;

	QVector<qint64> lms = im_start->listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName);

	for (qint64 lm_id : lms) {
		ImageLandmark* ilm = qobject_cast<ImageLandmark*>(im_start->getById(lm_id));

		if (ilm == nullptr) {
			continue;
		}

		if (!s_pts.contains(ilm->attachedLandmarkid())) {
			continue;
		}

		float px = (ilm->x().value() - cam_start->opticalCenterX().value())/cam_start->fLen().value();
		float py = -(ilm->y().value() - cam_start->opticalCenterY().value())/cam_start->fLen().value();
		float pz = -1;

		r.points.emplace(ilm->attachedLandmarkid(), Pt3D(px, py, pz));
		presents.insert(ilm->attachedLandmarkid());

		float rx = (im_start->xRot().isSet()) ? im_start->xRot().value()/180.*M_PI : 0.;
		float ry = (im_start->yRot().isSet()) ? im_start->yRot().value()/180.*M_PI : 0.;
		float rz = (im_start->zRot().isSet()) ? im_start->zRot().value()/180.*M_PI : 0.;

		rot = Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX())
			* Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ());

		//float angle_conv = 180.*M_PI;
		r.cams.emplace(im_start->internalId(), Pt6D(R0, Eigen::Vector3f(0,0,0)));

	}

	Image* currentImg = nullptr;
	QSet<qint64> processedImgs;
	processedImgs.insert(im_start->internalId());

	do {

		currentImg = nullptr;
		int n_presents = 2;

		QSet<qint64> alreadyPresent;
		QSet<qint64> toAdd;

		bool isUp;

		for (qint64 id : s_imgs) {

			if (processedImgs.contains(id)) {
				continue;
			}

			Image* im = qobject_cast<Image*>(p->getById(id));

			if (im == nullptr) {
				continue;
			}

			QVector<qint64> imlm = im->getAttachedLandmarksIds();

			QSet<qint64> c_alreadyPresent;
			QSet<qint64> c_toAdd;

			for (qint64 lm_id : imlm) {
				if (presents.contains(lm_id)) {
					c_alreadyPresent.insert(lm_id);
				} else {
					c_toAdd.insert(lm_id);
				}
			}

			if (c_alreadyPresent.size() > n_presents) {
				currentImg = im;
				n_presents = c_alreadyPresent.size();
				alreadyPresent = c_alreadyPresent;
				toAdd = c_toAdd;
			}

		}

		if (currentImg != nullptr) {

			Camera* currentCam = qobject_cast<Camera*>(p->getById(currentImg->assignedCamera()));

			if (currentCam == nullptr) {
				r.cams.clear();
				r.points.clear();
				return r;
			}

			QVector<ImageLandmark*> selectedPt;

			for (qint64 imlm_id : currentImg->listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName)) {
				ImageLandmark* imlm = currentImg->getImageLandmark(imlm_id);

				if (imlm == nullptr) {
					continue;
				}

				if (alreadyPresent.contains(imlm->attachedLandmarkid())) {
					selectedPt.push_back(imlm);
				}

				if (selectedPt.size() == 3) {
					break;
				}
			}

			if (selectedPt.size() < 3) {
				r.cams.clear();
				r.points.clear();
				return r;
			}

			qint64 pt1 = selectedPt[0]->attachedLandmarkid();
			qint64 pt2 = selectedPt[1]->attachedLandmarkid();
			qint64 pt3 = selectedPt[2]->attachedLandmarkid();

			float cross = (r.points[pt2].x() - r.points[pt1].x())*(r.points[pt3].y() - r.points[pt1].y());
			cross -= (r.points[pt3].x() - r.points[pt1].x())*(r.points[pt2].y() - r.points[pt1].y());


			float cross_img = (selectedPt[1]->x().value() - selectedPt[0]->x().value())*(selectedPt[0]->y().value() - selectedPt[2]->y().value());
			cross_img -= (selectedPt[2]->x().value() - selectedPt[0]->x().value())*(selectedPt[0]->y().value() - selectedPt[1]->y().value());

			isUp = (cross > 0.) == (cross_img > 0.);

			Eigen::Vector4f x(0, 0, 0, 0);
			Eigen::MatrixXf A = Eigen::MatrixXf::Zero(alreadyPresent.size()*2, 4);
			Eigen::VectorXf l = Eigen::VectorXf::Zero(alreadyPresent.size()*2);
			Eigen::VectorXf l0 = Eigen::VectorXf::Zero(alreadyPresent.size()*2);

			for (int i = 0; i < _rot_adjustement_steps; i++) {

				int j = 0;
				for(auto it = alreadyPresent.begin(); it != alreadyPresent.end(); it++, j += 2) {

					qint64 ptId = *it;

					ImageLandmark* imlm = currentImg->getImageLandmarkByLandmarkId(ptId);

					if (imlm == nullptr) {
						r.cams.clear();
						r.points.clear();
						return r;
					}

					float i_h_x = (imlm->x().value() - currentCam->opticalCenterX().value())/currentCam->fLen().value();
					float i_h_y = -(imlm->y().value() - currentCam->opticalCenterY().value() )/currentCam->fLen().value();

					if (!isUp) {
						i_h_y = -i_h_y;
					}

					A(j,0) = expf(x(0))*(i_h_x*cos(x(1)) + i_h_y*sin(x(1)));
					A(j,1) = expf(x(0))*(i_h_y*cos(x(1)) - i_h_x*sin(x(1)));
					A(j,2) = 1;


					A(j+1,0) = expf(x(0))*(-i_h_x*sin(x(1)) + i_h_y*cos(x(1)));
					A(j+1,1) = expf(x(0))*(-i_h_y*sin(x(1)) - i_h_x*cos(x(1)));
					A(j+1,3) = 1;

					if (i == 0) {
						l(j) = r.points[ptId].x();
						l(j+1) = r.points[ptId].y();
					}

					l0(j) = x(0)*(i_h_x*cos(x(1)) + i_h_y*sin(x(1))) + x(2);
					l0(j+1) = x(0)*(-i_h_x*sin(x(1)) + i_h_y*cos(x(1))) + x(3);
				}

				Eigen::Matrix4f P = (A.transpose()*A);
				Eigen::Matrix4f Qxx = P.inverse();

				x = x + Qxx*A.transpose()*(l-l0);
			}

			for (qint64 ptId : toAdd) {

				ImageLandmark* imlm = currentImg->getImageLandmarkByLandmarkId(ptId);

				if (imlm == nullptr) {
					r.cams.clear();
					r.points.clear();
					return r;
				}

				float i_h_x = (imlm->x().value() - currentCam->opticalCenterX().value())/currentCam->fLen().value();
				float i_h_y = -(imlm->y().value() - currentCam->opticalCenterY().value() )/currentCam->fLen().value();

				if (!isUp) {
					i_h_y = -i_h_y;
				}

				float tx = expf(x(0))*(i_h_x*cos(x(1)) + i_h_y*sin(x(1))) + x(2);
				float ty = expf(x(0))*(-i_h_x*sin(x(1)) + i_h_y*cos(x(1))) + x(3);
				float tz = -1;


				r.points.emplace(ptId, Pt3D(tx, ty, tz));
				presents.insert(ptId);

			}

			r.cams.emplace(currentImg->internalId(), Pt6D(isUp ? R0 : Rx180, Eigen::Vector3f(x(2), x(3), isUp ? 0.1f + 1/expf(x(0)) : -0.1f -1/expf(x(0)))));
			processedImgs.insert(currentImg->internalId());

		}

	} while (currentImg != nullptr);

	//apply initial cam rotation to all points

	for(auto & map_pt : r.points) {
		r.points[map_pt.first] = rot*map_pt.second;
	}

	for(auto & map_cam : r.cams) {
		r.cams[map_cam.first].t = rot*map_cam.second.t;
		r.cams[map_cam.first].R = rot*map_cam.second.R;
	}

	//adjust points positions as much as possible.

	int nMeasures = 0;

	for (qint64 id : processedImgs) {
		Image* img = qobject_cast<Image*>(p->getById(id));

		if (img == nullptr) {
			continue;
		}

		if (img->xCoord().isSet()) {
			nMeasures++;
		}

		if (img->yCoord().isSet()) {
			nMeasures++;
		}

		if (img->zCoord().isSet()) {
			nMeasures++;
		}
	}

	for (qint64 id : presents) {
		Landmark* lm = qobject_cast<Landmark*>(p->getById(id));

		if (lm == nullptr) {
			continue;
		}

		if (lm->xCoord().isSet()) {
			nMeasures++;
		}
		if (lm->yCoord().isSet()) {
			nMeasures++;
		}
		if (lm->zCoord().isSet()) {
			nMeasures++;
		}
	}

	if (nMeasures >= 4) {

		Eigen::MatrixXf A = Eigen::MatrixXf::Zero(nMeasures, 4);
		Eigen::VectorXf l = Eigen::VectorXf::Zero(nMeasures);
		int row = 0;

		for (qint64 id : processedImgs) {
			Image* img = qobject_cast<Image*>(p->getById(id));

			if (img == nullptr) {
				continue;
			}

			if (img->xCoord().isSet()) {
				A(row, 0) = r.cams[img->internalId()].t.x();
				A(row, 1) = 1;
				l(row) = img->xCoord().value();
				row++;
			}

			if (img->yCoord().isSet()) {
				A(row, 0) = r.cams[img->internalId()].t.y();
				A(row, 2) = 1;
				l(row) = img->yCoord().value();
				row++;
			}

			if (img->zCoord().isSet()) {
				A(row, 0) = r.cams[img->internalId()].t.z();
				A(row, 3) = 1;
				l(row) = img->zCoord().value();
				row++;
			}
		}

		for (qint64 id : presents) {
			Landmark* lm = qobject_cast<Landmark*>(p->getById(id));

			if (lm == nullptr) {
				continue;
			}

			if (lm->xCoord().isSet()) {
				A(row, 0) = r.points[lm->internalId()].x();
				A(row, 1) = 1;
				l(row) = lm->xCoord().value();
				row++;
			}

			if (lm->yCoord().isSet()) {
				A(row, 0) = r.points[lm->internalId()].y();
				A(row, 2) = 1;
				l(row) = lm->yCoord().value();
				row++;
			}

			if (lm->zCoord().isSet()) {
				A(row, 0) = r.points[lm->internalId()].z();
				A(row, 3) = 1;
				l(row) = lm->zCoord().value();
				row++;
			}
		}

		Eigen::Matrix4f P = A.transpose()*A;
		Eigen::Matrix4f Qxx = P.inverse();
		Eigen::Vector4f x = Qxx*(A.transpose()*l);

		float s = x(0);
		Eigen::Vector3f t = Eigen::Vector3f(x(1), x(2), x(3));

		//apply initial scale and translation to all points

		for(auto & map_pt : r.points) {
			r.points[map_pt.first] = s*map_pt.second + t;
		}

		for(auto & map_cam : r.cams) {
			r.cams[map_cam.first].t = s*map_cam.second.t + t;
		}


	}

	return r;

}


RandomPosSBAInitializer::RandomPosSBAInitializer(float camDist, float camStd, float pointStd) :
	_camDist(camDist),
	_camStd(camStd),
	_pointStd(pointStd)
{

}

SBAInitializer::InitialSolution RandomPosSBAInitializer::computeInitialSolution(Project* p, QSet<qint64> const& s_pts, QSet<qint64> const& s_imgs) {

	Q_UNUSED(p);

	InitialSolution r;

	std::random_device rd;

	std::default_random_engine engine(rd());
	std::uniform_real_distribution<float> cDist(-_camStd, _camStd);
	std::uniform_real_distribution<float> pDist(-_pointStd, _pointStd);
	std::uniform_real_distribution<float> latDist(0, M_PI/2);
	std::uniform_real_distribution<float> azDist(0, 2*M_PI);

	for (qint64 id : s_pts) {
		r.points.emplace(id, Eigen::Vector3f(pDist(engine), pDist(engine), pDist(engine)));
	}

	for (qint64 id : s_imgs) {
		Eigen::Matrix3f rot = Eigen::AngleAxisf(azDist(engine), Eigen::Vector3f::UnitZ()).toRotationMatrix()
				* Eigen::AngleAxisf(latDist(engine), Eigen::Vector3f::UnitX()).toRotationMatrix();

		Eigen::Vector3f t(cDist(engine), cDist(engine), _camDist + cDist(engine));
		r.cams.emplace(id, Pt6D(rot, rot*t));
	}

	return r;

}

} // namespace StereoVisionApp
