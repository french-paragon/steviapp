#include "sbainitializer.h"

#include "datablocks/project.h"
#include "datablocks/image.h"
#include "datablocks/landmark.h"
#include "datablocks/camera.h"

#include <eigen3/Eigen/Core>
#include <Eigen/Geometry>

#include "geometry/imagecoordinates.h"
#include "geometry/pointcloudalignment.h"
#include "geometry/alignement.h"
#include "geometry/geometricexception.h"

#include <QSet>
#include <QDebug>

#include <random>
#include <algorithm>

namespace StereoVisionApp {

SBAInitializer::~SBAInitializer() {

}


EightPointsSBAMultiviewInitializer::EightPointsSBAMultiviewInitializer(qint64 f1,
																	   bool preconstrain,
																	   bool useConstraintsRefinement) :
	_f1(f1),
	_preconstrain(preconstrain),
	_useConstraintsRefinement(useConstraintsRefinement)
{

}

SBAInitializer::InitialSolution EightPointsSBAMultiviewInitializer::computeInitialSolution(Project* p,
																		   QSet<qint64> const& s_pts,
																		   QSet<qint64> const& s_imgs) {
	InitialSolution r;
	r.cams.clear();
	r.points.clear();

	QMap<qint64, QVector<qint64>> imagesConnections;
	QMap<qint64, qint64> imagesConnected;
	qint64 selectedFrame = _f1;

	if (_f1 < 0) {
		for (auto it1 = s_imgs.begin(); it1+1 != s_imgs.end(); it1++) {

			Image* img1 = qobject_cast<Image*>(p->getById(*it1));

			if (img1 == nullptr) {
				continue;
			}

			QVector<qint64> lms = img1->getAttachedLandmarksIds();
			QSet<qint64> attachedLandmarks1 = QSet<qint64>(lms.begin(), lms.end());

			for (auto it2 = it1+1; it2 != s_imgs.end(); it2++) {

				Image* img2 = qobject_cast<Image*>(p->getById(*it2));

				if (img2 == nullptr) {
					continue;
				}

				lms = img2->getAttachedLandmarksIds();
				QSet<qint64> attachedLandmarks2 = QSet<qint64>(lms.begin(), lms.end());
				QSet<qint64> inter = attachedLandmarks1.intersect(attachedLandmarks2).intersect(s_pts);

				if (inter.size() >= 8) {
					if (!imagesConnected.contains(*it1)) {
						imagesConnected[*it1] = 0;
					}
					if (!imagesConnected.contains(*it2)) {
						imagesConnected[*it2] = 0;
					}
					if (!imagesConnections.contains(*it1)) {
						imagesConnections[*it1] = QVector<qint64>();
					}
					if (!imagesConnections.contains(*it2)) {
						imagesConnections[*it2] = QVector<qint64>();
					}

					imagesConnections[*it1].push_back(*it2);
					imagesConnections[*it2].push_back(*it1);

					imagesConnected[*it1] += 1;
					imagesConnected[*it2] += 1;
				}
			}
		}

		qint64 nConnections = 0;

		QVector<qint64> keys = s_imgs.values().toVector();

		std::random_device rd;
		std::default_random_engine re(rd());
		std::shuffle(keys.begin(), keys.end(), re);

		for (qint64 id : keys) {
			if (imagesConnected[id] > nConnections) {
				selectedFrame = id;
				nConnections = imagesConnected[id];
			}
		}
	} else {
		imagesConnections[_f1] = QVector<qint64>();

		Image* img1 = qobject_cast<Image*>(p->getById(_f1));

		if (img1 == nullptr) {
			return r;
		}

		QVector<qint64> lms = img1->getAttachedLandmarksIds();
		QSet<qint64> attachedLandmarks1 = QSet<qint64>(lms.begin(), lms.end());

		for (qint64 id2 : s_imgs) {

			if (id2 == _f1) {
				continue;
			}

			Image* img2 = qobject_cast<Image*>(p->getById(id2));

			if (img2 == nullptr) {
				continue;
			}

			lms = img2->getAttachedLandmarksIds();
			QSet<qint64> attachedLandmarks2 = QSet<qint64>(lms.begin(), lms.end());
			QSet<qint64> inter = attachedLandmarks1.intersect(attachedLandmarks2).intersect(s_pts);

			if (inter.size() >= 8) {

				imagesConnections[_f1].push_back(id2);
			}
		}
	}

	Image* f1 = qobject_cast<Image*>(p->getById(selectedFrame));
	qDebug() << f1->objectName();

	QVector<qint64> tmp = f1->getAttachedLandmarksIds();
	QSet<qint64> lm_im1 = QSet<qint64>(tmp.begin(), tmp.end());
	lm_im1.intersect(s_pts);

	QMap<qint64, std::vector<Eigen::Vector3f>> pts;

	QVector<qint64> targets = imagesConnections[selectedFrame];
	QSet<qint64> toProcess = QSet<qint64>(targets.begin(), targets.end());
	qDebug() << toProcess.size();
	QSet<qint64> processedLandmark = QSet<qint64>();

	while (!toProcess.isEmpty()) {

		Image* nextInLine = nullptr;
		qint64 nConnections = 0;
		QVector<qint64> tiePoints;
		QVector<qint64> scaleMeasureCand;

		for (qint64 id : toProcess) {
			Image* f2 = qobject_cast<Image*>(p->getById(id));

			QVector<qint64> tmp = f2->getAttachedLandmarksIds();
			QSet<qint64> lm_im2 = QSet<qint64>(tmp.begin(), tmp.end());

			QSet<qint64> inter = lm_im2.intersect(lm_im1);

			if (!processedLandmark.isEmpty()) {
				inter.intersect(processedLandmark);
			}

			if (inter.size() > nConnections) {
				nextInLine = f2;
				nConnections = inter.size();
				tiePoints = lm_im2.values().toVector();

				if (!processedLandmark.isEmpty()) {
					scaleMeasureCand = inter.values().toVector();
				}
			}
		}

		Eigen::Array2Xf coordsIm1 = getHomogeneousImageCoordinates(f1, tiePoints);
		Eigen::Array2Xf coordsIm2 = getHomogeneousImageCoordinates(nextInLine, tiePoints);

		StereoVision::Geometry::AffineTransform T;
		Eigen::Array3Xf proj;

		try {

			T = StereoVision::Geometry::findTransform(coordsIm1, coordsIm2);
			proj = reprojectPoints(T, coordsIm1, coordsIm2);

		} catch (StereoVision::Geometry::GeometricException const&) {
			toProcess.remove(nextInLine->internalId());
			continue;
		}

		float scale = 0.;

		if (!processedLandmark.isEmpty()) {

			if (scaleMeasureCand.isEmpty()) {
				toProcess.remove(nextInLine->internalId());
				continue;
			}

			for (qint64 id : scaleMeasureCand) {
				int i = tiePoints.indexOf(id);
				scale += (pts[id].front().array()/proj.col(i).array()).mean();
			}

			scale /= scaleMeasureCand.size();

		} else {
			scale = 1.;
		}

		for (int i = 0; i < tiePoints.size(); i++) {
			pts[tiePoints[i]].push_back(Eigen::Vector3f(scale*proj.col(i)));
			processedLandmark.insert(tiePoints[i]);
		}

		toProcess.remove(nextInLine->internalId());
	}

	r.cams.emplace(selectedFrame, Pt6D(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero()));

	for (qint64 id : pts.keys()) {

		Pt3D pt = Pt3D::Zero();

		for (size_t i = 0; i < pts[id].size(); i++) {
			pt += pts[id][i];
		}

		pt /= pts[id].size();

		r.points.emplace(id, pt);
	}

	completeSolution(r, p, s_pts, s_imgs);

	if (_preconstrain) {
		StereoVision::Geometry::AffineTransform adjust = estimateTransform(r, p, _useConstraintsRefinement);
		StereoVision::Geometry::ShapePreservingTransform rigid = affine2ShapePreservingMap(adjust);
		Eigen::Matrix3f rotationPart = StereoVision::Geometry::rodriguezFormula(rigid.r);
		//apply adjustement transform to all points

		for(auto & map_pt : r.points) {
			r.points[map_pt.first] = adjust.R*map_pt.second + adjust.t;
		}

		for(auto & map_cam : r.cams) {
			r.cams[map_cam.first].t = adjust.R*map_cam.second.t + adjust.t;
			r.cams[map_cam.first].R = rotationPart*map_cam.second.R;
		}
	}

	return r;

}


EightPointsSBAInitializer::EightPointsSBAInitializer(qint64 f1,
													 qint64 f2,
													 bool preconstrain,
													 bool useConstraintsRefinement) :
	_f1(f1),
	_f2(f2),
	_preconstrain(preconstrain),
	_useConstraintsRefinement(useConstraintsRefinement)
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

						float score = scoreApproximateEssentialMatrix(Eapprox);
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

					s_f2 = *jt;
					n_pt_selected = intersection.size();
					s_intersection = intersection;

				} else if (strat == AutoMatrixQuality) {

					Eigen::Matrix3f Eapprox = ApproximateEssentialMatrix(ip, jp, intersection);

					float score = scoreApproximateEssentialMatrix(Eapprox);
					score += uDist(engine)*0.1;

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

	QVector<qint64> ids = s_intersection.values().toVector();

	Eigen::Array2Xf homCoordsIm1 = getHomogeneousImageCoordinates(f1, ids);
	Eigen::Array2Xf homCoordsIm2 = getHomogeneousImageCoordinates(f2, ids);

	StereoVision::Geometry::AffineTransform T;
	Eigen::Array3Xf reproj;

	try {

		T = StereoVision::Geometry::findTransform(homCoordsIm1, homCoordsIm2);
		reproj = reprojectPoints(T, homCoordsIm1, homCoordsIm2);

	} catch (StereoVision::Geometry::GeometricException const&) {
		return r;
	}

	r.cams.emplace(s_f1, Pt6D(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero()));
	r.cams.emplace(s_f2, Pt6D(T.R.transpose(), -T.R.transpose()*T.t));

	for (int i = 0; i < ids.size(); i++) {
		qint64 lm_id = ids[i];
		Eigen::Vector3f t_p = reproj.col(i);
		r.points.emplace(lm_id, t_p);
	}

	completeSolution(r, p, s_pts, s_imgs);

	if (_preconstrain) {
		StereoVision::Geometry::AffineTransform adjust = estimateTransform(r, p, _useConstraintsRefinement);

		//apply adjustement transform to all points

		for(auto & map_pt : r.points) {
			r.points[map_pt.first] = adjust.R*map_pt.second + adjust.t;
		}

		for(auto & map_cam : r.cams) {
			r.cams[map_cam.first].t = adjust.R*map_cam.second.t + adjust.t;
			r.cams[map_cam.first].R = adjust.R*map_cam.second.R;
		}
	}

	return r;
}

Eigen::Array3Xf PhotometricInitializer::getLandmarksWorldCoordinates(InitialSolution const& sol, QVector<qint64> const& idxs) {

	Eigen::Array3Xf pts_world;
	pts_world.resize(3, idxs.size());

	for (int i = 0; i < idxs.size(); i++) {

		pts_world.col(i) = sol.points.at(idxs[i]).array();

	}

	return pts_world;
}

Eigen::Matrix3f PhotometricInitializer::ApproximateEssentialMatrix(Image* im1, Image* im2, QSet<qint64> const& intersection) {

	QVector<qint64> v_ids = intersection.values().toVector();

	Eigen::Array2Xf coordsIm1 = getHomogeneousImageCoordinates(im1, v_ids);
	Eigen::Array2Xf coordsIm2 = getHomogeneousImageCoordinates(im2, v_ids);

	return StereoVision::Geometry::estimateEssentialMatrix(coordsIm1, coordsIm2);
}

float EightPointsSBAInitializer::scoreApproximateEssentialMatrix(Eigen::Matrix3f const& Eapprox) {

	auto svd = Eapprox.jacobiSvd();
	return std::log(svd.singularValues()[2]/svd.singularValues()[1]) + 6*std::log(svd.singularValues()[0]/svd.singularValues()[1]);
}

Eigen::Array2Xf PhotometricInitializer::getHomogeneousImageCoordinates(Image* im, QVector<qint64> ids) {

	Project* p = im->getProject();
	Camera* cam = qobject_cast<Camera*>(p->getById(im->assignedCamera()));

	return Image2HomogeneousCoordinates(im->getImageLandmarksCoordinates(ids),
										cam->fLen().value(),
										cam->fLenY(),
										Eigen::Vector2f(cam->opticalCenterX().value(), cam->opticalCenterY().value()),
										StereoVision::Geometry::ImageAnchors::TopLeft) ;

}


StereoVision::Geometry::AffineTransform PhotometricInitializer::estimateTransform(InitialSolution const& solution, Project* p, bool useConstraintsRefinement) {

	using Axis = StereoVision::Geometry::Axis;

	StereoVision::Geometry::AffineTransform t;

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

	StereoVision::Geometry::IterativeTermination status;
	StereoVision::Geometry::AffineTransform s;

	if (useConstraintsRefinement) {

		try {
			s = estimateShapePreservingMap(obs, pts, idxs, axis, &status, 50, 1e-4, 5e-1, 1e-1);
		} catch (StereoVision::Geometry::GeometricException const& e) {
			return t;
		}

	} else {
		s = estimateQuasiShapePreservingMap(obs, pts, idxs, axis, 2e-1, &status, 1e-6, 500, false);
		StereoVision::Geometry::ShapePreservingTransform s_tmps = affine2ShapePreservingMap(s);
		s = s_tmps.toAffineTransform();
	}

	if (status == StereoVision::Geometry::IterativeTermination::Error) {
		return t;
	}

	return s;

}


bool PhotometricInitializer::completeSolution(InitialSolution & solution,
											  Project* p,
											  QSet<qint64> const& s_pts,
											  QSet<qint64> const& s_imgs,
											  int minNTiePoints,
											  int minViewingImgs) {

	QSet<qint64> pts2add = s_pts;
	QSet<qint64> ptsAdded;
	QSet<qint64> img2add = s_imgs;
	QSet<qint64> imgsAdded;

	for (auto map_it : solution.cams) {
		img2add.remove(map_it.first);
		imgsAdded.insert(map_it.first);
	}

	for (auto map_it : solution.points) {
		pts2add.remove(map_it.first);
		ptsAdded.insert(map_it.first);
	}

	while (!pts2add.empty() and !img2add.empty()) {

		int n_additions = 0;

		Image* imgAdded;

		do {
			imgAdded = nullptr;
			int nTiePoints = minNTiePoints-1;
			QSet<qint64> tiePoints;

			for (qint64 id : img2add) {

				Image* im = qobject_cast<Image*>(p->getById(id));

				if (im == nullptr) {
					continue;
				}

				QVector<qint64> tmp = im->getAttachedLandmarksIds();
				QSet<qint64> lms(tmp.begin(), tmp.end());
				lms.intersect(ptsAdded);

				if (nTiePoints < lms.size()) {
					imgAdded = im;
					nTiePoints = lms.size();
					tiePoints = lms;
				}

			}

			if (imgAdded != nullptr) {
				if (alignImage(solution, p, imgAdded, tiePoints)) {
					img2add.remove(imgAdded->internalId());
					imgsAdded.insert(imgAdded->internalId());
					n_additions++;
				} else {
					img2add.remove(imgAdded->internalId());
				}
			}

		} while(imgAdded != nullptr);

		Landmark* lmAdded;

		do {
			lmAdded = nullptr;
			int nViewingImages = minViewingImgs-1;
			QSet<qint64> vImgs;

			for (qint64 id : pts2add) {

				Landmark* lm = qobject_cast<Landmark*>(p->getById(id));

				QSet<qint64> vI = lm->getViewingImgInList(imgsAdded);

				if (vI.size() > nViewingImages) {
					lmAdded = lm;
					nViewingImages = vI.size();
					vImgs = vI;
				}
			}

			if (lmAdded != nullptr) {
				if (triangulatePoint(solution, p, lmAdded->internalId(), vImgs.values().toVector())) {
					pts2add.remove(lmAdded->internalId());
					ptsAdded.insert(lmAdded->internalId());
					n_additions++;
				} else {
					pts2add.remove(lmAdded->internalId());
				}
			}

		} while(lmAdded != nullptr);

		if (n_additions == 0) {
			return false;
		}
	}

	return imgsAdded == s_imgs and ptsAdded == s_pts;

}

bool PhotometricInitializer::alignImage(InitialSolution & solution, Project* p, Image* img, QSet<qint64> & pts) {

	Camera* cam = qobject_cast<Camera*>(p->getById(img->assignedCamera()));

	if (cam == nullptr) {
		return false;
	}

	QVector<qint64> tmp = img->getAttachedLandmarksIds();
	QSet<qint64> matchedPts = QSet<qint64>(tmp.begin(), tmp.end());
	matchedPts.intersect(pts);

	tmp = matchedPts.values().toVector();

	Eigen::Array3Xf pts_world = getLandmarksWorldCoordinates(solution, tmp);
	Eigen::Array2Xf pts_cam = img->getImageLandmarksCoordinates(tmp);

	pts_cam = Image2HomogeneousCoordinates(pts_cam,
										   cam->fLen().value(),
										   cam->fLenY(),
										   Eigen::Vector2f(cam->opticalCenterX().value(), cam->opticalCenterY().value()),
										   StereoVision::Geometry::ImageAnchors::TopLeft);

	StereoVision::Geometry::AffineTransform T = StereoVision::Geometry::pnp(pts_cam, pts_world);

	solution.cams.emplace(img->internalId(), StereoVision::Geometry::AffineTransform(T.R.transpose(), -T.R.transpose()*T.t));
	return true;

}

bool PhotometricInitializer::triangulatePoint(InitialSolution & solution, Project* p, qint64 pt, QVector<qint64> const& imgs) {

	Eigen::Vector3f coord = Eigen::Vector3f::Zero();
	int n_obs = 0;

	for(int i = 0; i+2 < imgs.size(); i++) {

		Image* im1 = qobject_cast<Image*>(p->getById(imgs[i]));

		if (im1 == nullptr) {
			continue;
		}

		ImageLandmark* im_lm1 = im1->getImageLandmarkByLandmarkId(pt);

		if (im_lm1 == nullptr) {
			continue;
		}

		StereoVision::Geometry::AffineTransform const& cam12world = solution.cams.at(imgs[i]);

		for (int j = i+1; j < imgs.size(); j++) {

			Image* im2 = qobject_cast<Image*>(p->getById(imgs[j]));

			if (im2 == nullptr) {
				continue;
			}

			ImageLandmark* im_lm2 = im1->getImageLandmarkByLandmarkId(pt);

			if (im_lm2 == nullptr) {
				continue;
			}

			StereoVision::Geometry::AffineTransform const& cam2toworld = solution.cams.at(imgs[j]);
			StereoVision::Geometry::AffineTransform world2cam2(cam2toworld.R.transpose(),
									   - cam2toworld.R.transpose()*cam2toworld.t);

			StereoVision::Geometry::AffineTransform cam12cam2(world2cam2.R*cam12world.R,
									  world2cam2.t + world2cam2.R*cam12world.t);

			Eigen::Array2f ptCam1 = getHomogeneousImageCoordinates(im1, {pt});
			Eigen::Array2f ptCam2 = getHomogeneousImageCoordinates(im2, {pt});

			Eigen::Vector3f tmp = reprojectPoints(cam12cam2,
												  ptCam1,
												  ptCam2);

			coord += cam12world*tmp;
			n_obs++;

		}
	}

	coord /= n_obs;

	solution.points.emplace(pt, coord);

	return true;
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
