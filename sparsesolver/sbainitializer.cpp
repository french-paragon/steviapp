#include "sbainitializer.h"

#include "datablocks/project.h"
#include "datablocks/image.h"
#include "datablocks/landmark.h"
#include "datablocks/camera.h"

#include <eigen3/Eigen/Core>
#include <Eigen/Geometry>

#include <QSet>

#include <random>

namespace StereoVisionApp {


SBAInitializer::~SBAInitializer() {

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
		float py = (ilm->y().value() - cam_start->opticalCenterY().value())/cam_start->fLen().value();
		float pz = -1;

		r.points.insert(ilm->attachedLandmarkid(), {px, py, pz});
		presents.insert(ilm->attachedLandmarkid());

		float rx = (im_start->xRot().isSet()) ? im_start->xRot().value()/180.*M_PI : 0.;
		float ry = (im_start->yRot().isSet()) ? im_start->yRot().value()/180.*M_PI : 0.;
		float rz = (im_start->zRot().isSet()) ? im_start->zRot().value()/180.*M_PI : 0.;

		rot = Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX())
			* Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ());

		//float angle_conv = 180.*M_PI;
		r.cams.insert(im_start->internalId(), {Eigen::Vector3f(0,0,0),
											   R0});

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


			float cross_img = (selectedPt[1]->x().value() - selectedPt[0]->x().value())*(selectedPt[2]->y().value() - selectedPt[0]->y().value());
			cross_img -= (selectedPt[2]->x().value() - selectedPt[0]->x().value())*(selectedPt[1]->y().value() - selectedPt[0]->y().value());

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
					float i_h_y = (imlm->y().value() - currentCam->opticalCenterY().value() )/currentCam->fLen().value();

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
				float i_h_y = (imlm->y().value() - currentCam->opticalCenterY().value() )/currentCam->fLen().value();

				if (!isUp) {
					i_h_y = -i_h_y;
				}

				float tx = expf(x(0))*(i_h_x*cos(x(1)) + i_h_y*sin(x(1))) + x(2);
				float ty = expf(x(0))*(-i_h_x*sin(x(1)) + i_h_y*cos(x(1))) + x(3);
				float tz = -1;


				r.points.insert(ptId, {tx, ty, tz});
				presents.insert(ptId);

			}

			r.cams.insert(currentImg->internalId(), {Eigen::Vector3f(x(2), x(3), isUp ? 0.1f + 1/expf(x(0)) : -0.1f -1/expf(x(0))),
												   isUp ? R0 : Rx180});
			processedImgs.insert(currentImg->internalId());

		}

	} while (currentImg != nullptr);

	//apply initial cam rotation to all points

	for(Pt3D & pt : r.points) {
		pt = rot*pt;
	}

	for(Pt6D & cam : r.cams) {
		cam.t = rot*cam.t;
		cam.R = rot*cam.R;
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

		for(Pt3D & pt : r.points) {
			pt = s*pt + t;
		}

		for(Pt6D & cam : r.cams) {
			cam.t = s*cam.t + t;
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
		r.points.insert(id, Eigen::Vector3f(pDist(engine), pDist(engine), pDist(engine)));
	}

	for (qint64 id : s_imgs) {
		Eigen::Matrix3f rot = Eigen::AngleAxisf(azDist(engine), Eigen::Vector3f::UnitZ()).toRotationMatrix()
				* Eigen::AngleAxisf(latDist(engine), Eigen::Vector3f::UnitX()).toRotationMatrix();

		Eigen::Vector3f t(cDist(engine), cDist(engine), _camDist + cDist(engine));
		r.cams.insert(id, {rot*t, rot});
	}

	return r;

}

} // namespace StereoVisionApp
