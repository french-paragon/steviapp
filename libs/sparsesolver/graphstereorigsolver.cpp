#include "graphstereorigsolver.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/solver.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "datablocks/project.h"
#include "datablocks/landmark.h"
#include "datablocks/image.h"
#include "datablocks/camera.h"
#include "datablocks/stereorig.h"
#include "datablocks/angleconstrain.h"
#include "datablocks/distanceconstrain.h"

#include "g2o/types/sba/types_six_dof_expmap.h"
#include "vertices/vertexcamerapose.h"
#include "vertices/vertexcameraparam.h"

#include "edges/edgeparametrizedxyz2uv.h"
#include "edges/edgexyzprior.h"
#include "edges/edgese3fullprior.h"
#include "edges/edgese3rpyprior.h"
#include "edges/edgese3xyzprior.h"
#include "edges/edgepointdistance.h"
#include "edges/edgepointsangle.h"

#include "sbagraphreductor.h"
#include "sbainitializer.h"

#include <Eigen/Geometry>

namespace StereoVisionApp {

GraphStereoRigSolver::GraphStereoRigSolver(Project *p,
										   qint64 img1,
										   qint64 img2,
										   bool computeUncertainty,
										   bool sparse,
										   QObject *parent) :
	SparseSolverBase(p, parent),
	_sparse(sparse),
	_compute_marginals(computeUncertainty),
	_img1(img1),
	_img2(img2),
	_optimizer(nullptr)
{

}

GraphStereoRigSolver::~GraphStereoRigSolver() {
	cleanup();
}

int GraphStereoRigSolver::uncertaintySteps() const {
	return (_compute_marginals) ? 1 : 0;
}

bool GraphStereoRigSolver::hasUncertaintyStep() const {
	return _compute_marginals;
}

bool GraphStereoRigSolver::initialize(const InitialSolution *sol) {
	if (_currentProject == nullptr) {
		return false;
	}

	bool s = true;

	_optimizer = new g2o::SparseOptimizer();
	_optimizer->setVerbose(false);

	std::unique_ptr<lSolvType> linearSolver;
	if (_sparse) {
		linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >::PoseMatrixType>>();
	} else {
		linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >::PoseMatrixType>>();
	}

	g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<bSolvType>(std::move(linearSolver)));

	_optimizer->setAlgorithm(solver);

	Image* img1 = _currentProject->getDataBlock<Image>(_img1);
	Image* img2 = _currentProject->getDataBlock<Image>(_img2);

	if (img1 == nullptr or img2 == nullptr) {
		return false;
	}

	SBAGraphReductor::elementsSet selection;
	selection.imgs.insert(_img1);
	selection.imgs.insert(_img2);

	QVector<qint64> lms1 = img1->getAttachedLandmarksIds();
	QVector<qint64> lms2 = img2->getAttachedLandmarksIds();

	QSet<qint64> pts(lms1.begin(), lms1.end());
	QSet<qint64> tmp(lms2.begin(), lms2.end());

	pts.intersect(tmp);

	selection.pts = pts;

	if (selection.imgs.isEmpty() or selection.pts.isEmpty()) {
		return false;
	}

	int vid = 0;

	for (qint64 id : selection.pts) {
		Landmark* lm = qobject_cast<Landmark*>(_currentProject->getById(id));

		if (lm != nullptr) {

			landMarkVertex* v = new landMarkVertex();
			v->setId(vid++);
			v->setMarginalized(false);
			v->setReferedDatablock(lm);

			g2o::Vector3 d;

			if (sol != nullptr) {
				if (sol->points.count(id) > 0) {
					d = sol->points.at(id).cast<double>();
				} else {
					d.x() = lm->optPos().value(0);
					d.y() = lm->optPos().value(1);
					d.z() = lm->optPos().value(2);
				}
			} else {
				d.x() = lm->optPos().value(0);
				d.y() = lm->optPos().value(1);
				d.z() = lm->optPos().value(2);
			}

			v->setEstimate(d);

			_optimizer->addVertex(v);
			_landmarkVertices.insert(id, v);
		}
	}

	StereoRig* rig = nullptr;
	ImagePair* rigImgPair = nullptr;
	bool inverted_img_pair = false;

	QVector<qint64> rigs = _currentProject->getIdsByClass(StereoRig::staticMetaObject.className());

	for (qint64 id : rigs) {
		StereoRig* rg = _currentProject->getDataBlock<StereoRig>(id);

		if (rg != nullptr) {
			QVector<qint64> pairs = rg->listTypedSubDataBlocks(ImagePair::staticMetaObject.className());

			for (qint64 p_id : pairs) {
				ImagePair* p = qobject_cast<ImagePair*>(rg->getById(p_id));

				if (p != nullptr) {
					if (p->idImgCam1() == _img1 and p->idImgCam2() == _img2) {
						rig = rg;
						rigImgPair = p;
						break;
					}
					if (p->idImgCam1() == _img2 and p->idImgCam2() == _img1) {
						rig = rg;
						rigImgPair = p;
						inverted_img_pair = true;
						break;
					}
				}
			}
		}

		if (rig != nullptr) {
			break;
		}
	}

	for (qint64 id : selection.imgs) {
		Image* im = qobject_cast<Image*>(_currentProject->getById(id));

		if (im != nullptr) {

			qint64 cam_id = im->assignedCamera();
			Camera* c = qobject_cast<Camera*>(_currentProject->getById(cam_id));

			if (c == nullptr) {
				continue;
			}

			if (!_camVertices.contains(cam_id)) {

				VertexCameraFocal* cp_v = new VertexCameraFocal();

				cp_v->setId(vid++);
				cp_v->setMarginalized(false);
				cp_v->setReferedDatablock(c);

				Eigen::Vector2d extend;
				extend.x() = c->imSize().width();
				extend.y() = c->imSize().height();

				Eigen::Vector2d pp;
				pp.x() = c->opticalCenterX().value();
				pp.y() = c->opticalCenterY().value();

				cp_v->setEstimate(c->fLen().value());
				cp_v->setPP(pp);
				cp_v->setExtend(extend);

				_optimizer->addVertex(cp_v);

				_camVertices.insert(cam_id, cp_v);

			}

			VertexCameraPose* v = new VertexCameraPose();
			v->setId(vid++);
			v->setMarginalized(false);
			v->setReferedDatablock(im);

			Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
			Eigen::Vector3d t = Eigen::Vector3d::Zero();
			Eigen::Vector3d raxis;

			bool refCam = (inverted_img_pair) ? id == _img2 : id == _img1;

			if (!refCam) {

				raxis.x() = im->optRot().value(0);
				raxis.y() = im->optRot().value(1);
				raxis.z() = im->optRot().value(2);
				r = StereoVision::Geometry::rodriguezFormulaD(raxis);

				t.x() = im->optPos().value(0);
				t.y() = im->optPos().value(1);
				t.z() = im->optPos().value(2);
			}

			CameraPose p(r, t);

			v->setEstimate(p);

			if (refCam) {
				v->setFixed(true);
			}

			_optimizer->addVertex(v);
			_frameVertices.insert(id, v);

			if (!refCam and rig != nullptr) {

				r = (Eigen::AngleAxisd(rig->offsetRotX().value()/180*M_PI, Eigen::Vector3d::UnitX())*
						Eigen::AngleAxisd(rig->offsetRotY().value()/180*M_PI, Eigen::Vector3d::UnitY())*
						Eigen::AngleAxisd(rig->offsetRotZ().value()/180*M_PI, Eigen::Vector3d::UnitZ())).toRotationMatrix();

				t.x() = rig->offsetX().value();
				t.y() = rig->offsetY().value();
				t.z() = rig->offsetZ().value();

				if (rig->offsetX().isUncertain() and rig->offsetY().isUncertain() and rig->offsetZ().isUncertain() and
					rig->offsetRotX().isUncertain() and rig->offsetRotY().isUncertain() and rig->offsetRotZ().isUncertain()) {

					EdgeSE3FullPrior* e = new EdgeSE3FullPrior();

					CameraPose p_prior(r, t);

					e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );

					e->setMeasurement(p_prior);

					EdgeSE3FullPrior::InformationType info = EdgeSE3FullPrior::InformationType::Identity();
					info(0,0) = 1./(rig->offsetX().stddev()*rig->offsetX().stddev());
					info(1,1) = 1./(rig->offsetY().stddev()*rig->offsetY().stddev());
					info(2,2) = 1./(rig->offsetZ().stddev()*rig->offsetZ().stddev());
					info(3,3) = 1./(rig->offsetRotX().stddev()*rig->offsetRotX().stddev());
					info(4,4) = 1./(rig->offsetRotY().stddev()*rig->offsetRotY().stddev());
					info(5,5) = 1./(rig->offsetRotZ().stddev()*rig->offsetRotZ().stddev());

					e->setInformation(info);

					_optimizer->addEdge(e);

				} else {
					CameraPose p(r, t);
					v->setEstimate(p);
					v->setFixed(true);
				}

			}

			for (qint64 lmId : im->listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName)) {
				ImageLandmark* iml = im->getImageLandmark(lmId);

				if (iml != nullptr) {
					landMarkVertex* l_v = _landmarkVertices.value(iml->attachedLandmarkid(), nullptr);

					if (l_v == nullptr) {
						continue;
					}

					VertexCameraFocal* c_vs = _camVertices.value(cam_id);

					EdgeStereoRigXYZ2UV* e = new EdgeStereoRigXYZ2UV();
					e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );
					e->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex*>(l_v) );
					e->setVertex(2, static_cast<g2o::OptimizableGraph::Vertex*>(c_vs) );

					Eigen::Vector2d ptPos;
					ptPos.x() = iml->x().value();
					ptPos.y() = iml->y().value();

					e->setMeasurement(ptPos);

					Eigen::Matrix2d info = Eigen::Matrix2d::Identity();
					info(0,0) = (iml->x().isUncertain()) ? 1./(iml->x().stddev()*iml->x().stddev()) : 1;
					info(1,1) = (iml->y().isUncertain()) ? 1./(iml->y().stddev()*iml->y().stddev()) : 1;

					e->setInformation(info);

					_optimizer->addEdge(e);
				}
			}
		}
	}

	QVector<qint64> distConsts = _currentProject->getIdsByClass(DistanceConstrain::staticMetaObject.className());

	for (qint64 id : distConsts) {

		DistanceConstrain* constrain = _currentProject->getDataBlock<DistanceConstrain>(id);

		if (constrain == nullptr) {
			continue;
		}

		if (!constrain->distanceValue().isSet()) {
			continue; //not treating uset constraints for the moment TODO: treat them
		}

		QVector<qint64> lmPairs = constrain->listTypedSubDataBlocks(DistanceLandmarksPair::staticMetaObject.className());

		for (qint64 pair_id : lmPairs) {
			DistanceLandmarksPair* pair = constrain->getLandmarksPair(pair_id);

			if (pair == nullptr) {
				continue;
			}

			if (_landmarkVertices.contains(pair->getNthLandmarkId(0)) and
					_landmarkVertices.contains(pair->getNthLandmarkId(1))) {

				g2o::VertexPointXYZ* v1 = _landmarkVertices.value(pair->getNthLandmarkId(0), nullptr);
				g2o::VertexPointXYZ* v2 = _landmarkVertices.value(pair->getNthLandmarkId(1), nullptr);

				if (v1 != nullptr and v2 != nullptr) {

					floatParameter dist = constrain->distanceValue();

					EdgePointDistance* e = new EdgePointDistance();

					e->setMeasurement(dist.value());

					e->setVertex(0, v1);
					e->setVertex(1, v2);

					EdgePointDistance::InformationType info = EdgePointDistance::InformationType::Identity();
					info(0,0) = (dist.isUncertain()) ? 1./(dist.stddev()*dist.stddev()) : 1;

					e->setInformation(info);

					_optimizer->addEdge(e);
				}

			}
		}

	}

	QVector<qint64> angleConsts = _currentProject->getIdsByClass(AngleConstrain::staticMetaObject.className());

	for (qint64 id : angleConsts) {

		AngleConstrain* constrain = _currentProject->getDataBlock<AngleConstrain>(id);

		if (constrain == nullptr) {
			continue;
		}

		if (!constrain->angleValue().isSet()) {
			continue; //not treating uset constraints for the moment TODO: treat them
		}

		QVector<qint64> lmTriplets = constrain->listTypedSubDataBlocks(AngleLandmarksTriplets::staticMetaObject.className());

		for (qint64 pair_id : lmTriplets) {
			AngleLandmarksTriplets* triplet = constrain->getLandmarksTriplet(pair_id);

			if (triplet == nullptr) {
				continue;
			}

			if (_landmarkVertices.contains(triplet->getNthLandmarkId(0)) and
					_landmarkVertices.contains(triplet->getNthLandmarkId(1)) and
					_landmarkVertices.contains(triplet->getNthLandmarkId(2))) {

				g2o::VertexPointXYZ* v1 = _landmarkVertices.value(triplet->getNthLandmarkId(0), nullptr);
				g2o::VertexPointXYZ* v2 = _landmarkVertices.value(triplet->getNthLandmarkId(1), nullptr);
				g2o::VertexPointXYZ* v3 = _landmarkVertices.value(triplet->getNthLandmarkId(2), nullptr);

				if (v1 != nullptr and v2 != nullptr and v3 != nullptr) {

					floatParameter angle = constrain->angleValue();

					EdgePointsAngle* e = new EdgePointsAngle();

					e->setMeasurement(angle.value()/180.0 * M_PI);

					e->setVertex(0, v1);
					e->setVertex(1, v2);
					e->setVertex(2, v3);

					EdgePointDistance::InformationType info = EdgePointDistance::InformationType::Identity();
					float stddev_angle = angle.stddev()/180. * M_PI;
					info(0,0) = (angle.isUncertain()) ? 1./(stddev_angle*stddev_angle) : 1;

					e->setInformation(info);

					_optimizer->addEdge(e);
				}

			}
		}

	}

	s = _optimizer->initializeOptimization();
	_not_first_step = false;

	_optimizer->setVerbose(true);

	return s;
}
bool GraphStereoRigSolver::runSteps(int pn_steps) {

	int n_steps = _optimizer->optimize(pn_steps, _not_first_step);
	//int n_steps = _optimizer->optimize(optimizationSteps());
	_not_first_step = true;
	return n_steps > 0;
}

std::optional<Eigen::Vector3f> GraphStereoRigSolver::getLandmarkOptimizedPos(qint64 id) const {
	if (_landmarkVertices.contains(id)) {
		return _landmarkVertices[id]->estimate().cast<float>();
	}
	return std::nullopt;
}
std::optional<InitialSolution::Pt6D> GraphStereoRigSolver::getOptimizedViewPose(qint64 id) const {
	if (_frameVertices.contains(id)) {
		CameraPose p = _frameVertices[id]->estimate();
		InitialSolution::Pt6D r(p.r().cast<float>(), p.t().cast<float>());
		return r;
	}
	return std::nullopt;
}
std::optional<float> GraphStereoRigSolver::getOptimizedCamFocal(qint64 id) const {
	if (_camVertices.contains(id)) {
		return _camVertices[id]->estimate();
	}
	return std::nullopt;
}

InitialSolution GraphStereoRigSolver::resultToSolution() const {

	InitialSolution s;

	for (qint64 lm_id : _landmarkVertices.keys()) {
		auto val = getLandmarkOptimizedPos(lm_id);
		if (val.has_value()) {
			s.points[lm_id] = val.value();
		}
	}


	for (qint64 im_id : _frameVertices.keys()) {
		auto val = getOptimizedViewPose(im_id);
		if (val.has_value()) {
			s.cams[im_id] = val.value();
		}
	}

	return s;

}

bool GraphStereoRigSolver::init() {
	return initialize();
}

bool GraphStereoRigSolver::opt_step() {
	return runSteps(1);
}

bool GraphStereoRigSolver::std_step() {

	if (_currentProject == nullptr or !_compute_marginals) {
		return false;
	}

	if (_sparse) {

		std::vector<g2o::OptimizableGraph::Vertex*> marginals_computation_vertices;

		marginals_computation_vertices.reserve(_landmarkVertices.size() + _frameVertices.size() + _camVertices.size()*CameraInnerVertexCollection::VerticesPerCam);

		for (landMarkVertex* v : _landmarkVertices) {
			marginals_computation_vertices.push_back(v);
		}
		for (VertexCameraPose* v : _frameVertices) {
			marginals_computation_vertices.push_back(v);
		}
		for (qint64 id : _camVertices.keys()) {
			VertexCameraFocal* c = _camVertices[id];

			marginals_computation_vertices.push_back(c);
		}

		bool r =  _optimizer->computeMarginals(_spinv, marginals_computation_vertices);

		return r;

	} else {
		//dense solver do not allows direct access to Hessian and/or maginals
		//TODO write helper function to copy hessian from vertices to dense matrix (which can be inverted by hand).

		return false;
	}
}

bool GraphStereoRigSolver::writeResults() {

	if (_currentProject == nullptr) {
		return false;
	}

	for (qint64 id : _landmarkVertices.keys()) {

		Landmark* lm = qobject_cast<Landmark*>(_currentProject->getById(id));

		landMarkVertex* lmv = _landmarkVertices.value(id);

		const landMarkVertex::EstimateType & e = lmv->estimate();

		floatParameterGroup<3> pos;
		pos.value(0) = static_cast<float>(e.x());
		pos.value(1) = static_cast<float>(e.y());
		pos.value(2) = static_cast<float>(e.z());
		pos.setIsSet();
		lm->setOptPos(pos);

	}

	for (qint64 id : _frameVertices.keys()) {

		Image* im = qobject_cast<Image*>(_currentProject->getById(id));

		VertexCameraPose* imv = _frameVertices.value(id);

		const CameraPose & e = imv->estimate();

		Eigen::Vector3d t = e.t();
		Eigen::Vector3d r = StereoVision::Geometry::inverseRodriguezFormulaD(e.r());

		floatParameterGroup<3> pos;
		pos.value(0) = static_cast<float>(t.x());
		pos.value(1) = static_cast<float>(t.y());
		pos.value(2) = static_cast<float>(t.z());
		pos.setIsSet();
		im->setOptPos(pos);

		floatParameterGroup<3> rot;
		rot.value(0) = static_cast<float>(r.x());
		rot.value(1) = static_cast<float>(r.y());
		rot.value(2) = static_cast<float>(r.z());
		rot.setIsSet();
		im->setOptRot(rot);

	}

	for (qint64 id : _camVertices.keys()) {

		Camera* cam = qobject_cast<Camera*>(_currentProject->getById(id));
		cam->clearOptimized();

		VertexCameraFocal* camv = _camVertices[id];

		double f = camv->estimate();

		cam->setOptimizedFLen(static_cast<float>(f));

	}

	return true;

}
bool GraphStereoRigSolver::writeUncertainty() {

	if (_currentProject == nullptr or !_compute_marginals) {
		return false;
	}

	for (qint64 id : _landmarkVertices.keys()) {

		Landmark* lm = qobject_cast<Landmark*>(_currentProject->getById(id));

		landMarkVertex* lmv = _landmarkVertices.value(id);

		Eigen::MatrixXd* pm = _spinv.block(lmv->hessianIndex(), lmv->hessianIndex());

		if (pm == nullptr) {
			return false;
		}

		Eigen::MatrixXd const& m = *pm;

		if (m.rows() != 3 or m.cols() != 3) {
			return false;
		}

		floatParameterGroup<3> pos = lm->optPos();

		pos.stddev(0, 0) = m(0,0);
		pos.stddev(1, 0) = m(1,0);
		pos.stddev(2, 0) = m(2,0);
		pos.stddev(1, 1) = m(1,1);
		pos.stddev(2, 1) = m(2,1);
		pos.stddev(2, 2) = m(2,2);
		pos.setUncertain();

		lm->setOptPos(pos);

	}

	for (qint64 id : _frameVertices.keys()) {

		Image* im = qobject_cast<Image*>(_currentProject->getById(id));

		VertexCameraPose* imv = _frameVertices.value(id);

		Eigen::MatrixXd* pm = _spinv.block(imv->hessianIndex(), imv->hessianIndex());

		if (pm == nullptr) {
			return false;
		}

		Eigen::MatrixXd const& m = *pm;

		if (m.rows() != 6 or m.cols() != 6) {
			return false;
		}

		floatParameterGroup<3> pos = im->optPos();

		pos.stddev(0, 0) = m(0,0);
		pos.stddev(1, 0) = m(1,0);
		pos.stddev(2, 0) = m(2,0);
		pos.stddev(1, 1) = m(1,1);
		pos.stddev(2, 1) = m(2,1);
		pos.stddev(2, 2) = m(2,2);
		pos.setUncertain();

		im->setOptPos(pos);

		floatParameterGroup<3> rot = im->optRot();

		rot.stddev(0, 0) = m(3,3);
		rot.stddev(1, 0) = m(4,3);
		rot.stddev(2, 0) = m(5,3);
		rot.stddev(1, 1) = m(4,4);
		rot.stddev(2, 1) = m(5,4);
		rot.stddev(2, 2) = m(5,5);
		rot.setUncertain();

		im->setOptRot(rot);

	}

	for (qint64 id : _camVertices.keys()) {

		Camera* cam = qobject_cast<Camera*>(_currentProject->getById(id));
		cam->clearOptimized();

		VertexCameraFocal* camv = _camVertices[id];


		Eigen::MatrixXd* pm = _spinv.block(camv->hessianIndex(), camv->hessianIndex());

		if (pm == nullptr) {
			return false;
		}

		Eigen::MatrixXd const& m = *pm;

		if (m.rows() != 3 or m.cols() != 3) {
			return false;
		}

		floatParameter f = cam->optimizedFLen();

		f.setUncertainty(m(0,0));

		cam->setOptimizedFLen(f);

	}

	return true;
}
void GraphStereoRigSolver::cleanup() {

	_landmarkVertices.clear();
	_camVertices.clear();
	_frameVertices.clear();

	_spinv.clear(true);

	if (_optimizer != nullptr) {
		delete _optimizer;
		_optimizer = nullptr;
	}
}

bool GraphStereoRigSolver::splitOptSteps() const {
	return true;
}

} // namespace StereoVisionApp

