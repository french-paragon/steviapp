#include "graphsbasolver.h"

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
#include "datablocks/angleconstrain.h"
#include "datablocks/distanceconstrain.h"
#include "datablocks/stereorig.h"
#include "datablocks/localcoordinatesystem.h"

#include "g2o/types/sba/types_six_dof_expmap.h"
#include "vertices/vertexcamerapose.h"
#include "vertices/vertexcameraparam.h"
#include "vertices/vertexrigidbodypose.h"

#include "edges/edgeparametrizedxyz2uv.h"
#include "edges/edgexyzprior.h"
#include "edges/edgese3fullprior.h"
#include "edges/edgese3rpyprior.h"
#include "edges/edgese3xyzprior.h"
#include "edges/edgepointdistance.h"
#include "edges/edgepointsangle.h"
#include "edges/edgecamerase3leverarm.h"
#include "edges/edgelocalpointcoordinates.h"

#include "sbagraphreductor.h"
#include "sbainitializer.h"

#include <Eigen/Geometry>

namespace StereoVisionApp {

GraphSBASolver::GraphSBASolver(Project *p, bool computeUncertainty, bool sparse, QObject *parent) :
	SparseSolverBase(p, parent),
	_sparse(sparse),
	_compute_marginals(computeUncertainty),
	_optimizer(nullptr)
{

}

GraphSBASolver::~GraphSBASolver() {
	cleanup();
}

int GraphSBASolver::uncertaintySteps() const {
	return (_compute_marginals) ? 1 : 0;
}

bool GraphSBASolver::hasUncertaintyStep() const {
	return _compute_marginals;
}

bool GraphSBASolver::init() {

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

	SBAGraphReductor selector(3,2,true,true);

	SBAGraphReductor::elementsSet selection = selector(_currentProject, false);

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

			d.x() = lm->optPos().value(0);
			d.y() = lm->optPos().value(1);
			d.z() = lm->optPos().value(2);

			v->setEstimate(d);

			_optimizer->addVertex(v);
			_landmarkVertices.insert(id, v);

			if (lm->xCoord().isSet() and lm->yCoord().isSet() and lm->zCoord().isSet()) {

				EdgeXyzPrior* e = new EdgeXyzPrior();

				e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );

				Eigen::Vector3d m;
				m.x() = lm->xCoord().value();
				m.y() = lm->yCoord().value();
				m.z() = lm->zCoord().value();

				e->setMeasurement(m);

				Eigen::Matrix3d info = Eigen::Matrix3d::Identity();

				if (lm->xCoord().isUncertain()) {
					info(0,0) = 1./(lm->xCoord().stddev()*lm->xCoord().stddev());
				} else {
					info(0,0) = 1e6;
				}

				if (lm->yCoord().isUncertain()) {
					info(1,1) = 1./(lm->yCoord().stddev()*lm->yCoord().stddev());
				} else {
					info(1,1) = 1e6;
				}

				if (lm->zCoord().isUncertain()) {
					info(2,2) = 1./(lm->zCoord().stddev()*lm->zCoord().stddev());
				} else {
					info(2,2) = 1e6;
				}

				e->setInformation(info);

				_optimizer->addEdge(e);

			}
		}
	}

	for (qint64 id : selection.imgs) {
		Image* im = qobject_cast<Image*>(_currentProject->getById(id));

		if (im != nullptr) {

			qint64 cam_id = setupCameraVertexForImage(im, vid);

			if (cam_id < 0) {
				continue;
			}

			VertexCameraPose* v = new VertexCameraPose();
			v->setId(vid++);
			v->setMarginalized(false);
			v->setReferedDatablock(im);

			Eigen::Matrix3d r;
			Eigen::Vector3d t;
			Eigen::Vector3d raxis;

			raxis.x() = im->optRot().value(0);
			raxis.y() = im->optRot().value(1);
			raxis.z() = im->optRot().value(2);
			r = StereoVision::Geometry::rodriguezFormula(raxis);

			t.x() = im->optPos().value(0);
			t.y() = im->optPos().value(1);
			t.z() = im->optPos().value(2);

			CameraPose p(r, t);

			v->setEstimate(p);

			if (getFixedParametersFlag()&FixedParameter::CameraExternal) {
				v->setFixed(true);
			}

			_optimizer->addVertex(v);
			_frameVertices.insert(id, v);

			//TODO: check if the use of axis angle representation here is consistent.
			if (im->xRot().isSet() and im->yRot().isSet() and im->zRot().isSet()) {
				raxis.x() = im->xRot().value();
				raxis.y() = im->yRot().value();
				raxis.z() = im->zRot().value();
				r = StereoVision::Geometry::rodriguezFormula(raxis);
			}

			if (im->xCoord().isSet() and im->yCoord().isSet() and im->zCoord().isSet()) {
				t.x() = im->xCoord().value();
				t.y() = im->yCoord().value();
				t.z() = im->zCoord().value();
			}

			if (im->isFixed()) {

				CameraPose p_set(r, t);
				v->setEstimate(p_set);
				v->setFixed(true);

			} else if (im->xCoord().isUncertain() and im->yCoord().isUncertain() and im->zCoord().isUncertain() and
				im->xRot().isUncertain() and im->yRot().isUncertain() and im->zRot().isUncertain()) {

				EdgeSE3FullPrior* e = new EdgeSE3FullPrior();

				CameraPose p_prior(r, t);

				e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );

				e->setMeasurement(p_prior);

				EdgeSE3FullPrior::InformationType info = EdgeSE3FullPrior::InformationType::Identity();
				info(0,0) = 1./(im->xCoord().stddev()*im->xCoord().stddev());
				info(1,1) = 1./(im->yCoord().stddev()*im->yCoord().stddev());
				info(2,2) = 1./(im->zCoord().stddev()*im->zCoord().stddev());
				info(3,3) = 1./(im->xRot().stddev()*im->xRot().stddev());
				info(4,4) = 1./(im->yRot().stddev()*im->yRot().stddev());
				info(5,5) = 1./(im->zRot().stddev()*im->zRot().stddev());

				e->setInformation(info);

				_optimizer->addEdge(e);

			} else if (im->xCoord().isUncertain() and im->yCoord().isUncertain() and im->zCoord().isUncertain()) {

				EdgeSE3xyzPrior* e = new EdgeSE3xyzPrior();

				e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );

				e->setMeasurement(t);

				EdgeSE3xyzPrior::InformationType info = EdgeSE3xyzPrior::InformationType::Identity();
				info(0,0) = 1./(im->xCoord().stddev()*im->xCoord().stddev());
				info(1,1) = 1./(im->yCoord().stddev()*im->yCoord().stddev());
				info(2,2) = 1./(im->zCoord().stddev()*im->zCoord().stddev());

				e->setInformation(info);

				_optimizer->addEdge(e);

			} else if (im->xRot().isUncertain() and im->yRot().isUncertain() and im->zRot().isUncertain()) {

				EdgeSE3rpyPrior* e = new EdgeSE3rpyPrior();

				e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );

				e->setMeasurement(r);

				EdgeSE3rpyPrior::InformationType info = EdgeSE3rpyPrior::InformationType::Identity();
				info(0,0) = 1./(im->xRot().stddev()*im->xRot().stddev());
				info(1,1) = 1./(im->yRot().stddev()*im->yRot().stddev());
				info(2,2) = 1./(im->zRot().stddev()*im->zRot().stddev());

				e->setInformation(info);

				_optimizer->addEdge(e);

			}

			for (qint64 lmId : im->listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName)) {
				ImageLandmark* iml = im->getImageLandmark(lmId);

				if (iml != nullptr) {
					landMarkVertex* l_v = _landmarkVertices.value(iml->attachedLandmarkid(), nullptr);

					if (l_v == nullptr) {
						continue;
					}

					const CameraInnerVertexCollection& c_vs = _camVertices.value(cam_id);

					EdgeParametrizedXYZ2UV* e = new EdgeParametrizedXYZ2UV();
					e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );
					e->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex*>(l_v) );
					e->setVertex(2, static_cast<g2o::OptimizableGraph::Vertex*>(c_vs.param) );
					e->setVertex(3, static_cast<g2o::OptimizableGraph::Vertex*>(c_vs.radialDist) );
					e->setVertex(4, static_cast<g2o::OptimizableGraph::Vertex*>(c_vs.tangeantialDist) );
					e->setVertex(5, static_cast<g2o::OptimizableGraph::Vertex*>(c_vs.skewDist) );

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

	/* distance constraints */

	QVector<qint64> distConsts = _currentProject->getIdsByClass(DistanceConstrain::staticMetaObject.className());

	for (qint64 id : distConsts) {

		DistanceConstrain* constrain = _currentProject->getDataBlock<DistanceConstrain>(id);

		if (constrain == nullptr) {
			continue;
		}

		if (!constrain->isEnabled()) {
			continue;
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

	/* angle constraints */

	QVector<qint64> angleConsts = _currentProject->getIdsByClass(AngleConstrain::staticMetaObject.className());

	for (qint64 id : angleConsts) {

		AngleConstrain* constrain = _currentProject->getDataBlock<AngleConstrain>(id);

		if (constrain == nullptr) {
			continue;
		}

		if (!constrain->isEnabled()) {
			continue;
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

	//stereo rigs
	/*
	QVector<qint64> rigs = _currentProject->getIdsByClass(StereoRig::staticMetaObject.className());

	for (qint64 id : rigs) {

		if (!(_fixedParameters&FixedParameter::StereoRigs)) {
			break; //not considering stereo rigs for optimization at the moment
		}

		StereoRig* rg = _currentProject->getDataBlock<StereoRig>(id);

		if (rg == nullptr) {
			continue;
		}

		if (!rg->isEnabled()) {
			continue;
		}

		bool hasMeasure = true;
		hasMeasure = hasMeasure and rg->optPos().isSet();
		hasMeasure = hasMeasure and rg->optRot().isSet();

		if (!hasMeasure) {
			continue;
		}

		Eigen::Vector3d t;
		t.x() = rg->optXCoord();
		t.y() = rg->optYCoord();
		t.z() = rg->optZCoord();

		float eX = rg->optXRot();
		float eY = rg->optYRot();
		float eZ = rg->optZRot();

		Eigen::Matrix3d R = StereoVision::Geometry::rodriguezFormulaD(Eigen::Vector3d(eX, eY, eZ));

		CameraPose cam2tocam1Prior(R, t);

		QVector<qint64> pairs = rg->listTypedSubDataBlocks(ImagePair::staticMetaObject.className());
		QVector<ImagePair*> pairsInCalibration;
		pairsInCalibration.reserve(pairs.size());

		for (qint64 p_id : pairs) {
			ImagePair* p = qobject_cast<ImagePair*>(rg->getById(p_id));

			if (p == nullptr) {
				continue;
			}

			if (_frameVertices.contains(p->idImgCam1()) and _frameVertices.contains(p->idImgCam2())) {

				pairsInCalibration.push_back(p);

			} else if (_frameVertices.contains(p->idImgCam1())) {

				VertexCameraPose* vc1 = _frameVertices.value(p->idImgCam1());

				qint64 idc2 = p->idImgCam2();
				Image* img2 = _currentProject->getDataBlock<Image>(idc2);

				if (img2 == nullptr) {
					continue;
				}

				VertexCameraPose* v = new VertexCameraPose();
				v->setId(vid++);
				v->setMarginalized(false);
				v->setReferedDatablock(img2);

				CameraPose Cam1ToWorld = vc1->estimate();
				CameraPose Cam2ToWorldPrior = Cam1ToWorld.applySE3(cam2tocam1Prior);

				v->setEstimate(Cam2ToWorldPrior);

				if (getFixedParametersFlag()&FixedParameter::CameraExternal) {
					v->setFixed(true);
				}

				_optimizer->addVertex(v);
				_frameVertices.insert(idc2, v);

			} else if (_frameVertices.contains(p->idImgCam2())) {

				VertexCameraPose* vc2 = _frameVertices.value(p->idImgCam2());

				qint64 idc1 = p->idImgCam1();
				Image* img1 = _currentProject->getDataBlock<Image>(idc1);

				if (img1 == nullptr) {
					continue;
				}

				VertexCameraPose* v = new VertexCameraPose();
				v->setId(vid++);
				v->setMarginalized(false);
				v->setReferedDatablock(img1);

				CameraPose Cam2ToWorld = vc2->estimate();
				CameraPose Cam1ToWorldPrior = Cam2ToWorld.applySE3(cam2tocam1Prior.inverseSE3());

				v->setEstimate(Cam1ToWorldPrior);

				if (getFixedParametersFlag()&FixedParameter::CameraExternal) {
					v->setFixed(true);
				}

				_optimizer->addVertex(v);
				_frameVertices.insert(idc1, v);

			}
		}

		if (pairsInCalibration.isEmpty()) {
			continue;
		}

		for (ImagePair* imp : pairsInCalibration) {

			VertexCameraPose* vcam1 = _frameVertices.value(imp->idImgCam1());
			VertexCameraPose* vcam2 = _frameVertices.value(imp->idImgCam2());

			EdgeCameraSE3LeverArm* e = new EdgeCameraSE3LeverArm();
			e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(vcam1));
			e->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex*>(vcam2));

			e->setMeasurement(cam2tocam1Prior);

			if (rg->optPos().isUncertain() and rg->optRot().isUncertain()) {

				EdgeCameraSE3LeverArm::InformationType info = EdgeCameraSE3LeverArm::InformationType::Identity();
				info(0,0) = 1./(rg->optPos().stddev(0,0)*rg->optPos().stddev(0,0));
				info(0,0) = 1./(rg->optPos().stddev(1,1)*rg->optPos().stddev(1,1));
				info(2,2) = 1./(rg->optPos().stddev(2,2)*rg->optPos().stddev(2,2));
				info(3,3) = 1./(rg->optRot().stddev(0,0)*rg->optRot().stddev(0,0));
				info(4,4) = 1./(rg->optRot().stddev(1,1)*rg->optRot().stddev(1,1));
				info(5,5) = 1./(rg->optRot().stddev(2,2)*rg->optRot().stddev(2,2));

				e->setInformation(info);

			} else {
				e->setInformation(EdgeCameraSE3LeverArm::InformationType::Identity());
			}

			_optimizer->addEdge(e);

		}

	}*/

	// Local coordinate systems

	for (qint64 id : selection.localcoordsysts) {
		LocalCoordinateSystem* lcs = _currentProject->getDataBlock<LocalCoordinateSystem>(id);

		if (lcs != nullptr) {

			VertexRigidBodyPose* v = new VertexRigidBodyPose();
			v->setId(vid++);
			v->setMarginalized(false);
			v->setReferedDatablock(lcs);

			Eigen::Matrix3d r;
			Eigen::Vector3d t;
			Eigen::Vector3d raxis;

			r = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

			t.x() = 0;
			t.y() = 0;
			t.z() = 1;

			if (lcs->optPos().isSet() and lcs->optRot().isSet()) {

				raxis.x() = lcs->optRot().value(0);
				raxis.y() = lcs->optRot().value(1);
				raxis.z() = lcs->optRot().value(2);
				r = StereoVision::Geometry::rodriguezFormula(raxis);

				t.x() = lcs->optPos().value(0);
				t.y() = lcs->optPos().value(1);
				t.z() = lcs->optPos().value(2);
			}

			CameraPose p(r, t); //assume NADIR hypothesis with camera facing down.

			v->setEstimate(p);

			_optimizer->addVertex(v);
			_localCoordinatesVertices.insert(id, v);

			if (lcs->isFixed()) {
				v->setFixed(true);

			}

			for (qint64 lmId : lcs->listTypedSubDataBlocks(LandmarkLocalCoordinates::staticMetaObject.className())) {
				LandmarkLocalCoordinates* lmlc = lcs->getLandmarkLocalCoordinates(lmId);

				if (lmlc == nullptr) {
					continue;
				}

				if (!lmlc->xCoord().isSet() or !lmlc->yCoord().isSet() or !lmlc->zCoord().isSet()) {
					continue;
				}

				landMarkVertex* l_v = _landmarkVertices.value(lmlc->attachedLandmarkid(), nullptr);

				if (l_v == nullptr) {
					continue;
				}

				EdgeLocalPointCoordinates* e = new EdgeLocalPointCoordinates();
				e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );
				e->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex*>(l_v) );

				Eigen::Vector3d localPos;
				localPos.x() = lmlc->xCoord().value();
				localPos.y() = lmlc->yCoord().value();
				localPos.z() = lmlc->zCoord().value();
				e->setMeasurement(localPos);

				e->setInformation(Eigen::Matrix3d::Identity());

				_optimizer->addEdge(e);
			}
		}
	}

	s = _optimizer->initializeOptimization();
	_not_first_step = false;

	_optimizer->setVerbose(true);

	return s;
}

bool GraphSBASolver::opt_step() {
	int n_steps = _optimizer->optimize(1, _not_first_step);
	//int n_steps = _optimizer->optimize(optimizationSteps());
	_not_first_step = true;
	return n_steps > 0;
}

bool GraphSBASolver::std_step() {

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
			CameraInnerVertexCollection const& c = _camVertices[id];

			marginals_computation_vertices.push_back(c.param);

			Camera* cam = qobject_cast<Camera*>(_currentProject->getById(id));

			if (cam->useRadialDistortionModel()) {
				marginals_computation_vertices.push_back(c.radialDist);
			}
			if (cam->useTangentialDistortionModel()) {
				marginals_computation_vertices.push_back(c.tangeantialDist);
			}
			if (cam->useSkewDistortionModel()) {
				marginals_computation_vertices.push_back(c.skewDist);
			}
		}

		bool r =  _optimizer->computeMarginals(_spinv, marginals_computation_vertices);

		return r;

	} else {
		//dense solver do not allows direct access to Hessian and/or maginals
		//TODO write helper function to copy hessian from vertices to dense matrix (which can be inverted by hand).

		return false;
	}
}

bool GraphSBASolver::writeResults() {

	if (_currentProject == nullptr) {
		return false;
	}

	for (qint64 id : _landmarkVertices.keys()) {

		Landmark* lm = qobject_cast<Landmark*>(_currentProject->getById(id));

		if (lm->isFixed()) {
			continue;
		}

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

		if (im->isFixed()) {
			continue;
		}

		VertexCameraPose* imv = _frameVertices.value(id);

		const CameraPose & e = imv->estimate();

		Eigen::Vector3d t = e.t();
		Eigen::Vector3d r = StereoVision::Geometry::inverseRodriguezFormula(e.r());

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

		if (cam->isFixed()) {
			continue;
		}

		cam->clearOptimized();

		const CameraInnerVertexCollection & camv = _camVertices[id];

		CamParam e = camv.param->estimate();

		cam->setOptimizedFLen(static_cast<float>(e.f()));
		cam->setOptimizedOpticalCenterX(static_cast<float>(e.pp().x()));
		cam->setOptimizedOpticalCenterY(static_cast<float>(e.pp().y()));

		if (cam->useRadialDistortionModel()) {
			Eigen::Vector3d ke = camv.radialDist->estimate();

			cam->setOptimizedK1(static_cast<float>(ke.x()));
			cam->setOptimizedK2(static_cast<float>(ke.y()));
			cam->setOptimizedK3(static_cast<float>(ke.z()));
		}

		if (cam->useTangentialDistortionModel()) {
			Eigen::Vector2d pe = camv.tangeantialDist->estimate();

			cam->setOptimizedP1(static_cast<float>(pe.x()));
			cam->setOptimizedP2(static_cast<float>(pe.y()));
		}

		if (cam->useSkewDistortionModel()) {
			Eigen::Vector2d be = camv.skewDist->estimate();

			cam->setOptimizedB1(static_cast<float>(be.x()));
			cam->setOptimizedB2(static_cast<float>(be.y()));
		}

	}

	for (qint64 id : _localCoordinatesVertices.keys()) {

		LocalCoordinateSystem* lcs = _currentProject->getDataBlock<LocalCoordinateSystem>(id);

		if (lcs->isFixed()) {
			continue;
		}

		VertexRigidBodyPose* lcrb = _localCoordinatesVertices.value(id);

		const CameraPose & e = lcrb->estimate();

		Eigen::Vector3d t = e.t();
		Eigen::Vector3d r = StereoVision::Geometry::inverseRodriguezFormula(e.r());

		floatParameterGroup<3> pos;
		pos.value(0) = static_cast<float>(t.x());
		pos.value(1) = static_cast<float>(t.y());
		pos.value(2) = static_cast<float>(t.z());
		pos.setIsSet();
		lcs->setOptPos(pos);

		floatParameterGroup<3> rot;
		rot.value(0) = static_cast<float>(r.x());
		rot.value(1) = static_cast<float>(r.y());
		rot.value(2) = static_cast<float>(r.z());
		rot.setIsSet();
		lcs->setOptRot(rot);

	}

	return true;

}
bool GraphSBASolver::writeUncertainty() {

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

		const CameraInnerVertexCollection & camv = _camVertices[id];


		Eigen::MatrixXd* pm = _spinv.block(camv.param->hessianIndex(), camv.param->hessianIndex());

		if (pm == nullptr) {
			return false;
		}

		Eigen::MatrixXd const& m = *pm;

		if (m.rows() != 3 or m.cols() != 3) {
			return false;
		}

		floatParameter f = cam->optimizedFLen();
		floatParameter ppx = cam->optimizedOpticalCenterX();
		floatParameter ppy = cam->optimizedOpticalCenterY();

		f.setUncertainty(m(0,0));
		ppx.setUncertainty(m(1,1));
		ppy.setUncertainty(m(2,2));

		cam->setOptimizedFLen(f);
		cam->setOptimizedOpticalCenterX(ppx);
		cam->setOptimizedOpticalCenterY(ppy);

		if (cam->useRadialDistortionModel()) {
			pm = _spinv.block(camv.radialDist->hessianIndex(), camv.radialDist->hessianIndex());

			if (pm != nullptr) {

				Eigen::MatrixXd const& km = *pm;

				if (km.rows() == 3 and km.cols() == 3) {

					floatParameter k1 = cam->optimizedK1();
					floatParameter k2 = cam->optimizedK2();
					floatParameter k3 = cam->optimizedK3();

					k1.setUncertainty(km(0,0));
					k2.setUncertainty(km(1,1));
					k3.setUncertainty(km(2,2));

					cam->setOptimizedK1(k1);
					cam->setOptimizedK2(k2);
					cam->setOptimizedK3(k3);
				}
			}
		}

		if (cam->useTangentialDistortionModel()) {
			pm = _spinv.block(camv.tangeantialDist->hessianIndex(), camv.tangeantialDist->hessianIndex());

			if (pm != nullptr) {

				Eigen::MatrixXd const& tm = *pm;

				if (tm.rows() == 2 and tm.cols() == 2) {

					floatParameter p1 = cam->optimizedP1();
					floatParameter p2 = cam->optimizedP2();

					p1.setUncertainty(tm(0,0));
					p2.setUncertainty(tm(1,1));

					cam->setOptimizedP1(p1);
					cam->setOptimizedP2(p2);
				}
			}
		}

		if (cam->useSkewDistortionModel()) {
			pm = _spinv.block(camv.skewDist->hessianIndex(), camv.skewDist->hessianIndex());

			if (pm != nullptr) {

				Eigen::MatrixXd const& Bm = *pm;

				if (Bm.rows() == 2 and Bm.cols() == 2) {

					floatParameter B1 = cam->optimizedB1();
					floatParameter B2 = cam->optimizedB2();

					B1.setUncertainty(Bm(0,0));
					B2.setUncertainty(Bm(1,1));

					cam->setOptimizedB1(B1);
					cam->setOptimizedB2(B2);
				}
			}
		}

	}

	return true;
}
void GraphSBASolver::cleanup() {

	_landmarkVertices.clear();
	_camVertices.clear();
	_frameVertices.clear();

	_spinv.clear(true);

	if (_optimizer != nullptr) {
		delete _optimizer;
		_optimizer = nullptr;
	}
}

bool GraphSBASolver::splitOptSteps() const {
	return true;
}

qint64 GraphSBASolver::setupCameraVertexForImage(Image* im, int & vid) {

	if (im == nullptr) {
		return -1;
	}

	qint64 cam_id = im->assignedCamera();
	Camera* c = qobject_cast<Camera*>(_currentProject->getById(cam_id));

	if (c == nullptr) {
		return -1;
	}

	if (!_camVertices.contains(cam_id)) {

		VertexCameraParam* cp_v = new VertexCameraParam();
		VertexCameraRadialDistortion* rd_v = new VertexCameraRadialDistortion();
		VertexCameraTangentialDistortion* td_v = new VertexCameraTangentialDistortion();
		VertexCameraSkewDistortion* sd_v = new VertexCameraSkewDistortion();

		cp_v->setId(vid++);
		rd_v->setId(vid++);
		td_v->setId(vid++);
		sd_v->setId(vid++);

		cp_v->setMarginalized(false);
		rd_v->setMarginalized(false);
		td_v->setMarginalized(false);
		sd_v->setMarginalized(false);

		cp_v->setReferedDatablock(c);
		rd_v->setReferedDatablock(c);
		td_v->setReferedDatablock(c);
		sd_v->setReferedDatablock(c);

		Eigen::Vector2d extend;
		extend.x() = c->imSize().width();
		extend.y() = c->imSize().height();

		Eigen::Vector2d pp;

		if (c->optimizedOpticalCenterX().isSet() and
				c->optimizedOpticalCenterY().isSet()) {

			pp.x() = c->optimizedOpticalCenterX().value();
			pp.y() = c->optimizedOpticalCenterY().value();

		} else {

			pp.x() = c->opticalCenterX().value();
			pp.y() = c->opticalCenterY().value();
		}

		float flen;

		if (c->optimizedFLen().isSet()) {
			flen = c->optimizedFLen().value();
		} else {
			flen = c->fLen().value();
		}

		CamParam cp(extend, pp, flen);

		cp_v->setEstimate(cp);

		if (c->isFixed() or getFixedParametersFlag()&FixedParameter::CameraInternal) {
			cp_v->setFixed(true);
		}

		Eigen::Vector3d rs;

		if (c->optimizedK1().isSet() and
				c->optimizedK2().isSet() and
				c->optimizedK3().isSet()) {

			rs.x() = c->optimizedK1().value();
			rs.y() = c->optimizedK2().value();
			rs.z() = c->optimizedK3().value();

		} else {

			rs.x() = c->k1().value();
			rs.y() = c->k2().value();
			rs.z() = c->k3().value();
		}

		if (!c->useRadialDistortionModel()) {
			rs.setZero();
		}

		rd_v->setEstimate(rs);

		rd_v->setFixed(c->isFixed() or !c->useRadialDistortionModel() or getFixedParametersFlag()&FixedParameter::CameraInternal);

		Eigen::Vector2d ts;

		if (c->optimizedP1().isSet() and
				c->optimizedP2().isSet()) {

			ts.x() = c->optimizedP1().value();
			ts.y() = c->optimizedP2().value();

		} else {

			ts.x() = c->p1().value();
			ts.y() = c->p2().value();
		}

		if (!c->useTangentialDistortionModel()) {
			ts.setZero();
		}

		td_v->setEstimate(ts);

		td_v->setFixed(c->isFixed() or !c->useTangentialDistortionModel() or getFixedParametersFlag()&FixedParameter::CameraInternal);

		Eigen::Vector2d ss;

		if (c->optimizedB1().isSet() and
				c->optimizedB2().isSet()) {

			ss.x() = c->optimizedB1().value();
			ss.y() = c->optimizedB2().value();

		} else {

			ss.x() = c->B1().value();
			ss.y() = c->B2().value();
		}

		if (!c->useSkewDistortionModel()) {
			ss.setZero();
		}
		sd_v->setEstimate(ss);

		sd_v->setFixed(c->isFixed() or !c->useSkewDistortionModel() or getFixedParametersFlag()&FixedParameter::CameraInternal);

		_optimizer->addVertex(cp_v);
		_optimizer->addVertex(rd_v);
		_optimizer->addVertex(td_v);
		_optimizer->addVertex(sd_v);

		_camVertices.insert(cam_id, {cp_v, rd_v, td_v, sd_v});

	}

	return cam_id;
}

} // namespace StereoVisionApp
