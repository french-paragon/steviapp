#include "graphroughbundleadjustementsolver.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/solver.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/robust_kernel_impl.h"

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
#include "vertices/vertexrigidbodypose.h"
#include "vertices/vertexcameraparam.h"

#include "edges/edgecameralandmarkdirectionalalignement.h"
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

GraphRoughBundleAdjustementSolver::GraphRoughBundleAdjustementSolver(Project* p, bool sparse, bool useCurrentSolution, QObject* parent):
	SparseSolverBase(p, parent),
	_sparse(sparse),
	_use_current_solution(useCurrentSolution),
	_robust_kernel(true),
	_optimizer(nullptr)
{

}
GraphRoughBundleAdjustementSolver::~GraphRoughBundleAdjustementSolver() {
	cleanup();
}

bool GraphRoughBundleAdjustementSolver::hasUncertaintyStep() const {
	return false;
}

bool GraphRoughBundleAdjustementSolver::init() {

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

	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<bSolvType>(std::move(linearSolver)));

	_optimizer->setAlgorithm(solver);

	SBAGraphReductor selector(3,2,true,true);

	SBAGraphReductor::elementsSet selection = selector(_currentProject, true);

	if (selection.imgs.isEmpty() or selection.pts.isEmpty()) {
		return false;
	}

	int id_mcdparam = 0;
	MinCamDistanceParam* mcdparam = new MinCamDistanceParam(1.);
	mcdparam->setId(id_mcdparam);

	_optimizer->addParameter(mcdparam);

	int vid = 0;

	for (qint64 id : selection.pts) {
		Landmark* lm = qobject_cast<Landmark*>(_currentProject->getById(id));

		if (lm != nullptr) {

			landMarkVertex* v = new landMarkVertex();
			v->setId(vid++);
			v->setMarginalized(false);
			v->setReferedDatablock(lm);

			g2o::Vector3 d = g2o::Vector3::Zero();

			if (_use_current_solution) {
				d.x() = lm->optPos().value(0);
				d.y() = lm->optPos().value(1);
				d.z() = lm->optPos().value(2);
			}

			v->setEstimate(d); //init the landmarks at position 0;

			if (lm->isFixed()) {
				v->setFixed(true);
			}

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

				if (_robust_kernel) {
					g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
				}

				_optimizer->addEdge(e);

			} else {

				if (lm->xCoord().isSet()) {

					typedef EdgeLmCoordPrior<StereoVision::Geometry::Axis::X> XPriorEdge;

					XPriorEdge* e = new XPriorEdge();

					e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );
					e->setMeasurement(lm->xCoord().value());

					XPriorEdge::InformationType info = XPriorEdge::InformationType::Identity();

					if (lm->xCoord().isUncertain()) {
						info(0,0) = 1./(lm->xCoord().stddev()*lm->xCoord().stddev());
					}

					e->setInformation(info);

					if (_robust_kernel) {
						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
					}

					_optimizer->addEdge(e);
				}

				if (lm->yCoord().isSet()) {

					typedef EdgeLmCoordPrior<StereoVision::Geometry::Axis::Y> YPriorEdge;

					YPriorEdge* e = new YPriorEdge();

					e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );
					e->setMeasurement(lm->yCoord().value());

					YPriorEdge::InformationType info = YPriorEdge::InformationType::Identity();

					if (lm->xCoord().isUncertain()) {
						info(0,0) = 1./(lm->yCoord().stddev()*lm->yCoord().stddev());
					}

					e->setInformation(info);

					if (_robust_kernel) {
						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
					}

					_optimizer->addEdge(e);
				}

				if (lm->zCoord().isSet()) {

					typedef EdgeLmCoordPrior<StereoVision::Geometry::Axis::Z> ZPriorEdge;

					ZPriorEdge* e = new ZPriorEdge();

					e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );
					e->setMeasurement(lm->zCoord().value());

					ZPriorEdge::InformationType info = ZPriorEdge::InformationType::Identity();

					if (lm->xCoord().isUncertain()) {
						info(0,0) = 1./(lm->zCoord().stddev()*lm->zCoord().stddev());
					}

					e->setInformation(info);

					if (_robust_kernel) {
						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
					}

					_optimizer->addEdge(e);
				}
			}
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

			VertexCameraPose* v = new VertexCameraPose();
			v->setId(vid++);
			v->setMarginalized(false);
			v->setReferedDatablock(im);

			Eigen::Matrix3d r;
			Eigen::Vector3d t;
			Eigen::Vector3d raxis;

			r = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

			t.x() = 0;
			t.y() = 0;
			t.z() = 1;

			if (_use_current_solution) {

				raxis.x() = im->optRot().value(0);
				raxis.y() = im->optRot().value(1);
				raxis.z() = im->optRot().value(2);
				r = StereoVision::Geometry::rodriguezFormula(raxis);

				t.x() = im->optPos().value(0);
				t.y() = im->optPos().value(1);
				t.z() = im->optPos().value(2);
			}

			CameraPose p(r, t); //assume NADIR hypothesis with camera facing down.

			v->setEstimate(p);

			_optimizer->addVertex(v);
			_frameVertices.insert(id, v);

			raxis.x() = im->xRot().value();
			raxis.y() = im->yRot().value();
			raxis.z() = im->zRot().value();
			r = StereoVision::Geometry::rodriguezFormula(raxis);

			t.x() = im->xCoord().value();
			t.y() = im->yCoord().value();
			t.z() = im->zCoord().value();

			if (im->isFixed()) {
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

				if (_robust_kernel) {
					g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
				}

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

				if (_robust_kernel) {
					g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
				}

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

				if (_robust_kernel) {
					g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
				}

				_optimizer->addEdge(e);

			}

			for (qint64 lmId : im->listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName)) {
				ImageLandmark* iml = im->getImageLandmark(lmId);

				if (iml != nullptr) {
					landMarkVertex* l_v = _landmarkVertices.value(iml->attachedLandmarkid(), nullptr);

					if (l_v == nullptr) {
						continue;
					}

					EdgeCameraLandmarkDirectionalAlignement* e = new EdgeCameraLandmarkDirectionalAlignement();
					e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );
					e->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex*>(l_v) );
					e->setParameterId(0, id_mcdparam);

					Eigen::Vector2d ptPos;
					ptPos.x() = iml->x().value();
					ptPos.y() = iml->y().value();

					Eigen::Vector2d pp;
					pp.x() = c->opticalCenterX().value();
					pp.y() = c->opticalCenterY().value();

					ptPos -= pp;

					ptPos.x() /= c->fLen().value();
					ptPos.y() /= c->fLenY();

					e->setMeasurementFromNormalizedCoordinates(ptPos);
					e->setInformation(Eigen::Matrix3d::Identity());

					if (_robust_kernel) {
						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
					}

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

					if (_robust_kernel) {
						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
					}

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

				landMarkVertex* v1 = _landmarkVertices.value(triplet->getNthLandmarkId(0), nullptr);
				landMarkVertex* v2 = _landmarkVertices.value(triplet->getNthLandmarkId(1), nullptr);
				landMarkVertex* v3 = _landmarkVertices.value(triplet->getNthLandmarkId(2), nullptr);

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

					if (_robust_kernel) {
						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
					}

					_optimizer->addEdge(e);
				}

			}
		}

	}

	QVector<qint64> stereoRigs = _currentProject->getIdsByClass(StereoRig::staticMetaObject.className());

	for (qint64 id : stereoRigs) {
		StereoRig* rig = _currentProject->getDataBlock<StereoRig>(id);

		if (rig == nullptr) {
			continue;
		}

		if (!rig->isEnabled()) {
			continue;
		}

		bool hasMeasure = true;
		hasMeasure = hasMeasure and rig->optPos().isSet();
		hasMeasure = hasMeasure and rig->optRot().isSet();

		bool hasPrior = hasMeasure;

		if (!hasMeasure and !hasPrior) {
			continue;
		}

		Eigen::Vector3d t;
		Eigen::Matrix3d R;

		if (hasMeasure and _fixedParameters&FixedParameter::StereoRigs) {
            t.x() = rig->optXCoord().value();
            t.y() = rig->optYCoord().value();
            t.z() = rig->optZCoord().value();

            float eX = rig->optXRot().value();
            float eY = rig->optYRot().value();
            float eZ = rig->optZRot().value();

			R = StereoVision::Geometry::eulerDegXYZToRotation(eX, eY, eZ).cast<double>();
		} else if (hasPrior) {
			t.x() = rig->xCoord().value();
			t.y() = rig->yCoord().value();
			t.z() = rig->zCoord().value();

			float eX = rig->xRot().value();
			float eY = rig->yRot().value();
			float eZ = rig->zRot().value();

			R = StereoVision::Geometry::rodriguezFormula(Eigen::Vector3d(eX, eY, eZ));
		} else {
			continue;
		}

		CameraPose cam2tocam1Prior(R, t);

		QVector<qint64> imgPairs = rig->listTypedSubDataBlocks(ImagePair::staticMetaObject.className());

		for (qint64 pair_id : imgPairs) {

			ImagePair* p = qobject_cast<ImagePair*>(rig->getById(pair_id));

			if (p == nullptr) {
				continue;
			}

			if (_frameVertices.contains(p->idImgCam1()) and _frameVertices.contains(p->idImgCam2())) {

				VertexCameraPose* v1 = _frameVertices.value(p->idImgCam1(), nullptr);
				VertexCameraPose* v2 = _frameVertices.value(p->idImgCam2(), nullptr);

				if (v1 != nullptr and v2 != nullptr) {

					EdgeCameraSE3LeverArm* e = new EdgeCameraSE3LeverArm();
					e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v1));
					e->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex*>(v2));

					e->setMeasurement(cam2tocam1Prior);

					if (rig->xCoord().isUncertain() and rig->yCoord().isUncertain() and rig->zCoord().isUncertain() and
						rig->xRot().isUncertain() and rig->yRot().isUncertain() and rig->zRot().isUncertain()) {

						EdgeCameraSE3LeverArm::InformationType info = EdgeSE3FullPrior::InformationType::Identity();
						info(0,0) = 1./(rig->xCoord().stddev()*rig->xCoord().stddev());
						info(1,1) = 1./(rig->yCoord().stddev()*rig->yCoord().stddev());
						info(2,2) = 1./(rig->zCoord().stddev()*rig->zCoord().stddev());
						info(3,3) = 1./(rig->xRot().stddev()*rig->xRot().stddev());
						info(4,4) = 1./(rig->yRot().stddev()*rig->yRot().stddev());
						info(5,5) = 1./(rig->zRot().stddev()*rig->zRot().stddev());

						e->setInformation(info);

					} else {
						e->setInformation(EdgeCameraSE3LeverArm::InformationType::Identity());
					}

					if (_robust_kernel) {
						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
					}

					_optimizer->addEdge(e);

				}
			}

		}

	}

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

			if (_use_current_solution) {

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

				if (_robust_kernel) {
					g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
				}

					_optimizer->addEdge(e);

			}
		}
	}

	s = _optimizer->initializeOptimization();
	_not_first_step = false;

	_optimizer->setVerbose(true);

	return s;
}

bool GraphRoughBundleAdjustementSolver::opt_step() {
	int n_steps = _optimizer->optimize(1, _not_first_step);
	//int n_steps = _optimizer->optimize(optimizationSteps());
	_not_first_step = true;
	return n_steps > 0;
}
bool GraphRoughBundleAdjustementSolver::writeResults() {

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
void GraphRoughBundleAdjustementSolver::cleanup() {

	_landmarkVertices.clear();
	_frameVertices.clear();

	_spinv.clear(true);

	if (_optimizer != nullptr) {
		delete _optimizer;
		_optimizer = nullptr;
	}
}

bool GraphRoughBundleAdjustementSolver::splitOptSteps() const {
	return true;
}

} // namespace StereoVisionApp
