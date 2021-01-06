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

#include "g2o/types/sba/types_six_dof_expmap.h"
#include "vertices/vertexcamerapose.h"
#include "vertices/vertexcameraparam.h"

#include "edges/edgeparametrizedxyz2uv.h"
#include "edges/edgexyzprior.h"
#include "edges/edgese3fullprior.h"
#include "edges/edgese3rpyprior.h"
#include "edges/edgese3xyzprior.h"

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

	SBAGraphReductor::elementsSet selection = selector(_currentProject, true);

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

			g2o::Vector3 d;

			d.x() = lm->optimizedX().value();
			d.y() = lm->optimizedY().value();
			d.z() = lm->optimizedZ().value();

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

			qint64 cam_id = im->assignedCamera();
			Camera* c = qobject_cast<Camera*>(_currentProject->getById(cam_id));

			if (c == nullptr) {
				continue;
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

				Eigen::Vector2d extend;
				extend.x() = c->imSize().width();
				extend.y() = c->imSize().height();

				Eigen::Vector2d pp;
				pp.x() = c->opticalCenterX().value();
				pp.y() = c->opticalCenterY().value();

				CamParam cp(extend, pp, c->fLen().value());

				cp_v->setEstimate(cp);

				Eigen::Vector3d rs;
				rs.x() = c->k1().value();
				rs.y() = c->k2().value();
				rs.z() = c->k3().value();
				rd_v->setEstimate(rs);
				rd_v->setFixed(!c->useRadialDistortionModel());

				Eigen::Vector2d ts;
				ts.x() = c->p1().value();
				ts.y() = c->p2().value();
				td_v->setEstimate(ts);
				td_v->setFixed(!c->useTangentialDistortionModel());

				Eigen::Vector2d ss;
				ss.x() = c->B1().value();
				ss.y() = c->B2().value();
				sd_v->setEstimate(ss);
				sd_v->setFixed(!c->useSkewDistortionModel());

				_optimizer->addVertex(cp_v);
				_optimizer->addVertex(rd_v);
				_optimizer->addVertex(td_v);
				_optimizer->addVertex(sd_v);

				_camVertices.insert(cam_id, {cp_v, rd_v, td_v, sd_v});

			}

			VertexCameraPose* v = new VertexCameraPose();
			v->setId(vid++);
			v->setMarginalized(false);

			Eigen::Matrix3d r;
			Eigen::Vector3d t;
			Eigen::Vector3d raxis;

			raxis.x() = im->optXRot().value();
			raxis.y() = im->optYRot().value();
			raxis.z() = im->optZRot().value();
			r = rodriguezFormulaD(raxis);

			t.x() = im->optXCoord().value();
			t.y() = im->optYCoord().value();
			t.z() = im->optZCoord().value();

			CameraPose p(r, t);

			v->setEstimate(p);

			_optimizer->addVertex(v);
			_frameVertices.insert(id, v);

			raxis.x() = im->xRot().value();
			raxis.y() = im->yRot().value();
			raxis.z() = im->zRot().value();
			r = rodriguezFormulaD(raxis);

			t.x() = im->xCoord().value();
			t.y() = im->yCoord().value();
			t.z() = im->zCoord().value();

			if (im->xCoord().isUncertain() and im->yCoord().isUncertain() and im->zCoord().isUncertain() and
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

		landMarkVertex* lmv = _landmarkVertices.value(id);

		const landMarkVertex::EstimateType & e = lmv->estimate();

		lm->setOptimisedX(static_cast<float>(e.x()));
		lm->setOptimisedY(static_cast<float>(e.y()));
		lm->setOptimisedZ(static_cast<float>(e.z()));

	}

	for (qint64 id : _frameVertices.keys()) {

		Image* im = qobject_cast<Image*>(_currentProject->getById(id));

		VertexCameraPose* imv = _frameVertices.value(id);

		const CameraPose & e = imv->estimate();

		Eigen::Vector3d t = e.t();
		Eigen::Vector3d r = inverseRodriguezFormulaD(e.r());

		im->setOptXCoord(static_cast<float>(t.x()));
		im->setOptYCoord(static_cast<float>(t.y()));
		im->setOptZCoord(static_cast<float>(t.z()));

		im->setOptXRot(static_cast<float>(r.x()));
		im->setOptYRot(static_cast<float>(r.y()));
		im->setOptZRot(static_cast<float>(r.z()));

	}

	for (qint64 id : _camVertices.keys()) {

		Camera* cam = qobject_cast<Camera*>(_currentProject->getById(id));
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

		floatParameter x = lm->optimizedX();
		floatParameter y = lm->optimizedY();
		floatParameter z = lm->optimizedZ();

		x.setUncertainty(m(0,0));
		y.setUncertainty(m(1,1));
		z.setUncertainty(m(2,2));

		lm->setOptimisedX(x);
		lm->setOptimisedY(y);
		lm->setOptimisedZ(z);

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

		floatParameter x = im->optXCoord();
		floatParameter y = im->optYCoord();
		floatParameter z = im->optZCoord();

		x.setUncertainty(m(0,0));
		y.setUncertainty(m(1,1));
		z.setUncertainty(m(2,2));

		im->setOptXCoord(x);
		im->setOptYCoord(y);
		im->setOptZCoord(z);

		floatParameter rx = im->optXRot();
		floatParameter ry = im->optYRot();
		floatParameter rz = im->optZRot();

		rx.setUncertainty(m(3,3));
		ry.setUncertainty(m(4,4));
		rz.setUncertainty(m(5,5));

		im->setOptXRot(rx);
		im->setOptYRot(ry);
		im->setOptZRot(rz);

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

} // namespace StereoVisionApp
