#include "graphcameracalibrator.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/solver.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "datablocks/image.h"
#include "datablocks/camera.h"
#include "datablocks/cameracalibration.h"

#include "vertices/vertexcamerapose.h"
#include "vertices/vertexcameraparam.h"

#include "edges/edgeparametrizedxyz2uv.h"

#include "LibStevi/geometry/alignement.h"

#include <QDebug>

bool operator<(const QPoint& s1, const QPoint& s2) {
	if (s1.x() == s2.x()) {
		return  s1.y() < s2.y();
	}
	return s1.x() < s2.x();
}

namespace StereoVisionApp {

GraphCameraCalibrator::GraphCameraCalibrator(CameraCalibration *calib, bool sparse, QObject *parent) :
	SparseSolverBase(nullptr, parent),
	_sparse(sparse),
	_currentCalibration(calib),
	_optimizer(nullptr)
{
	if (_currentCalibration != nullptr) {
		_currentProject = calib->getProject();
	}

}
GraphCameraCalibrator::~GraphCameraCalibrator() {

}

int GraphCameraCalibrator::uncertaintySteps() const {
	return 0;
}
bool GraphCameraCalibrator::hasUncertaintyStep() const {
	return false;
}

bool GraphCameraCalibrator::initialize() {

	if (_currentProject == nullptr or _currentCalibration == nullptr) {
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

	if (_currentCalibration->nSelectedCameras() <= 0) {
		_currentCalibration->configureSelectedImages();
	}


	if (_currentCalibration->nSelectedCameras() <= 0 or
			_currentCalibration->selectedGridSize() <= 0) {
		return false;
	}

	int vid = 0;

	for (int i = 0; i < _currentCalibration->selectedGridHeight(); i++) {
		for (int j = 0; j < _currentCalibration->selectedGridWidth(); j++) {


			checkboardPointVertex* v = new checkboardPointVertex();
			v->setId(vid++);
			v->setMarginalized(false);

			QPointF gridPos = _currentCalibration->gridPointXYCoordinate(QPoint(j,i));

			g2o::Vector3 d;
			d.x() = gridPos.x();
			d.y() = gridPos.y();
			d.z() = 0;

			v->setEstimate(d);
			v->setFixed(true);

			_optimizer->addVertex(v);
			_checkboardpointsVertices.insert(QPoint(j,i), v);

		}
	}


	for (int i = 0; i < _currentCalibration->nSelectedCameras(); i++) {

		qint64 id = _currentCalibration->nthSelectedCameras(i);

		auto optCorners = _currentCalibration->getImageCorners(id);

		if (!optCorners.has_value()) {
			continue;
		}

		QVector<StereoVision::refinedCornerInfos> refinedCorners = optCorners.value();

		Image* im = qobject_cast<Image*>(_currentProject->getById(id));

		if (im == nullptr) {
			continue;
		}

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

			cp_v->setReferedDatablock(c);
			rd_v->setReferedDatablock(c);
			td_v->setReferedDatablock(c);
			sd_v->setReferedDatablock(c);

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
		v->setReferedDatablock(im);

		int nPt = refinedCorners.size();

		Eigen::Array2Xf ptCam;
		ptCam.resize(2,nPt);

		Eigen::Array3Xf ptGrid;
		ptGrid.resize(3,nPt);

		QMap<QPoint, QPointF> refinedCornerIndex;

		for (int i = 0; i < nPt; i++) {
			ptCam(0,i) = (refinedCorners[i].pix_coord_x - c->opticalCenterX().value())/c->fLen().value();
			ptCam(1,i) = (refinedCorners[i].pix_coord_y - c->opticalCenterY().value())/c->fLen().value();

			QPoint dpos(refinedCorners[i].grid_coord_x, refinedCorners[i].grid_coord_y);
			QPointF coord = _currentCalibration->gridPointXYCoordinate(dpos);

			refinedCornerIndex.insert(dpos, QPointF(refinedCorners[i].pix_coord_x, refinedCorners[i].pix_coord_y));

			ptGrid(0,i) = coord.x();
			ptGrid(1,i) = coord.y();
			ptGrid(2,i) = 0;
		}

		StereoVision::Geometry::AffineTransform target2cam =
				StereoVision::Geometry::pnp(ptCam, ptGrid);

		Eigen::Matrix3d R = target2cam.R.transpose().cast<double>();
		Eigen::Vector3d t = (-target2cam.R.transpose()*target2cam.t).cast<double>();

		CameraPose p(R, t);

		v->setEstimate(p);

		_optimizer->addVertex(v);
		_frameVertices.insert(id, v);

		for (StereoVision::refinedCornerInfos & corner : refinedCorners) {

			QPoint dpos(corner.grid_coord_x, corner.grid_coord_y);

			if (!_checkboardpointsVertices.contains(dpos)) {
				continue;
			}

			checkboardPointVertex* l_v = _checkboardpointsVertices.value(dpos);

			const CameraInnerVertexCollection& c_vs = _camVertices.value(cam_id);

			EdgeParametrizedXYZ2UV* e = new EdgeParametrizedXYZ2UV();
			e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );
			e->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex*>(l_v) );
			e->setVertex(2, static_cast<g2o::OptimizableGraph::Vertex*>(c_vs.param) );
			e->setVertex(3, static_cast<g2o::OptimizableGraph::Vertex*>(c_vs.radialDist) );
			e->setVertex(4, static_cast<g2o::OptimizableGraph::Vertex*>(c_vs.tangeantialDist) );
			e->setVertex(5, static_cast<g2o::OptimizableGraph::Vertex*>(c_vs.skewDist) );

			QPointF measures = refinedCornerIndex.value(dpos);
			Eigen::Vector2d ptPos;
			ptPos.x() = measures.x();
			ptPos.y() = measures.y();

			e->setMeasurement(ptPos);

			Eigen::Matrix2d info = Eigen::Matrix2d::Identity();

			e->setInformation(info);

			_optimizer->addEdge(e);
		}

	}


	s = _optimizer->initializeOptimization();
	_not_first_step = false;

	_optimizer->setVerbose(true);

	return s;

}
bool GraphCameraCalibrator::runSteps(int pn_steps) {
	int n_steps = _optimizer->optimize(pn_steps, _not_first_step);
	_not_first_step = true;
	return n_steps > 0;
}

bool GraphCameraCalibrator::init() {
	return initialize();
}

bool GraphCameraCalibrator::opt_step() {
	return runSteps(1);
}

bool GraphCameraCalibrator::writeResults() {

	if (_currentProject == nullptr) {
		return false;
	}

	for (qint64 id : _frameVertices.keys()) {

		VertexCameraPose* imv = _frameVertices.value(id);

		const CameraPose & e = imv->estimate();

		Eigen::Vector3d t = e.t();
		Eigen::Vector3d r = StereoVision::Geometry::inverseRodriguezFormulaD(e.r());

		floatParameterGroup<6> poseEstimate;
		poseEstimate.value(0) = t[0];
		poseEstimate.value(1) = t[1];
		poseEstimate.value(2) = t[2];
		poseEstimate.value(3) = r[0];
		poseEstimate.value(4) = r[1];
		poseEstimate.value(5) = r[2];

		_currentCalibration->setImageEstimatedPose(id, poseEstimate);

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
void GraphCameraCalibrator::cleanup() {

	_checkboardpointsVertices.clear();
	_camVertices.clear();
	_frameVertices.clear();

	if (_optimizer != nullptr) {
		delete _optimizer;
		_optimizer = nullptr;
	}
}

bool GraphCameraCalibrator::splitOptSteps() const {
	return true;
}

} // namespace StereoVisionApp
