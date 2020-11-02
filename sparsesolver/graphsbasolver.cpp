#include "graphsbasolver.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/block_solver.h"
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

#include <Eigen/Geometry>

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >::LinearSolverType lSolvType;
typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > bSolvType;

typedef g2o::VertexSBAPointXYZ landMarkVertex;

namespace StereoVisionApp {

GraphSBASolver::GraphSBASolver(bool sparse, int nStep) :
	_sparse(sparse),
	_n_step(nStep)
{

}

bool GraphSBASolver::solve(Project* p) const {

	g2o::SparseOptimizer optimizer;
	optimizer.setVerbose(false);

	std::unique_ptr<lSolvType> linearSolver;
	if (_sparse) {
		linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >::PoseMatrixType>>();
	} else {
		linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >::PoseMatrixType>>();
	}

	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<bSolvType>(std::move(linearSolver)));

	optimizer.setAlgorithm(solver);

	SBAGraphReductor selector(3,2,true,true);

	SBAGraphReductor::elementsSet selection = selector(p);

	int vid = 0;

	QMap<qint64, landMarkVertex*> landmarkVertices;

	for (qint64 id : selection.pts) {
		Landmark* lm = qobject_cast<Landmark*>(p->getById(id));

		if (lm != nullptr) {

			landMarkVertex* v = new landMarkVertex();
			v->setId(vid++);
			v->setMarginalized(true);

			g2o::Vector3 d;
			d.x() = lm->xCoord().value();
			d.y() = lm->yCoord().value();
			d.z() = lm->zCoord().value();

			v->setEstimate(d);

			optimizer.addVertex(v);
			landmarkVertices.insert(id, v);

			if (lm->xCoord().isUncertain() and lm->yCoord().isUncertain() and lm->zCoord().isUncertain()) {

				EdgeXyzPrior* e = new EdgeXyzPrior();

				e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );

				Eigen::Vector3d m;
				m.x() = lm->xCoord().value();
				m.y() = lm->yCoord().value();
				m.z() = lm->zCoord().value();

				e->setMeasurement(m);

				Eigen::Matrix3d info = Eigen::Matrix3d::Identity();
				info(0,0) = lm->xCoord().stddev()*lm->xCoord().stddev();
				info(1,1) = lm->yCoord().stddev()*lm->yCoord().stddev();
				info(2,2) = lm->zCoord().stddev()*lm->zCoord().stddev();

				e->setInformation(info);

				optimizer.addEdge(e);

			}
		}
	}

	QMap<qint64, CameraInnerVertexCollection> camVertices;
	QMap<qint64, VertexCameraPose*> frameVertices;

	for (qint64 id : selection.imgs) {
		Image* im = qobject_cast<Image*>(p->getById(id));

		if (im != nullptr) {

			qint64 cam_id = im->assignedCamera();
			Camera* c = qobject_cast<Camera*>(p->getById(cam_id));

			if (c == nullptr) {
				continue;
			}

			if (!camVertices.contains(cam_id)) {

				VertexCameraParam* cp_v = new VertexCameraParam();
				VertexCameraRadialDistortion* rd_v = new VertexCameraRadialDistortion();
				VertexCameraTangentialDistortion* td_v = new VertexCameraTangentialDistortion();
				VertexCameraSkewDistortion* sd_v = new VertexCameraSkewDistortion();

				cp_v->setId(vid++);
				rd_v->setId(vid++);
				td_v->setId(vid++);
				sd_v->setId(vid++);

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

				optimizer.addVertex(cp_v);
				optimizer.addVertex(rd_v);
				optimizer.addVertex(td_v);
				optimizer.addVertex(sd_v);

				camVertices.insert(cam_id, {cp_v, rd_v, td_v, sd_v});

			}

			VertexCameraPose* v = new VertexCameraPose();
			v->setId(vid++);

			Eigen::Matrix3d r;
			r = Eigen::AngleAxisd(im->xRot().value()/180.*M_PI, Eigen::Vector3d::UnitX())
				* Eigen::AngleAxisd(im->yRot().value()/180.*M_PI, Eigen::Vector3d::UnitY())
				* Eigen::AngleAxisd(im->zRot().value()/180.*M_PI, Eigen::Vector3d::UnitZ());

			Eigen::Vector3d t;
			t.x() = im->xCoord().value();
			t.y() = im->yCoord().value();
			t.z() = im->zCoord().value();

			CameraPose p(r, t);

			v->setEstimate(p);

			optimizer.addVertex(v);
			frameVertices.insert(id, v);

			if (im->xCoord().isUncertain() and im->yCoord().isUncertain() and im->zCoord().isUncertain() and
				im->xRot().isUncertain() and im->yRot().isUncertain() and im->zRot().isUncertain()) {

				EdgeSE3FullPrior* e = new EdgeSE3FullPrior();

				e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );

				e->setMeasurement(p);

				EdgeSE3FullPrior::InformationType info = EdgeSE3FullPrior::InformationType::Identity();
				info(0,0) = im->xCoord().stddev()*im->xCoord().stddev();
				info(1,1) = im->yCoord().stddev()*im->yCoord().stddev();
				info(2,2) = im->zCoord().stddev()*im->zCoord().stddev();
				info(3,3) = im->xRot().stddev()*im->xRot().stddev();
				info(4,4) = im->yRot().stddev()*im->yRot().stddev();
				info(5,5) = im->zRot().stddev()*im->zRot().stddev();

				e->setInformation(info);

				optimizer.addEdge(e);

			} else if (im->xCoord().isUncertain() and im->yCoord().isUncertain() and im->zCoord().isUncertain()) {

				EdgeSE3xyzPrior* e = new EdgeSE3xyzPrior();

				e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );

				e->setMeasurement(t);

				EdgeSE3xyzPrior::InformationType info = EdgeSE3xyzPrior::InformationType::Identity();
				info(0,0) = im->xCoord().stddev()*im->xCoord().stddev();
				info(1,1) = im->yCoord().stddev()*im->yCoord().stddev();
				info(2,2) = im->zCoord().stddev()*im->zCoord().stddev();

				e->setInformation(info);

				optimizer.addEdge(e);

			} else if (im->xRot().isUncertain() and im->yRot().isUncertain() and im->zRot().isUncertain()) {

				EdgeSE3rpyPrior* e = new EdgeSE3rpyPrior();

				e->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex*>(v) );

				e->setMeasurement(r);

				EdgeSE3rpyPrior::InformationType info = EdgeSE3rpyPrior::InformationType::Identity();
				info(0,0) = im->xRot().stddev()*im->xRot().stddev();
				info(1,1) = im->yRot().stddev()*im->yRot().stddev();
				info(2,2) = im->zRot().stddev()*im->zRot().stddev();

				e->setInformation(info);

				optimizer.addEdge(e);

			}

			for (qint64 lmId : im->listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName)) {
				ImageLandmark* iml = im->getImageLandmark(lmId);

				if (iml != nullptr) {
					landMarkVertex* l_v = landmarkVertices.value(iml->attachedLandmarkid(), nullptr);

					if (l_v == nullptr) {
						continue;
					}

					const CameraInnerVertexCollection& c_vs = camVertices.value(cam_id);

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
					info(0,0) = (iml->x().isUncertain()) ? iml->x().stddev()*iml->x().stddev() : 1;
					info(1,1) = (iml->y().isUncertain()) ? iml->y().stddev()*iml->y().stddev() : 1;

					e->setInformation(info);

					optimizer.addEdge(e);
				}
			}
		}
	}

	optimizer.initializeOptimization();

	optimizer.setVerbose(true);

	std::cout << std::endl << "Performing full BA:" << std::endl;
	optimizer.optimize(_n_step);

	for (qint64 id : landmarkVertices.keys()) {

		Landmark* lm = qobject_cast<Landmark*>(p->getById(id));

		landMarkVertex* lmv = landmarkVertices.value(id);

		const landMarkVertex::EstimateType & e = lmv->estimate();

		lm->setOptimisedX(static_cast<float>(e.x()));
		lm->setOptimisedY(static_cast<float>(e.y()));
		lm->setOptimisedZ(static_cast<float>(e.z()));

	}

	for (qint64 id : frameVertices.keys()) {

		Image* im = qobject_cast<Image*>(p->getById(id));

		VertexCameraPose* imv = frameVertices.value(id);

		const CameraPose & e = imv->estimate();

		Eigen::Vector3d t = e.t();
		Eigen::Vector3d r = e.r().eulerAngles(0,1,2);

		im->setOptXCoord(static_cast<float>(t.x()));
		im->setOptYCoord(static_cast<float>(t.y()));
		im->setOptZCoord(static_cast<float>(t.z()));

		im->setOptXRot(static_cast<float>(r.x()/M_PI*180.));
		im->setOptYRot(static_cast<float>(r.y()/M_PI*180.));
		im->setOptZRot(static_cast<float>(r.z()/M_PI*180.));

	}

	for (qint64 id : camVertices.keys()) {

		Camera* cam = qobject_cast<Camera*>(p->getById(id));
		cam->clearOptimizedParams();

		const CameraInnerVertexCollection & camv = camVertices[id];

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

} // namespace StereoVisionApp
