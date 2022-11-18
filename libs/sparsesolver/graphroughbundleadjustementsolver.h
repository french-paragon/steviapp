#ifndef STEREOVISIONAPP_GRAPHROUGHBUNDLEADJUSTEMENTSOLVER_H
#define STEREOVISIONAPP_GRAPHROUGHBUNDLEADJUSTEMENTSOLVER_H

#include "./sparsesolverbase.h"
#include "g2o/core/block_solver.h"
#include "./vertices/vertexcameraparam.h"
#include "./vertices/vertexlandmarkpos.h"


#include <QMap>

namespace g2o {
	class SparseOptimizer;
}

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >::LinearSolverType lSolvType;
typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > bSolvType;

namespace StereoVisionApp {

typedef VertexLandmarkPos landMarkVertex;

class VertexCameraPose;
class VertexRigidBodyPose;

class Project;

class GraphRoughBundleAdjustementSolver : public SparseSolverBase
{
public:
	GraphRoughBundleAdjustementSolver(Project* p, bool sparse = true, bool use_current_solution = false, QObject* parent = nullptr);
	virtual ~GraphRoughBundleAdjustementSolver();

	bool hasUncertaintyStep() const override;

protected:

	bool init() override;
	bool opt_step() override;
	bool writeResults() override;
	void cleanup() override;

	bool splitOptSteps() const override;

	bool _sparse;
	bool _not_first_step;

	bool _use_current_solution;
	bool _robust_kernel;

	g2o::SparseOptimizer* _optimizer;

	QMap<qint64, landMarkVertex*> _landmarkVertices;
	QMap<qint64, VertexCameraPose*> _frameVertices;
	QMap<qint64, VertexRigidBodyPose*> _localCoordinatesVertices;

	g2o::SparseBlockMatrix<Eigen::MatrixXd> _spinv;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_GRAPHROUGHBUNDLEADJUSTEMENTSOLVER_H
