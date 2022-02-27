#ifndef STEREOVISIONAPP_GRAPHSTEREORIGSOLVER_H
#define STEREOVISIONAPP_GRAPHSTEREORIGSOLVER_H

#include "./sparsesolverbase.h"
#include "g2o/core/block_solver.h"
#include "./vertices/vertexcameraparam.h"

#include <QMap>

namespace g2o {
	class SparseOptimizer;
	class VertexSBAPointXYZ;
}

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >::LinearSolverType lSolvType;
typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > bSolvType;

typedef g2o::VertexSBAPointXYZ landMarkVertex;

namespace StereoVisionApp {

class VertexCameraPose;

class Project;

class GraphStereoRigSolver : public SparseSolverBase
{
public:
	GraphStereoRigSolver(Project* p,
						 qint64 img1,
						 qint64 img2,
						 bool computeUncertainty = true,
						 bool sparse = true,
						 QObject* parent = nullptr);
	virtual ~GraphStereoRigSolver();

	int uncertaintySteps() const override;
	bool hasUncertaintyStep() const override;

protected:

	bool init() override;
	bool opt_step() override;
	bool std_step() override;
	bool writeResults() override;
	bool writeUncertainty() override;
	void cleanup() override;

	bool splitOptSteps() const override;

	bool _sparse;
	bool _compute_marginals;
	bool _not_first_step;

	qint64 _img1;
	qint64 _img2;

	g2o::SparseOptimizer* _optimizer;

	QMap<qint64, landMarkVertex*> _landmarkVertices;

	QMap<qint64, VertexCameraFocal*> _camVertices;
	QMap<qint64, VertexCameraPose*> _frameVertices;

	g2o::SparseBlockMatrix<Eigen::MatrixXd> _spinv;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_GRAPHSTEREORIGSOLVER_H
