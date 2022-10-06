#ifndef STEREOVISIONAPP_GRAPHSTEREORIGSOLVER_H
#define STEREOVISIONAPP_GRAPHSTEREORIGSOLVER_H

#include "./sparsesolverbase.h"
#include "g2o/core/block_solver.h"
#include "./vertices/vertexcameraparam.h"
#include "./vertices/vertexlandmarkpos.h"

#include "initialsolution.h"

#include <QMap>
#include <optional>

namespace g2o {
	class SparseOptimizer;
	class VertexPointXYZ;
}

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >::LinearSolverType lSolvType;
typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > bSolvType;

namespace StereoVisionApp {

typedef VertexLandmarkPos landMarkVertex;

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

	bool initialize(const InitialSolution* sol = nullptr);
	bool runSteps(int n_steps);

	std::optional<Eigen::Vector3f> getLandmarkOptimizedPos(qint64 id) const;
	std::optional<InitialSolution::Pt6D> getOptimizedViewPose(qint64 id) const;
	std::optional<float> getOptimizedCamFocal(qint64 id) const;

	InitialSolution resultToSolution() const;

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
