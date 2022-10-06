#ifndef STEREOVISIONAPP_GRAPHCAMERACALIBRATOR_H
#define STEREOVISIONAPP_GRAPHCAMERACALIBRATOR_H

#include "./sparsesolverbase.h"
#include "g2o/core/block_solver.h"
#include "./vertices/vertexcameraparam.h"
#include "./vertices/vertexlandmarkpos.h"

#include "initialsolution.h"

#include <QMap>

namespace g2o {
	class SparseOptimizer;
	class VertexPointXYZ;
}

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >::LinearSolverType lSolvType;
typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > bSolvType;

namespace StereoVisionApp {

typedef g2o::VertexPointXYZ checkboardPointVertex;

class VertexCameraPose;

class Project;
class CameraCalibration;

class GraphCameraCalibrator : public SparseSolverBase
{
public:
	GraphCameraCalibrator(CameraCalibration* calib, bool sparse = true, QObject* parent = nullptr);
	virtual ~GraphCameraCalibrator();

	int uncertaintySteps() const override;
	bool hasUncertaintyStep() const override;

	bool initialize();
	bool runSteps(int n_steps);

protected:

	bool init() override;
	bool opt_step() override;
	bool writeResults() override;
	void cleanup() override;

	bool splitOptSteps() const override;

	bool _sparse;
	bool _not_first_step;

	CameraCalibration* _currentCalibration;

	g2o::SparseOptimizer* _optimizer;

	QMap<QPoint, checkboardPointVertex*> _checkboardpointsVertices;

	QMap<qint64, CameraInnerVertexCollection> _camVertices;
	QMap<qint64, VertexCameraPose*> _frameVertices;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_GRAPHCAMERACALIBRATOR_H
