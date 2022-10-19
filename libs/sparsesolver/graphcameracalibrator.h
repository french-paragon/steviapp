#ifndef STEREOVISIONAPP_GRAPHCAMERACALIBRATOR_H
#define STEREOVISIONAPP_GRAPHCAMERACALIBRATOR_H

#include "./sparsesolverbase.h"
#include "g2o/core/block_solver.h"
#include "./vertices/vertexcameraparam.h"
#include "./vertices/vertexlandmarkpos.h"

#include "./vertices/camerapose.h"

#include "initialsolution.h"

#include <QMap>
#include <QVector>

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
class ImagePair;

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

	CameraPose::Vector6d meanLogRigOffset(QVector<ImagePair*> const& imgPairs) const;
	CameraPose::Vector6d stdLogRigOffset(QVector<ImagePair*> const& imgPairs, CameraPose::Vector6d const& mean) const;
	bool activateStereoRigs();

	bool init() override;
	bool opt_step() override;
	bool writeResults() override;
	void cleanup() override;

	bool splitOptSteps() const override;

	bool _sparse;
	bool _not_first_step;
	bool _stereo_rig_activated;

	int _vid;

	CameraCalibration* _currentCalibration;

	g2o::SparseOptimizer* _optimizer;

	QMap<QPoint, checkboardPointVertex*> _checkboardpointsVertices;

	QMap<qint64, CameraInnerVertexCollection> _camVertices;
	QMap<qint64, VertexCameraPose*> _frameVertices;
	QMap<qint64, VertexCameraPose*> _StereoRigPoseVertices;
	QMap<qint64, QVector<ImagePair*>> _StereoRigToEstimate;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_GRAPHCAMERACALIBRATOR_H
