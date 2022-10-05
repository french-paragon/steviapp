#include "sparsesolverbase.h"

#include "datablocks/project.h"

namespace StereoVisionApp {

SparseSolverBase::SparseSolverBase(Project *p, QObject *parent) :
	SteppedProcess(parent),
	_currentProject(p)
{

}

int SparseSolverBase::numberOfSteps() {

	int n = (splitOptSteps() ? optimizationSteps() : 1) + 1;
	if (hasUncertaintyStep()) n += uncertaintySteps();

	return n;

}

int SparseSolverBase::optimizationSteps() const
{
	return _n_step;
}

void SparseSolverBase::setOptimizationSteps(int n_step)
{
	_n_step = n_step;
}

int SparseSolverBase::uncertaintySteps() const
{
	if (!hasUncertaintyStep()) {
		return 0;
	}
	return _n_step_std;
}

void SparseSolverBase::setUncertaintySteps(int n_step_std)
{
	_n_step_std = n_step_std;
}

QString SparseSolverBase::currentStepName() {

	if (currentStep() < optimizationSteps()) {
		return tr("Optimize solution");
	} else if (currentStep() < optimizationSteps() + uncertaintySteps()) {
		return tr("Compute uncertainty");
	} else if (currentStep() == optimizationSteps() + uncertaintySteps()) {
		return tr("Write results");
	}

	return "-";
}

bool SparseSolverBase::std_step() {
	return true;
}

bool SparseSolverBase::writeUncertainty() {
	return true;
}

bool SparseSolverBase::splitOptSteps() const {
	return true;
}

bool SparseSolverBase::doNextStep() {

	bool s = false;

	if ((splitOptSteps() and currentStep() < optimizationSteps()) or (currentStep() < 1 and !splitOptSteps())) {
		s = opt_step();
	} else if (currentStep() < (splitOptSteps() ? optimizationSteps() : 1) + uncertaintySteps()) {
		s = std_step();
	} else if (currentStep() == (splitOptSteps() ? optimizationSteps() : 1) + uncertaintySteps()) {
		s = writeResults();

		if (s and hasUncertaintyStep()) {
			s = writeUncertainty();
		}
	}

	return s;
}

} // namespace StereoVisionApp
