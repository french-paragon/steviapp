#ifndef STEREOVISIONAPP_SPARSESOLVERBASE_H
#define STEREOVISIONAPP_SPARSESOLVERBASE_H

#include "processing/steppedprocess.h"

#include "fixedpreoptimizedparameters.h"

namespace StereoVisionApp {

class Project;

class SparseSolverBase : public SteppedProcess
{
public:
	SparseSolverBase(Project* p,QObject *parent = nullptr);

	int numberOfSteps() override;

	int optimizationSteps() const;
	void setOptimizationSteps(int n_step);

	virtual int uncertaintySteps() const;
	void setUncertaintySteps(int n_step_std);

	virtual bool hasUncertaintyStep() const = 0;

	QString currentStepName() override;

        inline FixedParameters getFixedParametersFlag() const {
            return _fixedParameters;
        }

        inline void setFixedParametersFlag(FixedParameters fixedParameters) {
            _fixedParameters = fixedParameters;
        }

protected:

	bool doNextStep() override;

	virtual bool opt_step() = 0;
	virtual bool std_step();
	virtual bool writeResults() = 0;
	virtual bool writeUncertainty();

	virtual bool splitOptSteps() const;

	Project* _currentProject;

        FixedParameters _fixedParameters;

	int _n_step;
	int _n_step_std;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_SPARSESOLVERBASE_H
