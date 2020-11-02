#ifndef STEREOVISIONAPP_GRAPHSBASOLVER_H
#define STEREOVISIONAPP_GRAPHSBASOLVER_H


namespace StereoVisionApp {

class Project;

class GraphSBASolver
{
public:
	GraphSBASolver(bool sparse = true, int nStep = 50);

	bool solve(Project* p) const;

protected:

	bool _sparse;
	int _n_step;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_GRAPHSBASOLVER_H
