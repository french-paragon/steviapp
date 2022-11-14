#ifndef SPARSESOLVERCONFIGDIALOG_H
#define SPARSESOLVERCONFIGDIALOG_H

#include <QDialog>

#include "sparsesolver/fixedpreoptimizedparameters.h"

namespace StereoVisionApp {

namespace Ui {
class SparseSolverConfigDialog;
}

class SparseSolverConfigDialog : public QDialog
{
	Q_OBJECT

public:
	explicit SparseSolverConfigDialog(QWidget *parent = nullptr);
	~SparseSolverConfigDialog();

	void enableComputeUncertaintyOption(bool enable);
	void enableUseCurrentSolutionOption(bool enable);

	bool shouldRun() const;
	bool computeUncertainty() const;
	bool useSparseOptimizer() const;
	bool initWithCurrentSol() const;
	int numberOfSteps() const;

	void setNumberOfSteps(int nSteps);

	FixedParameters getFixedParameters() const;
	void setFixedParameters(FixedParameters parameters);

private:
	Ui::SparseSolverConfigDialog *ui;

	void onRunButtonPressed();
	void onCancelButtonPressed();

	bool _shouldRun;
};

} //namespace StereoVisionApp

#endif // SPARSESOLVERCONFIGDIALOG_H
