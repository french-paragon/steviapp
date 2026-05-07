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

    struct OptimizerTypeInfos {
        QString name;
        QString toolTip;
        int id;
    };

	explicit SparseSolverConfigDialog(QWidget *parent = nullptr);
	~SparseSolverConfigDialog();

	void enableComputeUncertaintyOption(bool enable);
	void enableUseCurrentSolutionOption(bool enable);

	bool shouldRun() const;
    bool computeUncertainty() const;
    bool useRobustCameras() const;
	bool initWithCurrentSol() const;
	int numberOfSteps() const;

    void setOptimizerTypeList(QList<OptimizerTypeInfos> const& optTypes);
    int selectedOptimizerTypeId() const;

    double functionTolerance() const;
    double parametersTolerance() const;

    void setComputeUncertainty(bool compute);
    void setUseRobustCameras(bool useRobust);

	void setNumberOfSteps(int nSteps);

    QString logsDirectory() const;
    void setLogsDirectory(QString const& directory);

    QString logsPrefix() const;
    void setLogsPrefix(QString const& prefix);

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
