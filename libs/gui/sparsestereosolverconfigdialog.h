#ifndef SPARSESTEREOSOLVERCONFIGDIALOG_H
#define SPARSESTEREOSOLVERCONFIGDIALOG_H

#include <QDialog>

namespace StereoVisionApp {

namespace Ui {
class SparseStereoSolverConfigDialog;
}

class Project;

class SparseStereoSolverConfigDialog : public QDialog
{
	Q_OBJECT

public:
	explicit SparseStereoSolverConfigDialog(QWidget *parent = nullptr);
	~SparseStereoSolverConfigDialog();

	bool shouldRun() const;
	bool computeUncertainty() const;
	bool useSparseOptimizer() const;
	int numberOfSteps() const;

	void setProject(Project *p);

	quint64 selectedStartingImage1() const;
	quint64 selectedStartingImage2() const;

private:
	Ui::SparseStereoSolverConfigDialog *ui;

	void onRunButtonPressed();
	void onCancelButtonPressed();

	void reloadImageList();

	bool _shouldRun;

	Project* _p;
	QVector<qint64> _keys;
};

} //namespace StereoVisionApp

#endif // SPARSESTEREOSOLVERCONFIGDIALOG_H
