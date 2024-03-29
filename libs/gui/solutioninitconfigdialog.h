#ifndef SOLUTIONINITCONFIGDIALOG_H
#define SOLUTIONINITCONFIGDIALOG_H

#include <QDialog>

#include "sparsesolver/fixedpreoptimizedparameters.h"

namespace StereoVisionApp {

class Project;

namespace Ui {
class SolutionInitConfigDialog;
}

class SolutionInitConfigDialog : public QDialog
{
	Q_OBJECT

public:
	explicit SolutionInitConfigDialog(QWidget *parent = nullptr);
	~SolutionInitConfigDialog();

	quint64 selectedStartingImage() const;

	FixedParameters getFixedParameters() const;
	void setFixedParameters(FixedParameters parameters);

	void setProject(Project *p);

private:
	Ui::SolutionInitConfigDialog *ui;

	void onRunButtonPressed();
	void onCancelButtonPressed();

	void reloadImageList();

	Project* _p;

	QVector<qint64> _keys;
};

} // namespace StereoVisionApp

#endif // SOLUTIONINITCONFIGDIALOG_H
