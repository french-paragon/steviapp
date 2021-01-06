#ifndef STEPPROCESSMONITORBOX_H
#define STEPPROCESSMONITORBOX_H

#include <QWidget>

namespace StereoVisionApp {

class SteppedProcess;

namespace Ui {
class StepProcessMonitorBox;
}

class StepProcessMonitorBox : public QWidget
{
	Q_OBJECT

public:
	explicit StepProcessMonitorBox(QWidget *parent = nullptr);
	~StepProcessMonitorBox();

	void setProcess(SteppedProcess* process);
	void clearProcess();

private:
	Ui::StepProcessMonitorBox *ui;

	void onPauseClicked();
	void onCancelClicked();

	void onStepChange();
	void onProcessFinished();
	void onProcessStopped();
	void onProcessFailed();
	void evaluatePauseState();

	void onEnd();

	float _progressInterval;

	SteppedProcess* _currentProcess;

	bool _removeWhenDone;
};

} // namespace StereoVisionApp

#endif // STEPPROCESSMONITORBOX_H
