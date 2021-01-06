#include "stepprocessmonitorbox.h"
#include "ui_stepprocessmonitorbox.h"

#include "processing/steppedprocess.h"

namespace StereoVisionApp {

StepProcessMonitorBox::StepProcessMonitorBox(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::StepProcessMonitorBox),
	_currentProcess(nullptr)
{
	ui->setupUi(this);

	connect(ui->pauseButton, &QPushButton::clicked, this, &StepProcessMonitorBox::onPauseClicked);
	connect(ui->Cancelbutton, &QPushButton::clicked, this, &StepProcessMonitorBox::onCancelClicked);

	clearProcess();
	evaluatePauseState();
}

StepProcessMonitorBox::~StepProcessMonitorBox()
{
	delete ui;
}

void StepProcessMonitorBox::setProcess(SteppedProcess* process) {

	clearProcess();

	ui->progressBar->setEnabled(true);
	ui->progressBar->setRange(0, process->numberOfSteps());
	ui->progressBar->setValue(process->currentStep());
	ui->stepDescr->setText(process->currentStepName());

	connect(process, &SteppedProcess::newStepStarted, this, &StepProcessMonitorBox::onStepChange);
	connect(process, &SteppedProcess::paused, this, &StepProcessMonitorBox::evaluatePauseState);
	connect(process, &SteppedProcess::stopped, this, &StepProcessMonitorBox::onProcessStopped);
	connect(process, &SteppedProcess::failed, this, &StepProcessMonitorBox::onProcessFailed);
	connect(process, &SteppedProcess::finished, this, &StepProcessMonitorBox::onProcessFinished);

	_currentProcess = process;
}

void StepProcessMonitorBox::clearProcess() {

	if (_currentProcess != nullptr) {
		disconnect(_currentProcess, &SteppedProcess::newStepStarted, this, &StepProcessMonitorBox::onStepChange);
		disconnect(_currentProcess, &SteppedProcess::paused, this, &StepProcessMonitorBox::evaluatePauseState);
		disconnect(_currentProcess, &SteppedProcess::stopped, this, &StepProcessMonitorBox::onProcessStopped);
		disconnect(_currentProcess, &SteppedProcess::failed, this, &StepProcessMonitorBox::onProcessFailed);
	}

	ui->progressBar->setValue(0);
	ui->progressBar->setEnabled(false);
	ui->stepDescr->setText("-");

	_currentProcess = nullptr;
}

void StepProcessMonitorBox::onPauseClicked() {

	if (_currentProcess == nullptr) {
		return;
	}

	if (_currentProcess->isPaused()) {
		_currentProcess->run();
	} else if (_currentProcess->isDone()) {
		onEnd();
	} else {
		_currentProcess->pause();
	}

	ui->pauseButton->setEnabled(false);

}
void StepProcessMonitorBox::onCancelClicked() {

	if (_currentProcess == nullptr) {
		return;
	}

	_currentProcess->stop();
	onEnd();

}


void StepProcessMonitorBox::evaluatePauseState() {

	if (_currentProcess == nullptr) {
		ui->pauseButton->setText(tr("Run"));
		ui->pauseButton->setEnabled(false);

		return;
	}

	ui->pauseButton->setEnabled(true);

	if (_currentProcess->isPaused()) {
		ui->pauseButton->setText(tr("Run"));
	} else if (_currentProcess->isDone()) {
		ui->pauseButton->setText(tr("Done"));
	} else {
		ui->pauseButton->setText(tr("Pause"));
	}

}

void StepProcessMonitorBox::onStepChange() {

	if (_currentProcess == nullptr) {
		return;
	}

	if (_currentProcess->currentStep() == _currentProcess->numberOfSteps() -1) {
		//disable pause button, it wont have any effect.
		ui->pauseButton->setEnabled(false);
	}

	if (_currentProcess->isDone()) {
		evaluatePauseState();
		ui->progressBar->setValue(_currentProcess->numberOfSteps());
		ui->stepDescr->setText(tr("Finished"));
	} else {
		ui->progressBar->setValue(_currentProcess->currentStep());
		ui->stepDescr->setText(_currentProcess->currentStepName());
	}

}
void StepProcessMonitorBox::onProcessFinished() {

	if (_currentProcess == nullptr) {
		return;
	}

	if (_currentProcess->isDone()) {
		evaluatePauseState();
		ui->progressBar->setValue(_currentProcess->numberOfSteps());
		ui->stepDescr->setText(tr("Finished"));
	}
}
void StepProcessMonitorBox::onProcessStopped() {
	ui->stepDescr->setText(tr("Stopped"));
	ui->pauseButton->setText(tr("Retry"));
}
void StepProcessMonitorBox::onProcessFailed() {
	ui->stepDescr->setText(tr("Failed"));
	ui->pauseButton->setText(tr("Retry"));
}

void StepProcessMonitorBox::onEnd() {

	if (windowType() & Qt::Window) {
		close();
		if (_removeWhenDone) {
			deleteLater();
		}
	}

}

} //namespace StereoVisionApp
