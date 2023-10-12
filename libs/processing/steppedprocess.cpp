#include "steppedprocess.h"

#include <QMutexLocker>

namespace StereoVisionApp {

SteppedProcess::SteppedProcess(QObject *parent) :
	QObject(parent),
	_current_step(-1),
	_goOn(false),
	_isDone(false),
	_deleteWhenDone(false)
{
	connect(this, &SteppedProcess::launched, this, &SteppedProcess::process, Qt::AutoConnection);
}
int SteppedProcess::currentStep() const {
	return _current_step;
}

bool SteppedProcess::isPaused() const {
	return !_goOn and currentStep() >= 0;
}
bool SteppedProcess::isDone() const {
	return _isDone;
}

void SteppedProcess::deleteWhenDone(bool set) {

	if (set) {
		connect(this, &SteppedProcess::stopped, this, &QObject::deleteLater);
		connect(this, &SteppedProcess::failed, this, &QObject::deleteLater);
		connect(this, &SteppedProcess::finished, this, &QObject::deleteLater);
	} else {
		if (_deleteWhenDone) {
			disconnect(this, &SteppedProcess::stopped, this, &QObject::deleteLater);
			disconnect(this, &SteppedProcess::failed, this, &QObject::deleteLater);
			disconnect(this, &SteppedProcess::finished, this, &QObject::deleteLater);
		}
	}

}

void SteppedProcess::run() {

	_sync.lock();

	_goOn = true;
	_isDone = false;

	_sync.unlock();

	Q_EMIT launched();
}

void SteppedProcess::process() {


	bool success = true;

	_sync.lock();
	if (_current_step < 0) {
		success = init();
		if (success) {
			_current_step = 0;
		}
	}
	_sync.unlock();

	if (!success) {
		Q_EMIT failed();
		cleanup();
		return;
	}

	while (true) {

		_sync.lock();
		if (_current_step >= numberOfSteps() or !_goOn or !success) {
			_sync.unlock();
			break;
		}
		_sync.unlock();

		Q_EMIT newStepStarted();
		success = doNextStep();

		if (success) {
			_sync.lock();
			_current_step++;
			_sync.unlock();
		}
	}

	_sync.lock();
	if (_current_step >= numberOfSteps() and success) {
		_current_step = -1;
		_isDone = true;
		Q_EMIT finished();
	} else if (_current_step >= 0 and success) {
		Q_EMIT paused();
	} else if (!success) {
		Q_EMIT failed();
	} else {
		Q_EMIT stopped();
	}
	_sync.unlock();

	if (_current_step < 0 or !success) {
		cleanup();
		_current_step = -1;
	}
}


void SteppedProcess::jumpStep() {

    bool extended = false;

    _sync.lock();
    if (_current_step < numberOfSteps()-1) {
        _current_step++;
        extended = true;
    }
    _sync.unlock();

    if (extended) {
        Q_EMIT newStepStarted();
    }
}

void SteppedProcess::stop() {
	QMutexLocker locker(&_sync);
	_goOn = false;
	_current_step = -1;
}
void SteppedProcess::pause() {
	QMutexLocker locker(&_sync);
	_goOn = false;
}

} // namespace StereoVisionApp
