#ifndef STEREOVISIONAPP_STEPPEDPROCESS_H
#define STEREOVISIONAPP_STEPPEDPROCESS_H

#include <QObject>
#include <QMutex>

namespace StereoVisionApp {

class SteppedProcess : public QObject
{
	Q_OBJECT
public:
	explicit SteppedProcess(QObject *parent = nullptr);

	virtual int numberOfSteps() = 0;
	int currentStep() const;
	virtual QString currentStepName() = 0;

	bool isPaused() const;
	bool isDone() const;

	void deleteWhenDone(bool set);

Q_SIGNALS:

	void launched();
	void newStepStarted();
	void finished();
	void failed();
	void paused();
	void stopped();

public Q_SLOTS:

	void run();
	void stop();
	void pause();

protected:

	void process();

    /*!
     * \brief jumpStep force a step jump to skip a step manually
     */
    void jumpStep();

	virtual bool doNextStep() = 0;

	virtual bool init() = 0;
	virtual void cleanup() = 0;

private:
	int _current_step;
	bool _goOn;
	bool _isDone;
	bool _deleteWhenDone;

	QMutex _sync;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_STEPPEDPROCESS_H
