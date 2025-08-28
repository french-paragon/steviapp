#ifndef STEREOVISIONAPP_CHECKBOARDDETECTOR_H
#define STEREOVISIONAPP_CHECKBOARDDETECTOR_H

#include "processing/steppedprocess.h"
#include <StereoVision/imageProcessing/checkBoardDetection.h>

#include <QVector>

namespace StereoVisionApp {

class Project;

class CheckboardDetector : public SteppedProcess
{
	Q_OBJECT
public:
	CheckboardDetector(QObject *parent = nullptr);

	int numberOfSteps() override;
    QString currentStepName() const override;

	void setProject(Project* p);
	void addImagesToProcess(QVector<qint64> const& imgsIdxs);

	void setLambda_threshold(float lambda_threshold);
	void setFilter_failure_threshold(float filter_failure_threshold);
	void setFilter_error_threshold(float filter_error_threshold);

Q_SIGNALS:

	void processedImage(qint64 id,
						QVector<StereoVision::discretCheckCornerInfos> detectedCorners,
						QVector<StereoVision::discretCheckCornerInfos> selectedCorners,
						QVector<StereoVision::refinedCornerInfos> refinedCorners);

protected:

	bool doNextStep() override;

	bool init() override;
	void cleanup() override;

	int _number_of_steps;

	float _lambda_threshold;
	float _filter_failure_threshold;
	float _filter_error_threshold;

	Project* _project;
	QVector<qint64> _imagesToProcess;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CHECKBOARDDETECTOR_H
