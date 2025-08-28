#include "checkboarddetector.h"

#include "datablocks/project.h"
#include "datablocks/image.h"

#include "imageio.h"

#include <StereoVision/imageProcessing/colorConversions.h>

#include <QDebug>

namespace StereoVisionApp {

CheckboardDetector::CheckboardDetector(QObject *parent) :
	SteppedProcess(parent),
	_number_of_steps(-1),
	_lambda_threshold(4.0),
	_filter_failure_threshold(0.6),
	_filter_error_threshold(0.2),
	_project(nullptr)
{

    setObjectName("CheckboardDetector");

}

int CheckboardDetector::CheckboardDetector::numberOfSteps() {

	if (_number_of_steps >= 0) {
		return _number_of_steps;
	}

	return _imagesToProcess.size();
}
QString CheckboardDetector::CheckboardDetector::currentStepName() const {
	return tr("Processing images");
}

void CheckboardDetector::setProject(Project* p) {
	_project = p;
}
void CheckboardDetector::addImagesToProcess(QVector<qint64> const& imgsIdxs) {

	for (qint64 id : imgsIdxs) {
		_imagesToProcess.push_back(id);
	}

}

bool CheckboardDetector::doNextStep() {

	if (_project == nullptr) {
        sendErrorMessageToQtHandlers(SteppedProcess::Critical, tr("Missing project!"));
		return false;
	}

	if (_imagesToProcess.size() <= 0) {
        sendErrorMessageToQtHandlers(SteppedProcess::Critical, tr("No images to process!"));
		return false;
	}

	qint64 imgId = _imagesToProcess.takeFirst();

	Image* img = _project->getDataBlock<Image>(imgId);

	if (img == nullptr) {
		return true; //skip this step and allow to continue the process
	}

	QString imageFile = img->getImageFile();

	ImageArray imageData = getImageData(imageFile);

	if (imageData.empty()) {
		return true; //skip this step and allow to continue the process
	}

	auto shp = imageData.shape();

	GrayImageArray gray = (imageData.shape()[2] == 3) ? StereoVision::ImageProcessing::img2gray(imageData) : imageData.sliceView(2,0);

	float maxGrey = 1;

	for (int i = 0; i < shp[0]; i++) {
		for (int j = 0; j < shp[1]; j++) {
			if (gray.valueUnchecked(i,j) > maxGrey) {
				maxGrey = gray.valueUnchecked(i,j);
			}
		}
	}

	#pragma omp parallel for
	for (int i = 0; i < shp[0]; i++) {
		for (int j = 0; j < shp[1]; j++) {
			gray.atUnchecked(i,j) /= maxGrey;
		}
	}

	auto candidates = StereoVision::checkBoardCornersCandidates(gray, 1, 2, _lambda_threshold); //TODO: make the parameters editables
	if (candidates.size() < 9) {

		Q_EMIT processedImage(imgId,
							  QVector<StereoVision::discretCheckCornerInfos>(candidates.begin(), candidates.end()),
							  {},
							  {});
		return true;
	}

	auto filtereds = StereoVision::checkBoardFilterCandidates(gray, candidates, _filter_failure_threshold, _filter_error_threshold);

	if (filtereds.size() < 9) {

		Q_EMIT processedImage(imgId,
							  QVector<StereoVision::discretCheckCornerInfos>(candidates.begin(), candidates.end()),
							  QVector<StereoVision::discretCheckCornerInfos>(filtereds.begin(), filtereds.end()),
							  {});

		return true;
	}

	auto selected = StereoVision::isolateCheckBoard(filtereds, 0.3, 0.25);

	if (selected.nPointsFound() < 9) {

		Q_EMIT processedImage(imgId,
							  QVector<StereoVision::discretCheckCornerInfos>(candidates.begin(), candidates.end()),
							  QVector<StereoVision::discretCheckCornerInfos>(filtereds.begin(), filtereds.end()),
							  {});

		return true;
	}

	auto refined = StereoVision::refineCheckBoardCorners(gray, selected);

	Q_EMIT processedImage(imgId,
						  QVector<StereoVision::discretCheckCornerInfos>(candidates.begin(), candidates.end()),
						  QVector<StereoVision::discretCheckCornerInfos>(filtereds.begin(), filtereds.end()),
						  QVector<StereoVision::refinedCornerInfos>(refined.begin(), refined.end()));

	return true;

}

bool CheckboardDetector::init() {

	_number_of_steps = _imagesToProcess.size();

	if (_project == nullptr) {
        sendErrorMessageToQtHandlers(SteppedProcess::Critical, tr("Missing project!"));
		return false;
	}

	return true;
}
void CheckboardDetector::cleanup() {
	_number_of_steps = -1;
}

void CheckboardDetector::setFilter_error_threshold(float filter_error_threshold)
{
	_filter_error_threshold = filter_error_threshold;
}

void CheckboardDetector::setFilter_failure_threshold(float filter_failure_threshold)
{
	_filter_failure_threshold = filter_failure_threshold;
}

void CheckboardDetector::setLambda_threshold(float lambda_threshold)
{
	_lambda_threshold = lambda_threshold;
}

} // namespace StereoVisionApp
