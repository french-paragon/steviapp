#include "cameracalibration.h"

#include "./image.h"
#include "../vision/imageio.h"

#include "LibStevi/imageProcessing/colorConversions.h"
#include "LibStevi/imageProcessing/checkBoardDetection.h"

#include <QDebug>

namespace StereoVisionApp {

CameraCalibration::CameraCalibration(Project* parent) : DataBlock(parent)
{
	_imageList = new CameraCalibrationImageList(this);
}

void CameraCalibration::addImg(qint64 id) {

	if (hasImage(id)) {
		return;
	}

	Image* img = getProject()->getDataBlock<Image>(id);

	if (img == nullptr) {
		return;
	}

	_imageList->beginResetModel();
	_images.insert(id);

	//CameraCalibrationWorker* worker = new CameraCalibrationWorker(this, id, img->getImageFile());
	//connect(worker, &QThread::finished, worker, &QObject::deleteLater); //ensure the thread gets deleted once done

	//_workersSyncMutex.lock();
	//_worker_queue.push_back(worker);
	//if (_worker_queue.size() == 1) {
		//if the queue was empty, we need to start the worker, else the previous worker is charged with starting the new one.
		//worker->start();
	//}
	//_workersSyncMutex.unlock();

	_imageList->endResetModel();
}


void CameraCalibration::setImageCorners(qint64 id,
										QVector<StereoVision::discretCheckCornerInfos> candidates,
										QVector<StereoVision::discretCheckCornerInfos> selected,
										QVector<StereoVision::refinedCornerInfos> corners) {

	QMutexLocker locker(&_workersSyncMutex);

	bool active = imageIsActive(id);
	_imagesInfos.insert(id, {active, candidates, selected, corners});
}
void CameraCalibration::clearImageCorners(qint64 id) {
	if (_imagesInfos.contains(id)) {
		_imagesInfos.remove(id);
	}
}

void CameraCalibration::removeImg(qint64 id) {

	QMutexLocker locker(&_workersSyncMutex);

	if (!_images.contains(id)) {
		return;
	}

	_imageList->beginResetModel();

	_images.remove(id);
	if (_imagesInfos.contains(id)) {
		_imagesInfos.remove(id);
	}

	_imageList->endResetModel();

}

std::optional<QVector<StereoVision::discretCheckCornerInfos>> CameraCalibration::getDetectedCandidates(qint64 id) const {
	if (_imagesInfos.contains(id)) {
		_workersSyncMutex.lock();
		auto candidates_corners = _imagesInfos.value(id).Candidates;
		_workersSyncMutex.unlock();
		return candidates_corners;
	}

	return std::nullopt;
}
std::optional<QVector<StereoVision::discretCheckCornerInfos>> CameraCalibration::getFilteredCandidates(qint64 id) const {
	if (_imagesInfos.contains(id)) {
		_workersSyncMutex.lock();
		auto selected_corners = _imagesInfos.value(id).Filtered;
		_workersSyncMutex.unlock();
		return selected_corners;
	}

	return std::nullopt;
}
std::optional<QVector<StereoVision::refinedCornerInfos>> CameraCalibration::getImageCorners(qint64 id) const {

	if (_imagesInfos.contains(id)) {
		_workersSyncMutex.lock();
		auto detected_corners = _imagesInfos.value(id).DetectedCorners;
		_workersSyncMutex.unlock();
		return detected_corners;
	}

	return std::nullopt;
}

QJsonObject CameraCalibration::encodeJson() const {

	QJsonObject obj;

	QJsonArray imgs;

	for (qint64 imgId : _images) {

		QJsonObject img;
		img.insert("id", imgId);

		if (_imagesInfos.contains(imgId)) {
			img.insert("active", _imagesInfos[imgId].Active);

			QJsonArray candidates;

			for (StereoVision::discretCheckCornerInfos const& candidate : _imagesInfos[imgId].Candidates) {
				QJsonObject corner;
				corner.insert("pix_x", candidate.pix_coord_x);
				corner.insert("pix_y", candidate.pix_coord_y);
				corner.insert("lambda_min", candidate.lambda_min);
				corner.insert("lambda_max", candidate.lambda_max);
				corner.insert("main_dir", candidate.main_dir);

				candidates.push_back(corner);
			}

			img.insert("candidates", candidates);

			QJsonArray selected;

			for (StereoVision::discretCheckCornerInfos const& candidate : _imagesInfos[imgId].Filtered) {
				QJsonObject corner;
				corner.insert("pix_x", candidate.pix_coord_x);
				corner.insert("pix_y", candidate.pix_coord_y);
				corner.insert("lambda_min", candidate.lambda_min);
				corner.insert("lambda_max", candidate.lambda_max);
				corner.insert("main_dir", candidate.main_dir);

				selected.push_back(corner);
			}

			img.insert("selected", selected);


			QJsonArray corners;

			for (StereoVision::refinedCornerInfos const& cornerInfos : _imagesInfos[imgId].DetectedCorners) {
				QJsonObject corner;
				corner.insert("grid_x", cornerInfos.grid_coord_x);
				corner.insert("grid_y", cornerInfos.grid_coord_y);
				corner.insert("pix_x", cornerInfos.pix_coord_x);
				corner.insert("pix_y", cornerInfos.pix_coord_y);

				corners.push_back(corner);
			}

			img.insert("corners", corners);
		}

		imgs.push_back(img);

	}

	obj.insert("imgs", imgs);

	return obj;

}

void CameraCalibration::configureFromJson(QJsonObject const& data) {

	if (!data.contains("imgs")) {
		return;
	}

	QJsonValue vimgs = data.value("imgs");

	if (!vimgs.isArray()) {
			return;
	}

	QJsonArray imgs = vimgs.toArray();

	for (QJsonValue val : imgs) {
		if (!val.isObject()) {
			continue;
		}

		QJsonObject img = val.toObject();

		if (!img.contains("id")) {
			continue;
		}

		qint64 id = img.value("id").toInt(-1);

		if (id >= 0) {
			_images.insert(id);
		}

		if (!img.contains("active") or !img.contains("corners")) {
			continue;
		}

		ImgInfos infos;

		infos.Active = img.value("active").toBool();

		QJsonValue cand = img.value("candidates");

		if (cand.isArray()) {
			QJsonArray corners = cand.toArray();

			for (QJsonValue vc : corners) {

				if (!vc.isObject()) {
					continue;
				}

				QJsonObject corner = vc.toObject();

				if (!corner.contains("pix_x") or
					!corner.contains("pix_y") or
					!corner.contains("lambda_min") or
					!corner.contains("lambda_max") or
					!corner.contains("main_dir")) {

					continue;

				}


				StereoVision::discretCheckCornerInfos cornerInfos;

				cornerInfos.pix_coord_x = corner.value("pix_x").toInt(-1);
				cornerInfos.pix_coord_y = corner.value("pix_y").toInt(-1);

				cornerInfos.lambda_min = corner.value("lambda_min").toDouble(-1);
				cornerInfos.lambda_max = corner.value("lambda_max").toDouble(1);

				cornerInfos.main_dir = corner.value("main_dir").toDouble(0);

				if (cornerInfos.pix_coord_x >= 0 and
					cornerInfos.pix_coord_y >= 0) {

					infos.Candidates.push_back(cornerInfos);

				}
			}
		}

		QJsonValue sel = img.value("selected");

		if (sel.isArray()) {
			QJsonArray corners = sel.toArray();

			for (QJsonValue vc : corners) {

				if (!vc.isObject()) {
					continue;
				}

				QJsonObject corner = vc.toObject();

				if (!corner.contains("pix_x") or
					!corner.contains("pix_y") or
					!corner.contains("lambda_min") or
					!corner.contains("lambda_max") or
					!corner.contains("main_dir")) {

					continue;

				}


				StereoVision::discretCheckCornerInfos cornerInfos;

				cornerInfos.pix_coord_x = corner.value("pix_x").toInt(-1);
				cornerInfos.pix_coord_y = corner.value("pix_y").toInt(-1);

				cornerInfos.lambda_min = corner.value("lambda_min").toDouble(-1);
				cornerInfos.lambda_max = corner.value("lambda_max").toDouble(1);

				cornerInfos.main_dir = corner.value("main_dir").toDouble(0);

				if (cornerInfos.pix_coord_x >= 0 and
					cornerInfos.pix_coord_y >= 0) {

					infos.Filtered.push_back(cornerInfos);

				}
			}
		}

		QJsonValue v = img.value("corners");

		if (v.isArray()) {
			QJsonArray corners = v.toArray();

			for (QJsonValue vc : corners) {

				if (!vc.isObject()) {
					continue;
				}

				QJsonObject corner = vc.toObject();

				if (!corner.contains("grid_x") or
					!corner.contains("grid_y") or
					!corner.contains("pix_x") or
					!corner.contains("pix_y")) {

					continue;

				}


				StereoVision::refinedCornerInfos cornerInfos;

				cornerInfos.grid_coord_x = corner.value("grid_x").toInt(-1);
				cornerInfos.grid_coord_y = corner.value("grid_y").toInt(-1);

				cornerInfos.pix_coord_x = corner.value("pix_x").toDouble(-1);
				cornerInfos.pix_coord_y = corner.value("pix_y").toDouble(-1);

				if (cornerInfos.grid_coord_x >= 0 and
					cornerInfos.grid_coord_y >= 0 and
					cornerInfos.pix_coord_x >= 0 and
					cornerInfos.pix_coord_y >= 0) {

					infos.DetectedCorners.push_back(cornerInfos);

				}
			}
		}

		_imagesInfos.insert(id, infos);

	}

}

CameraCalibrationWorker::CameraCalibrationWorker(CameraCalibration* parent, qint64 imgId, QString imageFile) :
	QThread(parent),
	_calibration(parent),
	_imgId(imgId),
	_imageFile(imageFile)
{

}

void CameraCalibrationWorker::run() {

	_calibration->_workersSyncMutex.lock();

	if (!_calibration->_images.contains(_imgId)) {
		_calibration->_workersSyncMutex.unlock();
		endWorker();
		return;
	}

	_calibration->_workersSyncMutex.unlock();

	ImageArray imageData = getImageData(_imageFile);

	if (imageData.empty()) {
		endWorker();
		return;
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

	auto candidates = StereoVision::checkBoardCornersCandidates(gray, 1, 2, 6.); //TODO: make the parameters editables

	if (candidates.size() < 9) {
		endWorker();
		return;
	}

	_calibration->_workersSyncMutex.lock();

	if (!_calibration->_images.contains(_imgId)) {
		_calibration->_workersSyncMutex.unlock();
		endWorker();
		return;
	}

	_calibration->_workersSyncMutex.unlock();

	auto filtereds = StereoVision::checkBoardFilterCandidates(gray, candidates, 0.2, 0.6);

	if (filtereds.size() < 9) {
		endWorker();
		return;
	}

	auto selected = StereoVision::isolateCheckBoard(filtereds, 0.3, 0.25);

	if (selected.nPointsFound() < 9) {
		endWorker();
		return;
	}

	auto refined = StereoVision::refineCheckBoardCorners(gray, selected);

	_calibration->_workersSyncMutex.lock();

	if (!_calibration->_images.contains(_imgId)) {
		_calibration->_workersSyncMutex.unlock();
		endWorker();
		return;
	}

	_calibration->_imagesInfos[_imgId] = {true,
										  QVector<StereoVision::discretCheckCornerInfos>(candidates.begin(), candidates.end()),
										  QVector<StereoVision::discretCheckCornerInfos>(filtereds.begin(), filtereds.end()),
										  QVector<StereoVision::refinedCornerInfos>(refined.begin(), refined.end())};

	_calibration->_workersSyncMutex.unlock();

	_calibration->imageDataAcquired(_imgId);

	endWorker();
}

void CameraCalibrationWorker::endWorker() {
	_calibration->_workersSyncMutex.lock();

	_calibration->_worker_queue.pop_front();

	if (!_calibration->_worker_queue.empty()) {
		_calibration->_worker_queue[0]->start();
	}

	_calibration->_workersSyncMutex.unlock();
}

CameraCalibrationImageList::CameraCalibrationImageList(CameraCalibration* parent) :
	QAbstractListModel(parent),
	_calibration(parent)
{

}

int CameraCalibrationImageList::rowCount(const QModelIndex &parent) const {
	if (parent == QModelIndex()) {
		return _calibration->_images.size();
	}
	return 0;
}

QVariant CameraCalibrationImageList::data(const QModelIndex &index, int role) const {
	int row = index.row();

	qint64 id = _calibration->_images.values()[row];

	Image* img = _calibration->getProject()->getDataBlock<Image>(id);

	if (img == nullptr) {
		return QVariant();
	}

	switch (role) {
	case Qt::DisplayRole:
		return img->objectName();
	case Project::IdRole :
		return img->internalId();
	default:
		break;
	}

	return QVariant();
}

QVariant CameraCalibrationImageList::headerData(int section, Qt::Orientation orientation, int role) const {
	if (section == 0 and orientation == Qt::Horizontal and role == Qt::DisplayRole) {
		return "Images";
	}
	return QVariant();
}

CameraCalibrationFactory::CameraCalibrationFactory(QObject* parent) :
	DataBlockFactory(parent)
{

}

QString CameraCalibrationFactory::TypeDescrName() const {
	return tr("Calibration");
}
DataBlockFactory::FactorizableFlags CameraCalibrationFactory::factorizable() const {
	return DataBlockFactory::RootDataBlock;
}
DataBlock* CameraCalibrationFactory::factorizeDataBlock(Project *parent) const {
	return new CameraCalibration(parent);
}

QString CameraCalibrationFactory::itemClassName() const {
	return cameraCalibrationClassName();
}
QString CameraCalibrationFactory::cameraCalibrationClassName() {
	return CameraCalibration::staticMetaObject.className();
}

} // namespace StereoVisionApp
