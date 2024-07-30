#include "image.h"

#include <QFileInfo>
#include <QUrl>
#include <QFileInfo>
#include <QStandardPaths>
#include <QJsonArray>
#include <QSet>

#include "datablocks/landmark.h"
#include "datablocks/camera.h"
#include "datablocks/trajectory.h"
#include "datablocks/dataexception.h"

#include "./itemdatamodel.h"

namespace StereoVisionApp {

const QString ImageLandmark::ImageLandmarkClassName = "StereoVisionApp::ImageLandmark";

Image::Image(Project *parent) :
    RigidBody(parent),
    _assignedCamera(-1),
    _assignedTrajectory(-1)
{
	extendDataModel();
}

qint64 Image::assignedCamera() const {
	return _assignedCamera;
}

Camera* Image::getAssignedCamera() const {
    return getProject()->getDataBlock<Camera>(_assignedCamera);
}
void Image::assignCamera(qint64 camId) {

	if (camId == _assignedCamera) {
		return;
	}

	if (_assignedCamera >= 0) removeRefered({_assignedCamera});
	_assignedCamera = camId;
	if (_assignedCamera >= 0) addRefered({_assignedCamera});
	emit assignedCameraChanged(_assignedCamera);
	return;
}

qint64 Image::assignedTrajectory() const {
    return _assignedTrajectory;
}

Trajectory* Image::getAssignedTrajectory() const {
    return getProject()->getDataBlock<Trajectory>(_assignedTrajectory);
}

void Image::assignTrajectory(qint64 trajId) {

    if (trajId == _assignedTrajectory) {
        return;
    }

    if (_assignedTrajectory >= 0) removeRefered({_assignedTrajectory});
    _assignedTrajectory = trajId;
    if (_assignedTrajectory >= 0) addRefered({_assignedTrajectory});
    emit assignedTrajectoryChanged(_assignedTrajectory);
    return;

}

QString Image::getImageFile() const
{
	return _imageFile;
}

void Image::setImageFile(const QString &imageFile)
{
	QString f = imageFile;

	if (f.startsWith("file:")) {
		f = QUrl(f).toLocalFile();
	}

	QFileInfo n(f);
	QFileInfo p(_imageFile);

	if (n.canonicalFilePath() != p.canonicalFilePath()) {
		_imageFile = n.canonicalFilePath();
		emit imageFileChanged(_imageFile);
	}
}

std::optional<double> Image::getImageTimestamp() const {
    return _imageTime;
}
void Image::setImageTimeStamp(std::optional<double> const& timing) {
    if (timing != _imageTime) {
        _imageTime = timing;
        Q_EMIT imageTimeChanged(timing);
    }
}


qint64 Image::addImageLandmark(QPointF const& coordinates, bool uncertain, qreal sigma_pos) {

	return addImageLandmark(coordinates, -1, uncertain, sigma_pos);

}
qint64 Image::addImageLandmark(const QPointF &coordinates, qint64 attacheLandmarkId, bool uncertain, qreal sigma_pos) {

	ImageLandmark* iml = new ImageLandmark(this);

	iml->stopTrackingChanges(true);

	floatParameter x(coordinates.x(), pFloatType(sigma_pos));
	floatParameter y(coordinates.y(), pFloatType(sigma_pos));

	if (!uncertain) {
		x.clearUncertainty();
		y.clearUncertainty();
	}

	iml->setX(x);
	iml->setY(y);

	insertSubItem(iml);

	if (iml->internalId() >= 0) {
		iml->setAttachedLandmark(attacheLandmarkId);
		emit pointAdded(iml->internalId());

		iml->stopTrackingChanges(false);
		return iml->internalId();
	} else {
		iml->clear();
		iml->deleteLater();
	}

	return -1;

}

ImageLandmark* Image::getImageLandmark(qint64 id) const {
	return qobject_cast<ImageLandmark*>(getById(id));
}
ImageLandmark* Image::getImageLandmarkByLandmarkId(qint64 id) const {
	QVector<qint64> lmIds = listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName);

	for (qint64 im_id : lmIds) {
		ImageLandmark* imlm = getImageLandmark(im_id);

		if (imlm != nullptr) {
			if (imlm->attachedLandmarkid() == id) {
				return imlm;
			}
		}
	}

	return nullptr;
}

void Image::clearImageLandmark(qint64 id) {

	ImageLandmark* iml = getImageLandmark(id);

	if (iml != nullptr) {
		clearSubItem(id, iml->metaObject()->className());
		iml = getImageLandmark(id);
		if (iml == nullptr) {
			Q_EMIT pointRemoved(id);
		}
	}

}

qint64 Image::getImageLandMarkAt(QPointF const& coord, float tol) {

	qint64 r = -1;
	float d = std::abs(tol);

	for(qint64 id : listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName)) {
		ImageLandmark* ilm = getImageLandmark(id);

		if (ilm != nullptr) {

			QPointF delta = coord - ilm->imageCoordinates();

			float dm = std::max(std::abs(delta.x()), std::abs(delta.y()));
			if (dm < d) {
				r = id;
				d = dm;
			}
		}
	}

	return r;
}

QVector<qint64> Image::getAttachedLandmarksIds() const {

	if (!isInProject()) {
		return {};
	}

	QVector<qint64> imlmids = listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName);
	QVector<qint64> r;
	r.reserve(imlmids.size());

	for (qint64 id : imlmids) {
		ImageLandmark* imlm = getImageLandmark(id);

		if (imlm == nullptr) {
			continue;
		}

		qint64 lmid = imlm->attachedLandmarkid();

		Landmark* lm = qobject_cast<Landmark*>(getProject()->getById(lmid));

		if (lm != nullptr) {
			r.push_back(lmid);
		}
	}

	return r;

}


Eigen::Array2Xf Image::getImageLandmarksCoordinates(QVector<qint64> ids) const {

	Eigen::Array2Xf coords;
	coords.resize(2, ids.size());

	for (int i = 0; i < ids.size(); i++) {

		ImageLandmark* lm = getImageLandmarkByLandmarkId(ids[i]);

		if (lm == nullptr) {
			throw DataException("Asked for coordinate of an image landmark not in the image !", this);
		}

		coords(0,i) = lm->x().value();
		coords(1,i) = lm->y().value();
	}

	return coords;

}


int Image::countPointsRefered(const QSet<qint64> &excluded) const {

	if (!isInProject()) {
		return 0;
	}

	QSet<qint64> referedPtId;
	for (qint64 id : listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName)) {
		ImageLandmark* iml = qobject_cast<ImageLandmark*>(getById(id));

		if (iml == nullptr) {
			continue;
		}

		qint64 lm_id = iml->attachedLandmarkid();
		Landmark* lm = qobject_cast<Landmark*>(getProject()->getById(lm_id));

		if (lm != nullptr) {
			referedPtId.insert(id);
		}
	}

	for (qint64 id : excluded) {
		referedPtId.remove(id);
	}

	return referedPtId.count();
}
int Image::countPointsRefered(QVector<qint64> const& excluded) const {

	QSet<qint64> s(excluded.begin(), excluded.end());
	return countPointsRefered(s);

}


void Image::clearOptimized() {

	clearOptPos();
	clearOptRot();
}

bool Image::hasOptimizedParameters() const {
	return _o_pos.isSet() or _o_rot.isSet();
}

QJsonObject Image::getJsonRepresentation() const {

	return RigidBody::encodeJson();

}
void Image::setParametersFromJsonRepresentation(QJsonObject const& rep) {
	RigidBody::configureFromJson(rep);
}

QJsonObject Image::encodeJson() const {

	QJsonObject obj = RigidBody::encodeJson();

	obj.insert("imFile", getImageFile());

	obj.insert("assignedCamera", assignedCamera());
    obj.insert("assignedTrajectory", assignedTrajectory());

	QJsonArray arr;

	auto ids = listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName);
	for(qint64 id : qAsConst(ids)) {
		arr.push_back(getImageLandmark(id)->toJson());
	}

	obj.insert("Landmarks", arr);

	return obj;

}
void Image::configureFromJson(QJsonObject const& data) {

	RigidBody::configureFromJson(data);

	if (data.contains("imFile")) {
		_imageFile = data.value("imFile").toString();
	}

	if (data.contains("assignedCamera")) {
		_assignedCamera = data.value("assignedCamera").toInt();
	}

    if (data.contains("assignedTrajectory")) {
        _assignedTrajectory = data.value("assignedTrajectory").toInt();
    }

	if (data.contains("Landmarks")) {
		QJsonArray arr = data.value("Landmarks").toArray();

		for (QJsonValue const& v : arr) {
			QJsonObject o = v.toObject();

			ImageLandmark* iml = new ImageLandmark(this);
			iml->setFromJson(o);

			if (iml->internalId() >= 0) {
				insertSubItem(iml);
			}
		}
	}
}

void Image::extendDataModel() {


	ItemDataModel::Category* g = _dataModel->addCategory(tr("Geometric properties"));

	//Position
	g->addCatProperty<floatParameter, Image, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("X pos"),
																												 &Image::xCoord,
																												 &Image::setXCoord,
																												 &Image::xCoordChanged);

	g->addCatProperty<floatParameter, Image, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Y pos"),
																												 &Image::yCoord,
																												 &Image::setYCoord,
																												 &Image::yCoordChanged);

	g->addCatProperty<floatParameter, Image, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Z pos"),
																												 &Image::zCoord,
																												 &Image::setZCoord,
																												 &Image::zCoordChanged);

	//Rotation
	g->addCatProperty<floatParameter, Image, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("X Euler"),
																												 &Image::xRot,
																												 &Image::setXRot,
																												 &Image::xRotChanged);

	g->addCatProperty<floatParameter, Image, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Y Euler"),
																												 &Image::yRot,
																												 &Image::setYRot,
																												 &Image::yRotChanged);

	g->addCatProperty<floatParameter, Image, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Z Euler"),
																												 &Image::zRot,
																												 &Image::setZRot,
																												 &Image::zRotChanged);

	ItemDataModel::Category* og = _dataModel->addCategory(tr("Optimized geometry"));

	//Position
	og->addCatProperty<float, Image, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("X pos"),
																								  &Image::optXCoord,
																								  &Image::setOptXCoord,
																								  &Image::optPosChanged);

	og->addCatProperty<float, Image, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Y pos"),
																								  &Image::optYCoord,
																								  &Image::setOptYCoord,
																								  &Image::optPosChanged);

	og->addCatProperty<float, Image, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Z pos"),
																								  &Image::optZCoord,
																								  &Image::setOptZCoord,
																								  &Image::optPosChanged);

	//Rotation
	og->addCatProperty<float, Image, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("X Raxis"),
																								  &Image::optXRot,
																								  &Image::setOptXRot,
																								  &Image::optRotChanged);

	og->addCatProperty<float, Image, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Y Raxis"),
																								  &Image::optYRot,
																								  &Image::setOptYRot,
																								  &Image::optRotChanged);

	og->addCatProperty<float, Image, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Z Raxis"),
																								  &Image::optZRot,
																								  &Image::setOptZRot,
																								  &Image::optRotChanged);



	ItemDataModel::Category* optCat = _dataModel->addCategory(tr("Optimizer properties"));

	optCat->addCatProperty<bool, DataBlock, false, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Enabled"),
																										  &DataBlock::isEnabled,
																										  &DataBlock::setEnabled,
																										  &DataBlock::isEnabledChanged);

	optCat->addCatProperty<bool, Image, false, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Fixed"),
																										  &Image::isFixed,
																										  &Image::setFixed,
																										  &Image::isFixedChanged);

	ItemDataModel::SubItemCollectionManager* im_lm = _dataModel->addCollectionManager(tr("Image landmarks"),
																					  ImageLandmark::ImageLandmarkClassName,
																					  [] (DataBlock* b) {
																							ImageLandmark* l = qobject_cast<ImageLandmark*>(b);
																							if (l != nullptr) {
																								return l->attachedLandmarkName();
																							}
																							return tr("Unvalid image landmark");
																						});

	im_lm->addCatProperty<floatParameter, ImageLandmark, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("X pos"),
																														  &ImageLandmark::x,
																														  &ImageLandmark::setX,
																														  &ImageLandmark::xCoordChanged);

	im_lm->addCatProperty<floatParameter, ImageLandmark, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Y pos"),
																														  &ImageLandmark::y,
																														  &ImageLandmark::setY,
																														  &ImageLandmark::yCoordChanged);

}

ImageLandmark::ImageLandmark(Image* parent) : DataBlock(parent)
{
	connect(this, &ImageLandmark::xCoordChanged, this, &ImageLandmark::coordsChanged);
	connect(this, &ImageLandmark::yCoordChanged, this, &ImageLandmark::coordsChanged);
}

qint64 ImageLandmark::attachedLandmarkid() const {
	return _attachedLandmarkId;
}
QString ImageLandmark::attachedLandmarkName() const {
	Project* p = getProject();

	if (p == nullptr) {
		return "";
	}

	Landmark* lm = qobject_cast<Landmark*>(p->getById(attachedLandmarkid()));

	if (lm != nullptr) {
		return lm->objectName();
	}

	return "";
}
Landmark* ImageLandmark::attachedLandmark() const {
	Project* p = getProject();

	if (p == nullptr) {
		return nullptr;
	}

	Landmark* lm = qobject_cast<Landmark*>(p->getById(attachedLandmarkid()));

	return lm;
}

void ImageLandmark::setAttachedLandmark(qint64 id) {

	if (id < 0 and _attachedLandmarkId >= 0) {
		removeRefered({_attachedLandmarkId});
		_attachedLandmarkId = -1;
		emit attachedLandmarkidChanged(-1);
		return;
	} else if (id < 0) {
		return;
	}

	if (id == _attachedLandmarkId) {
		return;
	}

	qint64 t = id;

	Image* im_p = qobject_cast<Image*>(parent());

	if (im_p != nullptr) {
		QVector<qint64> imLandmarkIds = im_p->listTypedSubDataBlocks(this->metaObject()->className());

		for (qint64 id : imLandmarkIds) {
			ImageLandmark* iml = im_p->getImageLandmark(id);

			if (iml != nullptr) {
				if (iml->attachedLandmarkid() == t) {
					t = -1;
					break;
				}
			}
		}
	}

	if (t != _attachedLandmarkId) {
		if (_attachedLandmarkId >= 0) removeRefered({_attachedLandmarkId});
		_attachedLandmarkId = t;
		if (_attachedLandmarkId >= 0) addRefered({_attachedLandmarkId});
		emit attachedLandmarkidChanged(_attachedLandmarkId);
		return;
	}
}

floatParameter ImageLandmark::x() const
{
	return _x;
}

void ImageLandmark::setX(const floatParameter &x)
{
	if (!x.isApproximatlyEqual(_x, 1e-4)) {
		_x = x;
		emit xCoordChanged(x);
		isChanged();
	}
}
void ImageLandmark::setX(float x) {
	if (!_x.isApproximatlyEqual(x, 1e-4) or !_x.isSet()) {
		_x.setIsSet(x);
		emit xCoordChanged(_x);
		isChanged();
	}
}

floatParameter ImageLandmark::y() const
{
	return _y;
}

void ImageLandmark::setY(const floatParameter &y)
{
	if (!y.isApproximatlyEqual(_y, 1e-4)) {
		_y = y;
		emit yCoordChanged(y);
		isChanged();
	}
}
void ImageLandmark::setY(float y) {
	if (!_y.isApproximatlyEqual(y, 1e-4) or !_y.isSet()) {
		_y.setIsSet(y);
		emit yCoordChanged(_y);
		isChanged();
	}
}

void ImageLandmark::setImageCoordinates(QPointF const& point) {
	setX(point.x());
	setY(point.y());
}

QPointF ImageLandmark::imageCoordinates() const {
	return QPointF(_x.value(), _y.value());
}

QJsonObject ImageLandmark::encodeJson() const {
	QJsonObject obj;

	obj.insert("attachedLandmarkId", attachedLandmarkid());

	obj.insert("x", floatParameter::toJson(x()));
	obj.insert("y", floatParameter::toJson(y()));

	return obj;
}
void ImageLandmark::configureFromJson(QJsonObject const& data) {

	if (data.contains("x")) {
		_x = floatParameter::fromJson(data.value("x").toObject());
	}
	if (data.contains("y")) {
		_y = floatParameter::fromJson(data.value("y").toObject());
	}

	if (data.contains("attachedLandmarkId")) {
		_attachedLandmarkId = data.value("attachedLandmarkId").toInt(-1);
	}
}

void ImageLandmark::referedCleared(QVector<qint64> const& referedId) {

	DataBlock::referedCleared(referedId);

	if (referedId.front() == _attachedLandmarkId) {
		_attachedLandmarkId = -1;
		emit attachedLandmarkidChanged(-1);
	}
}


ImageFactory::ImageFactory(QObject* parent)  : DataBlockFactory(parent)
{

}

QString ImageFactory::TypeDescrName() const {
	return tr("Image");
}
DataBlockFactory::FactorizableFlags ImageFactory::factorizable() const {
	return DataBlockFactory::RootDataBlock;
}
DataBlock* ImageFactory::factorizeDataBlock(Project *parent) const {
	return new Image(parent);
}


QString ImageFactory::itemClassName() const {
	return imageClassName();
}
QString ImageFactory::imageClassName() {
	return Image::staticMetaObject.className();
}

} // namespace StereoVisionApp
