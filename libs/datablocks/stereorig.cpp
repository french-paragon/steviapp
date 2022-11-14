#include "stereorig.h"

#include "itemdatamodel.h"
#include "image.h"

namespace StereoVisionApp {


const QString StereoRig::StereoRigClassName = "StereoVisionApp::StereoRig";
const QString ImagePair::ImagePairClassName = "StereoVisionApp::ImagePair";

StereoRig::StereoRig(Project *parent) :
	DataBlock(parent)
{
	extendDataModel();
}

floatParameter StereoRig::offsetX() const
{
	return _offset_x;
}

floatParameter StereoRig::offsetY() const
{
	return _offset_y;
}

floatParameter StereoRig::offsetZ() const
{
	return _offset_z;
}


void StereoRig::setOffsetX(const floatParameter &offset_x)
{
	if (!_offset_x.isApproximatlyEqual(offset_x, 1e-4)) {
		_offset_x = offset_x;
		emit offsetXChanged(_offset_x);
		isChanged();
	}
}

void StereoRig::setOffsetY(const floatParameter &offset_y)
{
	if (!_offset_y.isApproximatlyEqual(offset_y, 1e-4)) {
		_offset_y = offset_y;
		emit offsetYChanged(_offset_y);
		isChanged();
	}
}

void StereoRig::setOffsetZ(const floatParameter &offset_z)
{
	if (!_offset_z.isApproximatlyEqual(offset_z, 1e-4)) {
		_offset_z = offset_z;
		emit offsetZChanged(_offset_z);
		isChanged();
	}
}

floatParameter StereoRig::offsetRotX() const
{
	return _offset_rx;
}

floatParameter StereoRig::offsetRotY() const
{
	return _offset_ry;
}

floatParameter StereoRig::offsetRotZ() const
{
	return _offset_rz;
}

void StereoRig::setOffsetRotX(const floatParameter &offset_rx)
{
	if (!_offset_rx.isApproximatlyEqual(offset_rx, 1e-4)) {
		_offset_rx = offset_rx;
		emit offsetRotXChanged(_offset_rx);
		isChanged();
	}
}

void StereoRig::setOffsetRotY(const floatParameter &offset_ry)
{
	if (!_offset_ry.isApproximatlyEqual(offset_ry, 1e-4)) {
		_offset_ry = offset_ry;
		emit offsetRotYChanged(_offset_ry);
		isChanged();
	}
}

void StereoRig::setOffsetRotZ(const floatParameter &offset_rz)
{
	if (!_offset_rz.isApproximatlyEqual(offset_rz, 1e-4)) {
		_offset_rz = offset_rz;
		emit offsetRotZChanged(_offset_rz);
		isChanged();
	}
}




floatParameter StereoRig::optOffsetX() const {
	if (_o_offset.isUncertain()) {
		return floatParameter(_o_offset.value(0), std::sqrt(_o_offset.stddev(0)));
	} else {
		return floatParameter(_o_offset.value(0));
	}
}
floatParameter StereoRig::optOffsetY() const {
	if (_o_offset.isUncertain()) {
		return floatParameter(_o_offset.value(1), std::sqrt(_o_offset.stddev(1)));
	} else {
		return floatParameter(_o_offset.value(1));
	}
}
floatParameter StereoRig::optOffsetZ() const {
	if (_o_offset.isUncertain()) {
		return floatParameter(_o_offset.value(2), std::sqrt(_o_offset.stddev(2)));
	} else {
		return floatParameter(_o_offset.value(2));
	}
}

floatParameterGroup<3> StereoRig::optOffset() const {
	return _o_offset;
}
void StereoRig::setOptOffset(floatParameterGroup<3> const& o_pos) {
	if (!_o_offset.isApproximatlyEqual(o_pos, 1e-4)) {
		_o_offset = o_pos;
		emit optOffsetChanged();
		isChanged();
	}
}
void StereoRig::clearOptOffset() {
	if (_o_offset.isSet()) {
		_o_offset.clearIsSet();
		emit optOffsetChanged();
		isChanged();
	}
}


floatParameter StereoRig::optOffsetRotX() const {
	if (_o_offset.isUncertain()) {
		return floatParameter(_o_offsetrot.value(0), std::sqrt(_o_offsetrot.stddev(0)));
	} else {
		return floatParameter(_o_offsetrot.value(0));
	}
}
floatParameter StereoRig::optOffsetRotY() const {
	if (_o_offset.isUncertain()) {
		return floatParameter(_o_offsetrot.value(1), std::sqrt(_o_offsetrot.stddev(1)));
	} else {
		return floatParameter(_o_offsetrot.value(1));
	}
}
floatParameter StereoRig::optOffsetRotZ() const {
	if (_o_offset.isUncertain()) {
		return floatParameter(_o_offsetrot.value(2), std::sqrt(_o_offsetrot.stddev(2)));
	} else {
		return floatParameter(_o_offsetrot.value(2));
	}
}

floatParameterGroup<3> StereoRig::optOffsetRot() const {
	return _o_offsetrot;
}
void StereoRig::setOptOffsetRot(floatParameterGroup<3> const& o_rot) {
	if (!_o_offsetrot.isApproximatlyEqual(o_rot, 1e-4)) {
		_o_offsetrot = o_rot;
		emit optOffsetRotChanged();
		isChanged();
	}
}
void StereoRig::clearOptOffsetRot() {
	if (_o_offsetrot.isSet()) {
		_o_offsetrot.clearIsSet();
		emit optOffsetRotChanged();
		isChanged();
	}
}


ImagePair* StereoRig::getImagePair(qint64 id) const{
	return qobject_cast<ImagePair*>(getById(id));
}

qint64 StereoRig::insertImagePair(qint64 cam1ImgId, qint64 cam2ImgId) {

	if (getPairForImage(cam1ImgId) != nullptr or
			getPairForImage(cam2ImgId) != nullptr) {
		return -1;
	}

	ImagePair* imp = new ImagePair(this);

	imp->stopTrackingChanges(true);

	insertSubItem(imp);

	if (imp->internalId() >= 0) {
		imp->setidImgCam1(cam1ImgId);
		imp->setIdImgCam2(cam2ImgId);

		emit imagePairAdded(imp->internalId());

		imp->stopTrackingChanges(false);

		return imp->internalId();

	} else {
		imp->clear();
		imp->deleteLater();
	}

	return -1;

}

ImagePair* StereoRig::getPairForImage(qint64 id) const {

	QVector<qint64> imlmids = listTypedSubDataBlocks(ImagePair::ImagePairClassName);

	for (qint64 imp_id : imlmids) {

		ImagePair* imp = getImagePair(imp_id);

		if (imp != nullptr) {
			if (imp->idImgCam1() == id or imp->idImgCam2() == id) {
				return imp;
			}
		}
	}

	return nullptr;
}

bool StereoRig::removeImagePair(qint64 id) {

	ImagePair* imp = getImagePair(id);

	if (imp == nullptr) {
		return false;
	}

	clearSubItem(id, ImagePair::ImagePairClassName);

	imp = getImagePair(id);

	if (imp != nullptr) {
		return false;
	}

	return true;

}

QJsonObject StereoRig::encodeJson() const {

	QJsonObject obj;

	obj.insert("ofsx", floatParameter::toJson(offsetX()));
	obj.insert("ofsy", floatParameter::toJson(offsetY()));
	obj.insert("ofsz", floatParameter::toJson(offsetZ()));

	obj.insert("ofsrx", floatParameter::toJson(offsetRotX()));
	obj.insert("ofsry", floatParameter::toJson(offsetRotY()));
	obj.insert("ofsrz", floatParameter::toJson(offsetRotZ()));

	obj.insert("oofs", floatParameterGroup<3>::toJson(optOffset()));

	obj.insert("oofsr", floatParameterGroup<3>::toJson(optOffsetRot()));

	QJsonArray arr;

	for(qint64 id : listTypedSubDataBlocks(ImagePair::ImagePairClassName)) {
		arr.push_back(getImagePair(id)->toJson());
	}

	obj.insert("ImgsPairs", arr);

	return obj;

}

void StereoRig::configureFromJson(QJsonObject const& data) {

	if (data.contains("ofsx")) {
		_offset_x = floatParameter::fromJson(data.value("ofsx").toObject());
	}
	if (data.contains("ofsy")) {
		_offset_y = floatParameter::fromJson(data.value("ofsy").toObject());
	}
	if (data.contains("ofsz")) {
		_offset_z = floatParameter::fromJson(data.value("ofsz").toObject());
	}

	if (data.contains("ofsrx")) {
		_offset_rx = floatParameter::fromJson(data.value("ofsrx").toObject());
	}
	if (data.contains("ofsry")) {
		_offset_ry = floatParameter::fromJson(data.value("ofsry").toObject());
	}
	if (data.contains("ofsry")) {
		_offset_rz = floatParameter::fromJson(data.value("ofsrz").toObject());
	}

	if (data.contains("oofs")) {
		_o_offset = floatParameterGroup<3>::fromJson(data.value("oofs").toObject());
	}

	if (data.contains("oofsr")) {
		_o_offsetrot = floatParameterGroup<3>::fromJson(data.value("oofsr").toObject());
	}

	if (data.contains("ImgsPairs")) {
		QJsonArray arr = data.value("ImgsPairs").toArray();

		for (QJsonValue v : arr) {
			QJsonObject o = v.toObject();

			ImagePair* imp = new ImagePair(this);
			imp->setFromJson(o);

			if (imp->internalId() >= 0) {
				insertSubItem(imp);
			}
		}
	}

}

void StereoRig::extendDataModel() {

	ItemDataModel::Category* g = _dataModel->addCategory(tr("Geometric properties"));

	//Position
	g->addCatProperty<floatParameter, StereoRig, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("X pos"),
																												 &StereoRig::offsetX,
																												 &StereoRig::setOffsetX,
																												 &StereoRig::offsetXChanged);

	g->addCatProperty<floatParameter, StereoRig, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Y pos"),
																												  &StereoRig::offsetY,
																												  &StereoRig::setOffsetY,
																												  &StereoRig::offsetYChanged);

	g->addCatProperty<floatParameter, StereoRig, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Z pos"),
																												  &StereoRig::offsetZ,
																												  &StereoRig::setOffsetZ,
																												  &StereoRig::offsetZChanged);

	//Rotation
	g->addCatProperty<floatParameter, StereoRig, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("X Euler"),
																												 &StereoRig::offsetRotX,
																												 &StereoRig::setOffsetRotX,
																												 &StereoRig::offsetRotXChanged);

	g->addCatProperty<floatParameter, StereoRig, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Y Euler"),
																												  &StereoRig::offsetRotY,
																												  &StereoRig::setOffsetRotY,
																												  &StereoRig::offsetRotYChanged);

	g->addCatProperty<floatParameter, StereoRig, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Z Euler"),
																												  &StereoRig::offsetRotZ,
																												  &StereoRig::setOffsetRotZ,
																												  &StereoRig::offsetRotZChanged);



	ItemDataModel::Category* op = _dataModel->addCategory(tr("Optimized properties"));

	//Optimized Position
	op->addCatProperty<floatParameter, StereoRig, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("X pos"),
																											   &StereoRig::optOffsetX,
																											   nullptr,
																											   &StereoRig::optOffsetChanged);

	op->addCatProperty<floatParameter, StereoRig, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Y pos"),
																											   &StereoRig::optOffsetY,
																											   nullptr,
																											   &StereoRig::optOffsetChanged);

	op->addCatProperty<floatParameter, StereoRig, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Z pos"),
																											   &StereoRig::optOffsetZ,
																											   nullptr,
																											   &StereoRig::optOffsetChanged);

	//Optimized Rotation
	op->addCatProperty<floatParameter, StereoRig, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("X Rot axis"),
																											   &StereoRig::optOffsetRotX,
																											   nullptr,
																											   &StereoRig::optOffsetRotChanged);

	op->addCatProperty<floatParameter, StereoRig, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Y Rot axis"),
																											   &StereoRig::optOffsetRotY,
																											   nullptr,
																											   &StereoRig::optOffsetRotChanged);

	op->addCatProperty<floatParameter, StereoRig, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Z Rot axis"),
																											   &StereoRig::optOffsetRotZ,
																											   nullptr,
																											   &StereoRig::optOffsetRotChanged);

	ItemDataModel::SubItemCollectionManager* im_lm = _dataModel->addCollectionManager(tr("Image Pairs"),
																					  ImagePair::ImagePairClassName,
																					  [] (DataBlock* b) {
																							ImagePair* l = qobject_cast<ImagePair*>(b);
																							if (l != nullptr) {
																								return QString("Rig %1 - %2").arg(l->idImgCam1()).arg(l->idImgCam2());
																							}
																							return tr("Unvalid image landmark");
																						});

	im_lm->addCatProperty<QString, ImagePair, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Camera 1"),
																										   &ImagePair::nameCam1,
																										   nullptr,
																										   &ImagePair::attachedCam1NameChanged);

	im_lm->addCatProperty<QString, ImagePair, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Camera 2"),
																										   &ImagePair::nameCam2,
																										   nullptr,
																										   &ImagePair::attachedCam2NameChanged);
}

ImagePair::ImagePair(StereoRig* parent) :
	DataBlock(parent),
	_id_imgCam1(-1),
	_id_imgCam2(-1)
{
	connect(this, &ImagePair::attachedCam1Changed, this, &ImagePair::attachedCam1NameChanged);
	connect(this, &ImagePair::attachedCam2Changed, this, &ImagePair::attachedCam2NameChanged);
}


QString ImagePair::nameCam1() const {
	Project* p = getProject();

	if (p != nullptr) {
		Image* img = qobject_cast<Image*>(p->getById(idImgCam1()));

		if (img != nullptr) {
			return img->objectName();
		}
	}

	return tr("Unset image");
}
qint64 ImagePair::idImgCam1() const
{
	return _id_imgCam1;
}

void ImagePair::setidImgCam1(const qint64 &id_imgCam1)
{
	if (id_imgCam1 < 0 and _id_imgCam1 >= 0) {
		removeRefered({_id_imgCam1});
		_id_imgCam1 = -1;
		emit attachedCam1Changed(_id_imgCam1);
		return;
	} else if (id_imgCam1 < 0) {
		return;
	}

	int fid = id_imgCam1;

	StereoRig* s_rig = qobject_cast<StereoRig*>(parent());

	if (s_rig != nullptr) {
		QVector<qint64> imPairsIds = s_rig->listTypedSubDataBlocks(this->metaObject()->className());

		for (qint64 ipid : imPairsIds) {
			ImagePair* imp = s_rig->getImagePair(ipid);

			if (imp != nullptr) {
				if (imp->idImgCam1() == fid or imp->idImgCam2() == fid) {
					fid = -1;
					break;
				}
			}
		}
	}

	if (_id_imgCam1 >= 0 and id_imgCam1 == _id_imgCam2) {
		fid = -1;
	}

	if (_id_imgCam1 != fid) {
		if (_id_imgCam1 >= 0) {
			removeRefered({_id_imgCam1});
		}
		_id_imgCam1 = fid;
		if (_id_imgCam1 >= 0) {
			addRefered({_id_imgCam1});
		}
		emit attachedCam1Changed(_id_imgCam1);
		isChanged();
	}
}

QString ImagePair::nameCam2() const {
	Project* p = getProject();

	if (p != nullptr) {
		Image* img = qobject_cast<Image*>(p->getById(idImgCam2()));

		if (img != nullptr) {
			return img->objectName();
		}
	}

	return tr("Unset image");
}
qint64 ImagePair::idImgCam2() const
{
	return _id_imgCam2;
}

void ImagePair::setIdImgCam2(const qint64 &id_imgCam2)
{
	if (id_imgCam2 < 0 and _id_imgCam2 >= 0) {
		removeRefered({_id_imgCam2});
		_id_imgCam2 = -1;
		emit attachedCam2Changed(_id_imgCam2);
		return;
	} else if (id_imgCam2 < 0) {
		return;
	}

	int fid = id_imgCam2;

	StereoRig* s_rig = qobject_cast<StereoRig*>(parent());

	if (s_rig != nullptr) {
		QVector<qint64> imPairsIds = s_rig->listTypedSubDataBlocks(this->metaObject()->className());

		for (qint64 ipid : imPairsIds) {
			ImagePair* imp = s_rig->getImagePair(ipid);

			if (imp != nullptr) {
				if (imp->idImgCam1() == fid or imp->idImgCam2() == fid) {
					fid = -1;
					break;
				}
			}
		}
	}

	if (_id_imgCam1 >= 0 and id_imgCam2 == _id_imgCam1) {
		fid = -1;
	}

	if (_id_imgCam2 != fid) {
		if (_id_imgCam2 >= 0) {
			removeRefered({_id_imgCam2});
		}
		_id_imgCam2 = fid;
		if (_id_imgCam2 >= 0) {
			addRefered({_id_imgCam2});
		}
		emit attachedCam2Changed(_id_imgCam2);
		isChanged();
	}
}

QJsonObject ImagePair::encodeJson() const {

	QJsonObject obj;

	obj.insert("idcam1", idImgCam1());
	obj.insert("idcam2", idImgCam2());

	return obj;
}
void ImagePair::configureFromJson(QJsonObject const& data) {

	if (data.contains("idcam1")) {
		_id_imgCam1 = data.value("idcam1").toInt();
	}
	if (data.contains("idcam2")) {
		_id_imgCam2 = data.value("idcam2").toInt();
	}
}

void ImagePair::referedCleared(QVector<qint64> const& referedId) {

	DataBlock::referedCleared(referedId);

	if (referedId.front() == _id_imgCam1) {
		_id_imgCam1 = -1;
		emit attachedCam1Changed(-1);
	} else if (referedId.front() == _id_imgCam2) {
		_id_imgCam2 = -1;
		emit attachedCam2Changed(-1);
	}

	StereoRig* strg = qobject_cast<StereoRig*>(parent());

	if (strg != nullptr) {
		strg->removeImagePair(this->internalId());
	}
}


StereoRigFactory::StereoRigFactory(QObject* parent) : DataBlockFactory(parent)
{

}

QString StereoRigFactory::TypeDescrName() const {
	return tr("Stereo Rig");
}
DataBlockFactory::FactorizableFlags StereoRigFactory::factorizable() const {
	return DataBlockFactory::RootDataBlock;
}
DataBlock* StereoRigFactory::factorizeDataBlock(Project *parent) const {
	return new StereoRig(parent);
}

QString StereoRigFactory::itemClassName() const {
	return StereoRigClassName();
}
QString StereoRigFactory::StereoRigClassName() {
	StereoRig r;
	return r.metaObject()->className();
}

} // namespace StereoVisionApp
