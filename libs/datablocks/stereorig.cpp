#include "stereorig.h"

#include "itemdatamodel.h"
#include "image.h"

namespace StereoVisionApp {


const QString StereoRig::StereoRigClassName = "StereoVisionApp::StereoRig";
const QString ImagePair::ImagePairClassName = "StereoVisionApp::ImagePair";



ImagePair* ImagePair::getImagePairInProject(Project* project, qint64 cam1ImgId, qint64 cam2ImgId) {

	QVector<qint64> idxs = project->getIdsByClass(StereoRig::staticMetaObject.className());

	for (qint64 id : idxs) {

		StereoRig* rig = project->getDataBlock<StereoRig>(id);

		if (rig == nullptr) {
			continue;
		}

		ImagePair* pair = rig->getPairForImage(cam1ImgId);

		if (pair == nullptr) {
			continue;
		}

		if (pair->idImgCam1() == cam1ImgId and pair->idImgCam2() == cam2ImgId) {
			return pair;
		}

		if (pair->idImgCam1() == cam2ImgId and pair->idImgCam2() == cam1ImgId) {
			return pair;
		}
	}

	return nullptr;

}

StereoRig::StereoRig(Project *parent) :
	RigidBody(parent)
{
	extendDataModel();
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

QJsonObject StereoRig::getJsonRepresentation() const {

	QJsonObject obj;

	obj.insert("ofsx", floatParameter::toJson(xCoord()));
	obj.insert("ofsy", floatParameter::toJson(yCoord()));
	obj.insert("ofsz", floatParameter::toJson(zCoord()));

	obj.insert("ofsrx", floatParameter::toJson(xRot()));
	obj.insert("ofsry", floatParameter::toJson(yRot()));
	obj.insert("ofsrz", floatParameter::toJson(zRot()));

	obj.insert("oofs", floatParameterGroup<3>::toJson(optPos()));

	obj.insert("oofsr", floatParameterGroup<3>::toJson(optRot()));

	return obj;
}

void StereoRig::setParametersFromJsonRepresentation(QJsonObject const& rep) {

	if (rep.contains("ofsx")) {
		setXCoord(floatParameter::fromJson(rep.value("ofsx").toObject()));
	}
	if (rep.contains("ofsy")) {
		setYCoord(floatParameter::fromJson(rep.value("ofsy").toObject()));
	}
	if (rep.contains("ofsz")) {
		setZCoord(floatParameter::fromJson(rep.value("ofsz").toObject()));
	}

	if (rep.contains("ofsrx")) {
		setXRot(floatParameter::fromJson(rep.value("ofsrx").toObject()));
	}
	if (rep.contains("ofsry")) {
		setYRot(floatParameter::fromJson(rep.value("ofsry").toObject()));
	}
	if (rep.contains("ofsry")) {
		setZRot(floatParameter::fromJson(rep.value("ofsrz").toObject()));
	}

	if (rep.contains("oofs")) {
		setOptPos(floatParameterGroup<3>::fromJson(rep.value("oofs").toObject()));
	}

	if (rep.contains("oofsr")) {
		setOptRot(floatParameterGroup<3>::fromJson(rep.value("oofsr").toObject()));
	}
}

QJsonObject StereoRig::encodeJson() const {

	QJsonObject obj;

	obj.insert("ofsx", floatParameter::toJson(xCoord()));
	obj.insert("ofsy", floatParameter::toJson(yCoord()));
	obj.insert("ofsz", floatParameter::toJson(zCoord()));

	obj.insert("ofsrx", floatParameter::toJson(xRot()));
	obj.insert("ofsry", floatParameter::toJson(yRot()));
	obj.insert("ofsrz", floatParameter::toJson(zRot()));

	obj.insert("oofs", floatParameterGroup<3>::toJson(optPos()));

	obj.insert("oofsr", floatParameterGroup<3>::toJson(optRot()));

	QJsonArray arr;

	for(qint64 id : listTypedSubDataBlocks(ImagePair::ImagePairClassName)) {
		arr.push_back(getImagePair(id)->toJson());
	}

	obj.insert("ImgsPairs", arr);

	return obj;

}

void StereoRig::configureFromJson(QJsonObject const& data) {

	if (data.contains("ofsx")) {
		setXCoord(floatParameter::fromJson(data.value("ofsx").toObject()));
	}
	if (data.contains("ofsy")) {
		setYCoord(floatParameter::fromJson(data.value("ofsy").toObject()));
	}
	if (data.contains("ofsz")) {
		setZCoord(floatParameter::fromJson(data.value("ofsz").toObject()));
	}

	if (data.contains("ofsrx")) {
		setXRot(floatParameter::fromJson(data.value("ofsrx").toObject()));
	}
	if (data.contains("ofsry")) {
		setYRot(floatParameter::fromJson(data.value("ofsry").toObject()));
	}
	if (data.contains("ofsry")) {
		setZRot(floatParameter::fromJson(data.value("ofsrz").toObject()));
	}

	if (data.contains("oofs")) {
		setOptPos(floatParameterGroup<3>::fromJson(data.value("oofs").toObject()));
	}

	if (data.contains("oofsr")) {
		setOptRot(floatParameterGroup<3>::fromJson(data.value("oofsr").toObject()));
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
	g->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("X pos"),
																												 &RigidBody::xCoord,
																												 &RigidBody::setXCoord,
																												 &RigidBody::xCoordChanged);

	g->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Y pos"),
																												  &RigidBody::yCoord,
																												  &RigidBody::setYCoord,
																												  &RigidBody::yCoordChanged);

	g->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Z pos"),
																												  &StereoRig::zCoord,
																												  &StereoRig::setZCoord,
																												  &StereoRig::zCoordChanged);

	//Rotation
	g->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("X Euler"),
																												 &RigidBody::xRot,
																												 &RigidBody::setXRot,
																												 &RigidBody::xRotChanged);

	g->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Y Euler"),
																												  &RigidBody::yRot,
																												  &RigidBody::setYRot,
																												  &RigidBody::yRotChanged);

	g->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Z Euler"),
																												  &RigidBody::zRot,
																												  &RigidBody::setZRot,
																												  &RigidBody::zRotChanged);



	ItemDataModel::Category* optCat = _dataModel->addCategory(tr("Optimizer properties"));

	optCat->addCatProperty<bool, DataBlock, false, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Enabled"),
																										  &DataBlock::isEnabled,
																										  &DataBlock::setEnabled,
																										  &DataBlock::isEnabledChanged);

	optCat->addCatProperty<bool, DataBlock, false, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Fixed"),
																										  &DataBlock::isFixed,
																										  &DataBlock::setFixed,
																										  &DataBlock::isFixedChanged);



	ItemDataModel::Category* op = _dataModel->addCategory(tr("Optimized properties"));

	//Optimized Position
    op->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("X pos"),
																											   &RigidBody::optXCoord,
																											   nullptr,
																											   &RigidBody::optPosChanged);

    op->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Y pos"),
																											   &RigidBody::optYCoord,
																											   nullptr,
																											   &RigidBody::optPosChanged);

    op->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Z pos"),
																											   &RigidBody::optZCoord,
																											   nullptr,
																											   &RigidBody::optPosChanged);

	//Optimized Rotation
    op->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("X Rot axis"),
																											   &RigidBody::optXRot,
																											   nullptr,
																											   &RigidBody::optRotChanged);

    op->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Y Rot axis"),
																											   &RigidBody::optYRot,
																											   nullptr,
																											   &RigidBody::optRotChanged);

    op->addCatProperty<floatParameter, RigidBody, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Z Rot axis"),
																											   &RigidBody::optZRot,
																											   nullptr,
																											   &RigidBody::optRotChanged);

	auto imPairGetName = [] (DataBlock* b) {
		ImagePair* l = qobject_cast<ImagePair*>(b);
		if (l != nullptr) {
			return QString("Rig %1 - %2").arg(l->idImgCam1()).arg(l->idImgCam2());
		}
		return tr("Unvalid image landmark");
	};

	auto stereoRigDeleteImPair = [] (DataBlock* b, qint64 id) {
		StereoRig* rig = qobject_cast<StereoRig*>(b);

		if (rig != nullptr) {
			rig->removeImagePair(id);
		}
	};

	ItemDataModel::SubItemCollectionManager* im_lm = _dataModel->addCollectionManager(tr("Image Pairs"),
																					  ImagePair::ImagePairClassName,
																					  imPairGetName,
																					  stereoRigDeleteImPair);

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

StereoRig* ImagePair::getStereoRig() const {
	return qobject_cast<StereoRig*>(parent());
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
