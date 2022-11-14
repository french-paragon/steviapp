#include "localcoordinatesystem.h"

#include "./landmark.h"
#include "./itemdatamodel.h"

namespace StereoVisionApp {

LocalCoordinateSystem::LocalCoordinateSystem(Project *parent) :
	RigidBody(parent)
{
	extendDataModel();
}

qint64 LocalCoordinateSystem::addLandmarkLocalCoordinates(qint64 attachedLandmarkId,
														  floatParameter priorX,
														  floatParameter priorY,
														  floatParameter priorZ) {

	if (getLandmarkLocalCoordinatesByLandmarkId(attachedLandmarkId) != nullptr) {
		return -1;
	}

	LandmarkLocalCoordinates* iml = new LandmarkLocalCoordinates(this);

	iml->stopTrackingChanges(true);

	iml->setXCoord(priorX);
	iml->setYCoord(priorY);
	iml->setZCoord(priorZ);

	insertSubItem(iml);

	if (iml->internalId() >= 0) {
		iml->setAttachedLandmark(attachedLandmarkId);
		emit pointAdded(iml->internalId());

		iml->stopTrackingChanges(false);
		return iml->internalId();
	} else {
		iml->clear();
		iml->deleteLater();
	}

	return -1;

}

LandmarkLocalCoordinates* LocalCoordinateSystem::getLandmarkLocalCoordinates(qint64 local_coordinates_id) const {
	return qobject_cast<LandmarkLocalCoordinates*>(getById(local_coordinates_id));
}

LandmarkLocalCoordinates* LocalCoordinateSystem::getLandmarkLocalCoordinatesByLandmarkId(qint64 landmark_id) const {
	QVector<qint64> lmIds = listTypedSubDataBlocks(LandmarkLocalCoordinates::staticMetaObject.className());

	for (qint64 lm_id : lmIds) {
		LandmarkLocalCoordinates* lmlc = getLandmarkLocalCoordinates(lm_id);

		if (lmlc != nullptr) {
			if (lmlc->attachedLandmarkid() == landmark_id) {
				return lmlc;
			}
		}
	}

	return nullptr;
}

void LocalCoordinateSystem::clearLandmarkLocalCoordinates(qint64 attachedLandmarkId) {

	LandmarkLocalCoordinates* lmlc = getLandmarkLocalCoordinates(attachedLandmarkId);

	if (lmlc != nullptr) {
		clearSubItem(attachedLandmarkId, lmlc->metaObject()->className());
		lmlc = getLandmarkLocalCoordinates(attachedLandmarkId);
		if (lmlc == nullptr) {
			Q_EMIT pointRemoved(attachedLandmarkId);
		}
	}

}
QVector<qint64> LocalCoordinateSystem::getAttachedLandmarksIds() const {

	if (!isInProject()) {
		return {};
	}

	QVector<qint64> lmlcids = listTypedSubDataBlocks(LandmarkLocalCoordinates::staticMetaObject.className());
	QVector<qint64> r;
	r.reserve(lmlcids.size());

	for (qint64 id : lmlcids) {
		LandmarkLocalCoordinates* lmlc = getLandmarkLocalCoordinates(id);

		if (lmlc == nullptr) {
			continue;
		}

		qint64 lmid = lmlc->attachedLandmarkid();

		Landmark* lm = getProject()->getDataBlock<Landmark>(lmid);

		if (lm != nullptr) {
			r.push_back(lmid);
		}
	}

	return r;

}

QJsonObject LocalCoordinateSystem::encodeJson() const {

	QJsonObject obj = RigidBody::encodeJson();

	QJsonArray arr;

	for(qint64 id : listTypedSubDataBlocks(LandmarkLocalCoordinates::staticMetaObject.className())) {
		arr.push_back(getLandmarkLocalCoordinates(id)->toJson());
	}

	obj.insert("Landmarks", arr);

	return obj;

}
void LocalCoordinateSystem::configureFromJson(QJsonObject const& data) {

	RigidBody::configureFromJson(data);

	if (data.contains("Landmarks")) {
		QJsonArray arr = data.value("Landmarks").toArray();

		for (QJsonValue v : arr) {
			QJsonObject o = v.toObject();

			LandmarkLocalCoordinates* iml = new LandmarkLocalCoordinates(this);
			iml->setFromJson(o);

			if (iml->internalId() >= 0) {
				insertSubItem(iml);
			}
		}
	}

}

void LocalCoordinateSystem::extendDataModel() {

	ItemDataModel::Category* g = _dataModel->addCategory(tr("Geometric properties"));

	//Position
	g->addCatProperty<floatParameter, LocalCoordinateSystem, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("X pos"),
																												 &LocalCoordinateSystem::xCoord,
																												 &LocalCoordinateSystem::setXCoord,
																												 &LocalCoordinateSystem::xCoordChanged);

	g->addCatProperty<floatParameter, LocalCoordinateSystem, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Y pos"),
																												 &LocalCoordinateSystem::yCoord,
																												 &LocalCoordinateSystem::setYCoord,
																												 &LocalCoordinateSystem::yCoordChanged);

	g->addCatProperty<floatParameter, LocalCoordinateSystem, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Z pos"),
																												 &LocalCoordinateSystem::zCoord,
																												 &LocalCoordinateSystem::setZCoord,
																												 &LocalCoordinateSystem::zCoordChanged);

	//Rotation
	g->addCatProperty<floatParameter, LocalCoordinateSystem, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("X Euler"),
																												 &LocalCoordinateSystem::xRot,
																												 &LocalCoordinateSystem::setXRot,
																												 &LocalCoordinateSystem::xRotChanged);

	g->addCatProperty<floatParameter, LocalCoordinateSystem, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Y Euler"),
																												 &LocalCoordinateSystem::yRot,
																												 &LocalCoordinateSystem::setYRot,
																												 &LocalCoordinateSystem::yRotChanged);

	g->addCatProperty<floatParameter, LocalCoordinateSystem, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Z Euler"),
																												 &LocalCoordinateSystem::zRot,
																												 &LocalCoordinateSystem::setZRot,
																												 &LocalCoordinateSystem::zRotChanged);

	ItemDataModel::Category* og = _dataModel->addCategory(tr("Optimized geometry"));

	//Position
	og->addCatProperty<float, LocalCoordinateSystem, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("X pos"),
																								  &LocalCoordinateSystem::optXCoord,
																								  &LocalCoordinateSystem::setOptXCoord,
																								  &LocalCoordinateSystem::optPosChanged);

	og->addCatProperty<float, LocalCoordinateSystem, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Y pos"),
																								  &LocalCoordinateSystem::optYCoord,
																								  &LocalCoordinateSystem::setOptYCoord,
																								  &LocalCoordinateSystem::optPosChanged);

	og->addCatProperty<float, LocalCoordinateSystem, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Z pos"),
																								  &LocalCoordinateSystem::optZCoord,
																								  &LocalCoordinateSystem::setOptZCoord,
																								  &LocalCoordinateSystem::optPosChanged);

	//Rotation
	og->addCatProperty<float, LocalCoordinateSystem, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("X Raxis"),
																								  &LocalCoordinateSystem::optXRot,
																								  &LocalCoordinateSystem::setOptXRot,
																								  &LocalCoordinateSystem::optRotChanged);

	og->addCatProperty<float, LocalCoordinateSystem, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Y Raxis"),
																								  &LocalCoordinateSystem::optYRot,
																								  &LocalCoordinateSystem::setOptYRot,
																								  &LocalCoordinateSystem::optRotChanged);

	og->addCatProperty<float, LocalCoordinateSystem, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Z Raxis"),
																								  &LocalCoordinateSystem::optZRot,
																								  &LocalCoordinateSystem::setOptZRot,
																								  &LocalCoordinateSystem::optRotChanged);

	ItemDataModel::SubItemCollectionManager* im_lm = _dataModel->addCollectionManager(tr("Landmarks"),
																					  LandmarkLocalCoordinates::staticMetaObject.className(),
																					  [] (DataBlock* b) {
																							LandmarkLocalCoordinates* l = qobject_cast<LandmarkLocalCoordinates*>(b);
																							if (l != nullptr) {
																								return l->attachedLandmarkName();
																							}
																							return tr("Unvalid image landmark");
																						});


	im_lm->addCatProperty<floatParameter, LandmarkLocalCoordinates, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("X pos"),
																														  &LandmarkLocalCoordinates::xCoord,
																														  &LandmarkLocalCoordinates::setXCoord,
																														  &LandmarkLocalCoordinates::xCoordChanged);

	im_lm->addCatProperty<floatParameter, LandmarkLocalCoordinates, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Y pos"),
																														  &LandmarkLocalCoordinates::yCoord,
																														  &LandmarkLocalCoordinates::setYCoord,
																														  &LandmarkLocalCoordinates::yCoordChanged);

	im_lm->addCatProperty<floatParameter, LandmarkLocalCoordinates, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Z pos"),
																														  &LandmarkLocalCoordinates::zCoord,
																														  &LandmarkLocalCoordinates::setZCoord,
																														  &LandmarkLocalCoordinates::zCoordChanged);


}

LandmarkLocalCoordinates::LandmarkLocalCoordinates(LocalCoordinateSystem* parent) :
	Point3D(parent)
{

}

qint64 LandmarkLocalCoordinates::attachedLandmarkid() const {
	return _attachedLandmarkId;
}
void LandmarkLocalCoordinates::setAttachedLandmark(qint64 id) {

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

	LocalCoordinateSystem* lcs = qobject_cast<LocalCoordinateSystem*>(parent());

	if (lcs != nullptr) {
		QVector<qint64> imLandmarkIds = lcs->listTypedSubDataBlocks(this->metaObject()->className());

		for (qint64 id : imLandmarkIds) {
			LandmarkLocalCoordinates* lmlc = lcs->getLandmarkLocalCoordinates(id);

			if (lmlc != nullptr) {
				if (lmlc->attachedLandmarkid() == t) {
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
QString LandmarkLocalCoordinates::attachedLandmarkName() const {

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
Landmark* LandmarkLocalCoordinates::attachedLandmark() const {
	Project* p = getProject();

	if (p == nullptr) {
		return nullptr;
	}

	Landmark* lm = qobject_cast<Landmark*>(p->getById(attachedLandmarkid()));

	return lm;
}

void LandmarkLocalCoordinates::referedCleared(QVector<qint64> const& referedId) {

	DataBlock::referedCleared(referedId);

	if (referedId.front() == _attachedLandmarkId) {
		_attachedLandmarkId = -1;
		emit attachedLandmarkidChanged(-1);
	}

}

LocalCoordinateSystemFactory::LocalCoordinateSystemFactory(QObject* parent) :
	DataBlockFactory(parent)
{

}

QString LocalCoordinateSystemFactory::TypeDescrName() const {
	return tr("Local coordinate system");
}

DataBlockFactory::FactorizableFlags LocalCoordinateSystemFactory::factorizable() const {
	return DataBlockFactory::RootDataBlock;
}

DataBlock* LocalCoordinateSystemFactory::factorizeDataBlock(Project *parent) const {
	return new LocalCoordinateSystem(parent);
}

QString LocalCoordinateSystemFactory::itemClassName() const {
	return LocalCoordinateSystem::staticMetaObject.className();
}

} // namespace StereoVisionApp
