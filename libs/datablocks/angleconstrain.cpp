#include "angleconstrain.h"

#include "itemdatamodel.h"

namespace StereoVisionApp {

AngleConstrain::AngleConstrain(Project *parent) :
	DataBlock(parent)
{
	extendDataModel();
}

floatParameter AngleConstrain::angleValue() const
{
	return _angleValue;
}

void AngleConstrain::setAngleValue(const floatParameter &angleValue)
{

	if (!angleValue.isApproximatlyEqual(_angleValue, 1e-4)) {
		_angleValue = angleValue;
		emit angleChanged(angleValue);
		isChanged();
	}
}

floatParameter AngleConstrain::optimizedAngleValue() const
{
	return _opt_angleValue;
}

void AngleConstrain::setOptimizedAngleValue(const floatParameter &opt_angleValue)
{
	if (!opt_angleValue.isApproximatlyEqual(_opt_angleValue, 1e-4)) {
		_opt_angleValue = opt_angleValue;
		emit optAngleChanged();
		isChanged();
	}

}

void AngleConstrain::clearOptimized() {
	if (_opt_angleValue.isSet()) {
		_opt_angleValue.clearIsSet();
		emit optAngleChanged();
		isChanged();
	}
}
bool AngleConstrain::hasOptimizedParameters() const {
	return _opt_angleValue.isSet();
}

AngleLandmarksTriplets* AngleConstrain::getLandmarksTriplet(qint64 id) const {
	return qobject_cast<AngleLandmarksTriplets*>(getById(id));
}
qint64 AngleConstrain::insertLandmarksTriplet(qint64 lm1Id, qint64 lm2Id, qint64 lm3Id) {

	QVector<qint64> lmsids = listTypedSubDataBlocks(AngleLandmarksTriplets::staticMetaObject.className());

	for (qint64 id : lmsids) {
		LandmarksTriplet* lms3 = getLandmarksTriplet(id);
		if (lms3 != nullptr) {
			if (lms3->getNthLandmarkId(0) == lm1Id and
					lms3->getNthLandmarkId(1) == lm2Id and
					lms3->getNthLandmarkId(2) == lm2Id) {
				return -1;
			}
		}
	}

	AngleLandmarksTriplets* lms3 = new AngleLandmarksTriplets(this);

	lms3->stopTrackingChanges(true);

	insertSubItem(lms3);

	if (lms3->internalId() >= 0) {
		lms3->setNthLandmarkId(0, lm1Id);
		lms3->setNthLandmarkId(1, lm2Id);
		lms3->setNthLandmarkId(2, lm3Id);

		emit landmarksTripletAdded(lms3->internalId());

		lms3->stopTrackingChanges(false);

		return lms3->internalId();

	} else {
		lms3->clear();
		lms3->deleteLater();
	}

	return -1;
}
bool AngleConstrain::removeLandmarkTriplet(qint64 id) {
	LandmarksTriplet* lms3 = getLandmarksTriplet(id);
	if (lms3 != nullptr) {
		clearSubItem(id, AngleLandmarksTriplets::staticMetaObject.className());
		return true;
	}
	return false;
}

QJsonObject AngleConstrain::encodeJson() const {

	QJsonObject obj;

	obj.insert("angle", floatParameter::toJson(angleValue()));

	obj.insert("optangle", floatParameter::toJson(optimizedAngleValue()));

	QJsonArray arr;

	for(qint64 id : listTypedSubDataBlocks(AngleLandmarksTriplets::staticMetaObject.className())) {
		arr.push_back(getLandmarksTriplet(id)->toJson());
	}

	obj.insert("landmarks_triplets", arr);

	return obj;

}
void AngleConstrain::configureFromJson(QJsonObject const& data) {


	if (data.contains("angle")) {
		_angleValue = floatParameter::fromJson(data.value("angle").toObject());
	}

	if (data.contains("optangle")) {
		_opt_angleValue = floatParameter::fromJson(data.value("optangle").toObject());
	}

	if (data.contains("landmarks_triplets")) {
		QJsonArray arr = data.value("landmarks_triplets").toArray();

		for (QJsonValue v : arr) {
			QJsonObject o = v.toObject();

			AngleLandmarksTriplets* imp = new AngleLandmarksTriplets(this);
			imp->setFromJson(o);

			if (imp->internalId() >= 0) {
				insertSubItem(imp);
			}
		}
	}
}

void AngleConstrain::extendDataModel() {


	ItemDataModel::Category* g = _dataModel->addCategory(tr("Geometric properties"));

	//Position
	g->addCatProperty<floatParameter, AngleConstrain, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Angle"),
																												 &AngleConstrain::angleValue,
																												 &AngleConstrain::setAngleValue,
																												 &AngleConstrain::angleChanged);

	auto lmNameFunc = [] (DataBlock* b) {
		AngleLandmarksTriplets* l = qobject_cast<AngleLandmarksTriplets*>(b);
		if (l != nullptr) {
			return QString("Rig %1 - %2 - %3").arg(l->getNthLandmarkId(0)).arg(l->getNthLandmarkId(1)).arg(l->getNthLandmarkId(2));
		}
		return tr("Unvalid landmark triplet");
	};

	auto lmDeleteTriplet = [] (DataBlock* b, qint64 id) {
		AngleConstrain* constr = qobject_cast<AngleConstrain*>(b);

		if (constr != nullptr) {
			return constr->removeLandmarkTriplet(id);
		}

		return false;
	};

	ItemDataModel::SubItemCollectionManager* im_lm = _dataModel->addCollectionManager(tr("Landmarks triplets"),
																					  AngleLandmarksTriplets::staticMetaObject.className(),
																					  lmNameFunc,
																					  lmDeleteTriplet);



	im_lm->addCatProperty<QString, AngleLandmarksTriplets, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Landmark 1"),
																										   &AngleLandmarksTriplets::nameLandmark1,
																										   nullptr,
																										   &AngleLandmarksTriplets::attachedLandmark1NameChanged);

	im_lm->addCatProperty<QString, AngleLandmarksTriplets, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Landmark 2"),
																										   &AngleLandmarksTriplets::nameLandmark2,
																										   nullptr,
																										   &AngleLandmarksTriplets::attachedLandmark2NameChanged);

	im_lm->addCatProperty<QString, AngleLandmarksTriplets, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Landmark 3"),
																										   &AngleLandmarksTriplets::nameLandmark3,
																										   nullptr,
																										   &AngleLandmarksTriplets::attachedLandmark3NameChanged);
}


AngleConstrainFactory::AngleConstrainFactory(QObject* parent) :
	DataBlockFactory(parent)
{

}

QString AngleConstrainFactory::TypeDescrName() const {
	return tr("Angle constrain");
}
DataBlockFactory::FactorizableFlags AngleConstrainFactory::factorizable() const {
	return RootDataBlock;
}
DataBlock* AngleConstrainFactory::factorizeDataBlock(Project *parent) const {
	return new AngleConstrain(parent);
}

QString AngleConstrainFactory::itemClassName() const {
	return AngleConstrain::staticMetaObject.className();
}

} // namespace StereoVisionApp
