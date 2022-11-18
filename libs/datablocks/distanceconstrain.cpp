#include "distanceconstrain.h"

#include "itemdatamodel.h"

namespace StereoVisionApp {

DistanceConstrain::DistanceConstrain(Project *parent) :
	DataBlock(parent)
{
	extendDataModel();
}

floatParameter DistanceConstrain::distanceValue() const {
	return _distanceValue;
}
void DistanceConstrain::setDistanceValue(const floatParameter &distanceValue) {

	if (!distanceValue.isApproximatlyEqual(_distanceValue, 1e-4)) {
		_distanceValue = distanceValue;
		emit distanceChanged(distanceValue);
		isChanged();
	}
}

floatParameter DistanceConstrain::optimizedDistanceValue() const {
	return _opt_distanceValue;
}
void DistanceConstrain::setOptimizedDistanceValue(const floatParameter &opt_distanceValue) {
	if (!opt_distanceValue.isApproximatlyEqual(_opt_distanceValue, 1e-4)) {
		_opt_distanceValue = opt_distanceValue;
		emit optDistanceChanged();
		isChanged();
	}
}

void DistanceConstrain::clearOptimized() {
	if (_opt_distanceValue.isSet()) {
		_opt_distanceValue.clearIsSet();
		emit optDistanceChanged();
		isChanged();
	}
}
bool DistanceConstrain::hasOptimizedParameters() const {
	return _opt_distanceValue.isSet();
}

DistanceLandmarksPair* DistanceConstrain::getLandmarksPair(qint64 id) const {
	return qobject_cast<DistanceLandmarksPair*>(getById(id));
}

qint64 DistanceConstrain::insertLandmarksPair(qint64 lm1Id, qint64 lm2Id) {

	QVector<qint64> lmsids = listTypedSubDataBlocks(DistanceLandmarksPair::staticMetaObject.className());

	for (qint64 id : lmsids) {
		DistanceLandmarksPair* lms2 = getLandmarksPair(id);
		if (lms2 != nullptr) {
			if (lms2->getNthLandmarkId(0) == lm1Id and
					lms2->getNthLandmarkId(1) == lm2Id) {
				return -1;
			}
			if (lms2->getNthLandmarkId(0) == lm2Id and
					lms2->getNthLandmarkId(1) == lm1Id) {
				return -1;
			}
		}
	}

	DistanceLandmarksPair* lms2 = new DistanceLandmarksPair(this);

	lms2->stopTrackingChanges(true);

	insertSubItem(lms2);

	if (lms2->internalId() >= 0) {
		lms2->setNthLandmarkId(0, lm1Id);
		lms2->setNthLandmarkId(1, lm2Id);

		emit landmarksPairAdded(lms2->internalId());

		lms2->stopTrackingChanges(false);

		return lms2->internalId();

	} else {
		lms2->clear();
		lms2->deleteLater();
	}

	return -1;
}


QJsonObject DistanceConstrain::encodeJson() const {

	QJsonObject obj;

	obj.insert("distance", floatParameter::toJson(distanceValue()));

	obj.insert("optdistance", floatParameter::toJson(optimizedDistanceValue()));

	QJsonArray arr;

	for(qint64 id : listTypedSubDataBlocks(DistanceLandmarksPair::staticMetaObject.className())) {
		arr.push_back(getLandmarksPair(id)->toJson());
	}

	obj.insert("landmarks_pairs", arr);

	return obj;
}
void DistanceConstrain::configureFromJson(QJsonObject const& data) {


	if (data.contains("distance")) {
		_distanceValue = floatParameter::fromJson(data.value("distance").toObject());
	}

	if (data.contains("optdistance")) {
		_opt_distanceValue = floatParameter::fromJson(data.value("optdistance").toObject());
	}

	if (data.contains("landmarks_pairs")) {
		QJsonArray arr = data.value("landmarks_pairs").toArray();

		for (QJsonValue v : arr) {
			QJsonObject o = v.toObject();

			DistanceLandmarksPair* dlp = new DistanceLandmarksPair(this);
			dlp->setFromJson(o);

			if (dlp->internalId() >= 0) {
				insertSubItem(dlp);
			}
		}
	}
}

void DistanceConstrain::extendDataModel() {

	ItemDataModel::Category* g = _dataModel->addCategory(tr("Geometric properties"));

	//Position
	g->addCatProperty<floatParameter, DistanceConstrain, true, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Distance"),
																												 &DistanceConstrain::distanceValue,
																												 &DistanceConstrain::setDistanceValue,
																												 &DistanceConstrain::distanceChanged);



	ItemDataModel::Category* optCat = _dataModel->addCategory(tr("Optimizer properties"));

	optCat->addCatProperty<bool, DataBlock, false, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Enabled"),
																										  &DataBlock::isEnabled,
																										  &DataBlock::setEnabled,
																										  &DataBlock::isEnabledChanged);

	auto lmNameFunc = [] (DataBlock* b) {
		DistanceLandmarksPair* l = qobject_cast<DistanceLandmarksPair*>(b);
		if (l != nullptr) {
			return QString("Rig %1 - %2").arg(l->getNthLandmarkId(0)).arg(l->getNthLandmarkId(1));
		}
		return tr("Unvalid landmark pair");
	};

	ItemDataModel::SubItemCollectionManager* im_lm = _dataModel->addCollectionManager(tr("Landmarks pairs"),
																					  DistanceLandmarksPair::staticMetaObject.className(),
																					  lmNameFunc);



	im_lm->addCatProperty<QString, DistanceLandmarksPair, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Landmark 1"),
																										   &DistanceLandmarksPair::nameLandmark1,
																										   nullptr,
																										   &DistanceLandmarksPair::attachedLandmark1NameChanged);

	im_lm->addCatProperty<QString, DistanceLandmarksPair, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(tr("Landmark 2"),
																										   &DistanceLandmarksPair::nameLandmark2,
																										   nullptr,
																										   &DistanceLandmarksPair::attachedLandmark2NameChanged);
}


DistanceConstrainFactory::DistanceConstrainFactory(QObject* parent) :
	DataBlockFactory(parent)
{

}

QString DistanceConstrainFactory::TypeDescrName() const {
	return tr("Distance constrain");
}
DataBlockFactory::FactorizableFlags DistanceConstrainFactory::factorizable() const {
	return DataBlockFactory::RootDataBlock;
}
DataBlock* DistanceConstrainFactory::factorizeDataBlock(Project *parent) const {
	return new DistanceConstrain(parent);
}

QString DistanceConstrainFactory::itemClassName() const {
	return DistanceConstrain::staticMetaObject.className();
}

} // namespace StereoVisionApp
