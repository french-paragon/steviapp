#include "project.h"

#include <QUrl>
#include <QFile>

#include <QColor>

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>

#include <algorithm>

#include "./itemdatamodel.h"
#include "./io/jsonconversionsutils.h"

namespace StereoVisionApp {

const QString Project::PROJECT_FILE_EXT = ".steviapproj";

ProjectFactory* ProjectFactory::_defaultProjectFactory = new ProjectFactory();

ProjectFactory::ProjectFactory (QObject* parent) : QObject(parent)
{

}

Project* ProjectFactory::createProject(QObject* parent) const {

	Project* p = new Project(parent);

	for (DataBlockFactory* f : _factories) {
		p->addType(f);
	}

	return p;

}
bool ProjectFactory::addType(DataBlockFactory* factory) {

	if (_installedTypes.contains(factory->itemClassName())) {
		return false;
	}

	_installedTypes.push_back(factory->itemClassName());
	_factories.push_back(factory);

	return true;

}
bool ProjectFactory::hasType(QString const& blockType) const {
	return _installedTypes.contains(blockType);
}
QString ProjectFactory::typeDescr(QString const& blockType) const {


	int id = _installedTypes.indexOf(blockType);

	if (id < 0) {
		return tr("NoDescr");
	}

	return _factories[id]->TypeDescrName();
}

ProjectFactory& ProjectFactory::defaultProjectFactory() {
	return *_defaultProjectFactory;
}

QVector<QString> ProjectFactory::installedTypes() const
{
    return _installedTypes;
}


Project::Project(QObject* parent) :
	QAbstractItemModel(parent),
    _source(""),
    _defaultProjectCRS(""),
    _hasLocalCoordinateFrame(false)
{
	connect(this, &Project::dataChanged, this, &Project::projectChanged);
	connect(this, &Project::rowsAboutToBeInserted, this, &Project::projectChanged);
	connect(this, &Project::rowsAboutToBeMoved, this, &Project::projectChanged);

	connect(this, &Project::projectChanged, this, &Project::projectDataChanged);
}

bool Project::load(QString const& inFile) {

	QString fileName = (inFile.startsWith("file:")) ? QUrl(inFile).toLocalFile() : inFile;
	QFile opsFile(fileName);

	opsFile.open(QIODevice::ReadOnly);
	QByteArray data = opsFile.readAll();
	opsFile.close();

	QJsonParseError errors;
	QJsonDocument doc = QJsonDocument::fromJson(data, &errors);

	if(errors.error != QJsonParseError::NoError){
		return false;
	}

	if (!doc.isObject()) {
		return false;
	}

	setSource(inFile);

	beginResetModel();

	clearImpl();

	QJsonObject proj = doc.object();

	for (QString itemClass : _dataBlocksFactory.keys()) {

        if (itemClass == "metadata") {
            continue; //reserved keyword
        }

		QJsonArray items = proj.value(itemClass).toArray();

		for (QJsonValue v : items) {
			DataBlock* block = _dataBlocksFactory[itemClass]->factorizeDataBlock(this);
			block->setFromJson(v.toObject());

            QString proxyClass = _dataBlocksFactory[itemClass]->proxyClassName();

			if (block->internalId() >= 0) {
				_itemCache.insert(block->internalId(), block);
                _idsByTypes[itemClass].append(block->internalId());
                _idsByProxyTypes[proxyClass].append(block->internalId());

				connect(block, &DataBlock::datablockChanged, this, &Project::projectDataChanged);
			}
		}
	}

    _hasLocalCoordinateFrame = false;
    _defaultProjectCRS = "";
    if (proj.contains("metadata")) {

        QJsonObject metadata = proj.value("metadata").toObject();

        if (metadata.contains("local_transform")) {
            QJsonObject local = metadata.value("local_transform").toObject();

            auto ecef2local = json2affineTransform<double>(local);
            if (ecef2local.has_value()) {
                _ecef2local = ecef2local.value();
                _hasLocalCoordinateFrame = true;
            }
        }

        if (metadata.contains("defaultProjectCRS")) {
            _defaultProjectCRS = metadata.value("defaultProjectCRS").toString();
        }

    }

	endResetModel();

	return true;
}

bool Project::save() {
	if (source().isEmpty()) {
		return false;
	}

	return save(source());
}

bool Project::save(QString const& outFile) {

	QJsonObject proj;

	for (const QString &itemClass : _idsByTypes.keys()) {

		QJsonArray items;
		QVector<qint64> ids = _idsByTypes.value(itemClass);

		for (qint64 id : ids) {
			DataBlock* block = getById(id);
			QJsonObject bObj = block->toJson();

			items.push_back(bObj);
		}

		proj.insert(itemClass, items);
	}

    QJsonObject metadata;

    if (_hasLocalCoordinateFrame) {
        QJsonObject localTransform = affineTransform2json(_ecef2local);

        metadata.insert("local_transform", localTransform);
    }

    if (!_defaultProjectCRS.isEmpty()) {
        metadata.insert("defaultProjectCRS", _defaultProjectCRS);
    }

    proj.insert("metadata", metadata);

	QJsonDocument doc;

	doc.setObject(proj);
	QByteArray datas = doc.toJson();

	QString fileName = (outFile.startsWith("file:")) ? QUrl(outFile).toLocalFile() : outFile;

	if (!fileName.endsWith(PROJECT_FILE_EXT)) {
		fileName += PROJECT_FILE_EXT;
	}

	QFile out(fileName);

	if(!out.open(QIODevice::WriteOnly)){
		return false;
	}

	qint64 w_stat = out.write(datas);
	out.close();

	if(w_stat < 0){
		return false;
	}

	_source = outFile;
	return true;

}

qint64 Project::createDataBlock(const char* classname) {

	QString cn(classname);

	if (!_dataBlocksFactory.contains(cn)) {
		return -1;
	}

	DataBlock* db = _dataBlocksFactory.value(cn)->factorizeDataBlock(this);
	db->_internalId = nextAvailableId();
	db->setObjectName(_dataBlocksFactory.value(cn)->TypeDescrName() + QVariant(db->_internalId).toString());

    QString proxy = _dataBlocksFactory.value(cn)->proxyClassName();

	if (insertImpls(db)) {
        QModelIndex parent = createIndex(_proxyTypes.indexOf(proxy), 0);
        int row = countTypeInstances(proxy);
        beginInsertRows(parent, row, row);
		_idsByTypes[cn].append(db->_internalId);
        _idsByProxyTypes[proxy].append(db->_internalId);
		connect(db, &DataBlock::datablockChanged, this, &Project::projectDataChanged);
		endInsertRows();
		return db->_internalId;
	}

	delete db;
	return -1;

}
DataBlock* Project::getById(qint64 internalId) const {
	if (_itemCache.contains(internalId)) {
		return _itemCache.value(internalId);
	}

	Project* nonConst = const_cast<Project*>(this);

	if (nonConst->loadImpls(internalId)) {
		return _itemCache.value(internalId, nullptr);
	}

	return nullptr;
}

DataBlock* Project::getByName(QString const& name, QString const& typeHint) const {

	auto types = _idsByTypes.keys();
	for (QString const& type : qAsConst(types)) {

		if (!typeHint.isEmpty()) {
			if (typeHint != type) {
				continue;
			}
		}

		for (qint64 id : _idsByTypes.value(type)) {
			DataBlock* cand = getById(id);

			if (cand != nullptr) {
				if (cand->objectName() == name) {
					return cand;
				}
			}
		}
	}

	return nullptr;

}

DataBlock* Project::getByUrl(QVector<qint64> const& internalUrl) const {
	QVector<qint64> url = internalUrl;
	qint64 base = url.takeFirst();

	DataBlock* b = getById(base);

	if (b == nullptr) {
		return nullptr;
	}

	while (url.size() > 0) {
		base = url.takeFirst();
		b = b->getById(base);

		if (b == nullptr) {
			return nullptr;
		}
	}

	return b;
}

DataBlockFactory* Project::getFactoryForClass(QString cName) const {
	return _dataBlocksFactory.value(cName, nullptr);
}

bool Project::clearById(qint64 internalId) {

	DataBlock* db = getById(internalId);
	QString blockClass = db->metaObject()->className();

    QString proxyClass = _dataBlocksFactory[blockClass]->proxyClassName();

    int ItemRow = _idsByProxyTypes.value(proxyClass).indexOf(internalId);

	if (ItemRow < 0) {
		return false;
	}

    QModelIndex parentIndex = createIndex(_proxyTypes.indexOf(proxyClass), 0);

	beginRemoveRows(parentIndex, ItemRow, ItemRow);

	db->clear();
	_itemCache.remove(internalId);
	_idsByTypes[blockClass].remove(ItemRow);
    _idsByProxyTypes[proxyClass].remove(ItemRow);
	disconnect(db, &DataBlock::datablockChanged, this, &Project::projectDataChanged);
	db->deleteLater();

	endRemoveRows();

	return true;
}

bool Project::addType(DataBlockFactory* factory) {
	if (!(factory->factorizable() & DataBlockFactory::RootDataBlock)) {
		return false; //insert only factory able to creat root items.
	}

	if(_dataBlocksFactory.contains(factory->itemClassName())) {
		return true;
	}

	_dataBlocksFactory.insert(factory->itemClassName(), factory);

	if (_dataBlocksFactory.contains(factory->itemClassName())) {
		_installedTypes.push_back(factory->itemClassName());
        if (!_proxyTypes.contains(factory->proxyClassName())) {
            _proxyTypes.push_back(factory->proxyClassName());
        }
		return true;
	}

	return false;
}

QVector<qint64> Project::getIds() const {
	QVector<qint64> r;

	for (QVector<qint64> const& v : _idsByTypes.values()) {
		r += v;
	}

	return r;
}
QVector<qint64> Project::getIdsByClass(QString const& className) const {
	return _idsByTypes.value(className);
}
int Project::countTypeInstances(QString type) const {
	return _idsByTypes.value(type).size();
}


QModelIndex Project::indexOfClass(QString const& className) {
    int row = _proxyTypes.indexOf(className);

	if (row < 0) {
		return QModelIndex();
	}

	return createIndex(row, 0);
}

QModelIndex Project::indexOfClassInstance(QString const& className, qint64 id) {
    if (!_proxyTypes.contains(className)) {
		return QModelIndex();
	}

    int row = _idsByProxyTypes.value(className).indexOf(id);

	if (row < 0) {
		return QModelIndex();
	}

	DataBlockFactory* factory = _dataBlocksFactory.value(className);
	return createIndex(row, 0, factory);
}

QModelIndex Project::index(int row, int column, const QModelIndex &parent) const {

	if (parent == QModelIndex()) { //root items
        if (row < 0 or row >= _proxyTypes.count()) {
			return QModelIndex();
		}
		return createIndex(row, column);
	}

	//child items
    if (parent.row() < 0 or parent.row() >= _proxyTypes.count()) {
		return QModelIndex();
	}
    if (row < 0 or row >= _idsByProxyTypes[_proxyTypes.at(parent.row())].count()) {
		return QModelIndex();
	}
    DataBlockFactory* factory = _dataBlocksFactory.value(_proxyTypes.at(parent.row()));
	return createIndex(row, column, factory);

}
QModelIndex Project::parent(const QModelIndex &index) const {
	DataBlockFactory* factory = static_cast<DataBlockFactory*>(index.internalPointer());

	if (factory == nullptr) {
		return QModelIndex();
	}

	int id = -1;

	QMapIterator<QString, DataBlockFactory*> i(_dataBlocksFactory);
	while(i.hasNext()) {
		i.next();
		if (i.value() == factory) {
            id = _proxyTypes.indexOf(i.key());
			break;
		}
	}

	if (id < 0) {
		return QModelIndex();
	}

	return createIndex(id, 0);
}
int Project::rowCount(const QModelIndex &parent) const {
	if (parent == QModelIndex()) {
		return _dataBlocksFactory.values().size();
	}

	if (parent.parent() == QModelIndex()) {
        QString typeId = _proxyTypes.at(parent.row());
        return _idsByProxyTypes[typeId].size();
	}

	return 0;
}
int Project::columnCount(const QModelIndex &) const {
	return 1;
}
QVariant Project::data(const QModelIndex &index, int role) const {

	if (index == QModelIndex()) {
		return QVariant();
	}

	if (index.parent() == QModelIndex()) {

        if (index.row() < 0 or index.row() >= _proxyTypes.size()) {
			return QVariant();
		}

		switch (role) {
		case Qt::DisplayRole:
            return _dataBlocksFactory.value(_proxyTypes.at(index.row()))->TypeDescrName();
		case ClassRole:
            return _dataBlocksFactory.value(_proxyTypes.at(index.row()))->itemClassName();
		default:
			return QVariant();
		}

	} else if (index.parent().parent() == QModelIndex()) {
        QString type = _proxyTypes.at(index.parent().row());
        QVector<qint64> const& idxs = _idsByProxyTypes[type];

        if (index.row() < 0 or index.row() >= idxs.size()) {
			return QVariant();
		}

        DataBlock* datablock = getById(idxs.at(index.row()));

		switch (role) {
		case Qt::DisplayRole:
		case Qt::EditRole:
			return datablock->objectName();
		case Qt::ForegroundRole:
			return (datablock->isEnabled()) ? QColor(0,0,0) : QColor(120, 120, 120);
		case ClassRole:
			return datablock->metaObject()->className();
		case IdRole:
            return _idsByProxyTypes.value(type).at(index.row());
		default:
			return QVariant();
		}
	}

	return QVariant();
}

bool Project::setData(const QModelIndex &index, const QVariant &value, int role) {

	if (role != Qt::EditRole) {
		return false;
	}

	if (!value.canConvert(qMetaTypeId<QString>())) {
		return false;
	}

	if (index == QModelIndex()) {
		return false;
	}

	if (index.parent() == QModelIndex()) {
		return false;
	}

	if (index.parent().parent() == QModelIndex()) {
        QString type = _proxyTypes.at(index.parent().row());

		if (index.row() < 0 or index.row() >= countTypeInstances(type)) {
			return false;
		}

        getById(_idsByProxyTypes.value(type).at(index.row()))->setObjectName(value.toString());
		emit dataChanged(index, index, {Qt::DisplayRole});
	}

	return false;
}
Qt::ItemFlags Project::flags(const QModelIndex &index) const {
	if(index != QModelIndex() and index.parent() != QModelIndex() and index.parent().parent() == QModelIndex()) {
		return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
	} else if (index != QModelIndex() and index.parent() == QModelIndex()) {
		return QAbstractItemModel::flags(index) & ~Qt::ItemIsSelectable;
	}

	return QAbstractItemModel::flags(index);
}

void Project::clear() {
	beginResetModel();
	clearImpl();
	_source.clear();
	endResetModel();
}
void Project::clearOptimized() {
	beginResetModel();

	for (qint64 id : getIds()) {
		DataBlock* block = getById(id);

		if (block != nullptr) {
			block->deepClearOptimized();
		}
	}

	endResetModel();
	Q_EMIT projectChanged();
}

void Project::clearOptimizedState() {
	beginResetModel();

	for (qint64 id : getIds()) {
		DataBlock* block = getById(id);

		if (block != nullptr) {
			if (block->optimizationStep() >= DataBlock::Optimised) {
				block->setOptimizationStep(DataBlock::Initialized);
			}
		}
	}

	endResetModel();
	Q_EMIT projectChanged();
}

QString Project::source() const {
	return _source;
}
void Project::setSource(QString const& s) {
	_source = s;
}

bool Project::hasSolution() const {

	for (qint64 id : getIds()) {
		DataBlock* block = getById(id);

		if (block != nullptr) {
			if (block->hierarchyHasOptimizedParameters()) {
				return true;
			}
		}
	}

	return false;
}

void Project::clearImpl() {

	for (qint64 id : _itemCache.keys()) {
		DataBlock* db = getById(id);
		QString blockClass = db->metaObject()->className();
        QString proxyClass = _dataBlocksFactory[blockClass]->proxyClassName();

		int ItemRow = _idsByTypes.value(blockClass).indexOf(id);

		if (ItemRow < 0) {
			continue;
		}

        int ProxyRow = _idsByProxyTypes.value(proxyClass).indexOf(id);

		db->clear();
		_itemCache.remove(id);
		_idsByTypes[blockClass].remove(ItemRow);
        _idsByProxyTypes[proxyClass].remove(ProxyRow);
		db->deleteLater();
	}

}

void Project::setDataBlockId(DataBlock* b, qint64 id) const{
	if (b->getProject() == this) {
		b->_internalId = id;
	}
}

bool Project::loadImpls(qint64) {
	return false;
}

bool Project::insertImpls(DataBlock* db) {
	_itemCache.insert(db->_internalId, db);
	return true;
}

qint64 Project::nextAvailableId() const {
	qint64 id = 0;

	QVector<qint64> ids = getIds();

	std::sort(ids.begin(), ids.end());

	for (qint64 i : ids) {
		if (id < i) {
			return id;
		}
		id = i+1;
	}

	return id;
}

const QVector<QString> &Project::installedTypes() const
{
	return _installedTypes;
}


DataBlockFactory::DataBlockFactory(QObject* parent) : QObject(parent) {

}
DataBlock* DataBlockFactory::factorizeDataBlock(Project *) const {
	return nullptr;
}
DataBlock* DataBlockFactory::factorizeDataBlock(DataBlock *) const {
	return nullptr;
}

QString DataBlockFactory::itemClassName() const {

	DataBlock* db;

	if (factorizable() & DataBlockFactory::RootDataBlock) {
		db = factorizeDataBlock(static_cast<Project*>(nullptr));
	} else {
		db = factorizeDataBlock(static_cast<DataBlock*>(nullptr));
	}

	if (db == nullptr) {
		return "";
	}

	QString r = db->metaObject()->className();

	delete db;

	return r;
}

QString DataBlockFactory::proxyClassName() const {
    return itemClassName();
}

DataBlock::DataBlock(Project *parent) :
	QObject(parent),
	_internalId(-1),
	_hasChanges(false),
	_blockChanges(false),
	_opt_step(Unset),
	_isFixed(false),
	_isEnabled(true)
{
	connect(this, &QObject::objectNameChanged, this, &DataBlock::dataBlockNameChanged);
	buildDataModel();
}

DataBlock::DataBlock(DataBlock *parent) :
	QObject(parent),
	_internalId(-1),
	_hasChanges(false),
	_blockChanges(false),
	_opt_step(Unset),
	_isFixed(false),
	_isEnabled(true)
{
	connect(this, &QObject::objectNameChanged, this, &DataBlock::dataBlockNameChanged);
	_dataModel = nullptr;
}

DataBlock::~DataBlock() {
	for (DataBlockReference* r : _nonDatablockReferences) {
		r->_referedDatablock = nullptr;
	}
}

Project* DataBlock::getProject() const {

	Project* p = qobject_cast<Project*>(parent());

	if (p != nullptr) {
		return p;
	}

	DataBlock* b = qobject_cast<DataBlock*>(parent());

	if (b != nullptr) {
		return b->getProject();
	}

	return nullptr;

}

bool DataBlock::isInProject() const{
	return getProject() != nullptr and _internalId >= 0;
}
bool DataBlock::isRootItem() {
	return isInProject() and (qobject_cast<Project*>(parent()) != nullptr);
}
bool DataBlock::isChildItem() {
	return qobject_cast<DataBlock*>(parent()) != nullptr;
}

bool DataBlock::hasChanges() const {
	return _hasChanges;
}

void DataBlock::clearIsChanged() {
	if (hasChanges()) {
		_hasChanges = false;
		emit isChangedStatusChanged(false);
	}
}

QVector<qint64> DataBlock::internalUrl() const {
	Project* p = qobject_cast<Project*>(parent());

	if (p != nullptr) {
		return {_internalId};
	}

	DataBlock* b = qobject_cast<DataBlock*>(parent());

	if (b != nullptr) {
		return b->internalUrl() << _internalId;
	}

	return {};
}

QStringList DataBlock::subTypes() const {
	return _idBySubTypes.keys();
}
DataBlock* DataBlock::getById(qint64 internalId) const {
	return _itemCache.value(internalId, nullptr);
}

QVector<qint64> DataBlock::listAllSubDataBlocks() const {
	QVector<qint64> r;

	for (QVector<qint64> const& v : _idBySubTypes.values()) {
		r += v;
	}

	return r;
}
QVector<qint64> DataBlock::listTypedSubDataBlocks(QString const& type) const {
	return _idBySubTypes.value(type);
}

void DataBlock::insertSubItem(DataBlock* sub) {

	if (_itemCache.contains(sub->internalId())) {
		return;
	}

	DataBlock* p = qobject_cast<DataBlock*>(sub->parent());

	if (p != this) {
		return;
	}

	if (sub->internalId() < 0) {
		qint64 id = nextAvailableId();
		sub->_internalId = id;
	}

	_itemCache.insert(sub->internalId(), sub);

	QString className = sub->metaObject()->className();

	if (!_idBySubTypes.contains(className)) {
		_idBySubTypes.insert(className, {sub->internalId()});
	} else {
		_idBySubTypes[className].append(sub->internalId());
	}

	connect(sub, &DataBlock::datablockChanged, this, &DataBlock::datablockChanged); //change in the children mean a change in the parent
	connect(sub, &DataBlock::isChangedStatusChanged, this, &DataBlock::childrenChanged);

	Q_EMIT newSubItem(sub->_internalId);

}
void DataBlock::clearSubItem(qint64 id, QString className) {

	if (!_itemCache.contains(id)) {
		return;
	}

	QString cs;

	if (!className.isEmpty()) {
		if (!_idBySubTypes.contains(className)) {
			return;
		}

		if (!_idBySubTypes[className].contains(id)) {
			return;
		}

		cs = className;
	} else {
		cs = _itemCache[id]->metaObject()->className();
	}

	DataBlock* db = _itemCache[id];

	Q_EMIT subItemAboutToBeRemoved(id);
	db->clear();
	disconnect(db, &DataBlock::datablockChanged, this, &DataBlock::datablockChanged); //change in the children mean a change in the parent
	disconnect(db, &DataBlock::isChangedStatusChanged, this, &DataBlock::childrenChanged);
	_itemCache.remove(id);
	_idBySubTypes[cs].removeAll(id);

}

void DataBlock::stopTrackingChanges(bool lock) {
	_blockChanges = lock;
}
void DataBlock::isChanged() {

	emit datablockChanged();

	if (!hasChanges() and !_blockChanges) {
		_hasChanges = true;
		emit isChangedStatusChanged(true);
		if (isChildItem()) {
			qobject_cast<DataBlock*>(parent())->isChanged();
		}
	}
}

void DataBlock::childrenChanged(bool changed) {
	if (changed) {
		isChanged();
	}
}

ItemDataModel * DataBlock::getDataModel() {
	return _dataModel;
}
ItemDataModel const* DataBlock::getDataModel() const {
	return _dataModel;
}

void DataBlock::clearOptimized() {
	return;
}
void DataBlock::deepClearOptimized() {

	clearOptimized();

	for (qint64 id : listAllSubDataBlocks()) {
		DataBlock* block = getById(id);

		if (block != nullptr) {
			block->deepClearOptimized();
		}
	}
}

bool DataBlock::hasOptimizedParameters() const {
	return false;
}
bool DataBlock::hierarchyHasOptimizedParameters() const {

	if (hasOptimizedParameters()) {
		return true;
	}

	for (qint64 id : listAllSubDataBlocks()) {
		DataBlock* block = getById(id);

		if (block != nullptr) {
			if (block->hierarchyHasOptimizedParameters()) {
				return true;
			}
		}
	}

	return false;

}

DataBlock::OptimizationStep DataBlock::optimizationStep() const {
	return _opt_step;
}
void DataBlock::setOptimizationStep(OptimizationStep step) {

	if (step != _opt_step) {
		_opt_step = step;
		isChanged();
	}

}

bool DataBlock::isFixed() const {
	return _isFixed;
}
void DataBlock::setFixed(bool fixed) {
	if (_isFixed != fixed) {
		_isFixed = fixed;
		emit isFixedChanged(fixed);
	}
}

bool DataBlock::isEnabled() const {
	return _isEnabled;
}
void DataBlock::setEnabled(bool enabled) {
	if (_isEnabled != enabled) {
		_isEnabled = enabled;
		emit isEnabledChanged(enabled);
	}
}

QJsonObject DataBlock::getJsonRepresentation() const {
	return encodeJson();
}
void DataBlock::setParametersFromJsonRepresentation(QJsonObject const& rep) {
	configureFromJson(rep);
}

void DataBlock::clear() {

	if (!isInProject()) {
		return;
	}

	for (DataBlock* child : qAsConst(_itemCache)) {
		child->clear(); //recursively clear childrens
	}

	Project* p = getProject();

	QVector<QVector<qint64>> referers = _referers;

	for (QVector<qint64> referer : referers) {
		DataBlock* b = p->getByUrl(referer);

		if (b != nullptr) {
			b->referedCleared(internalUrl());
		}
	}

	for (QVector<qint64> refered : _referered) {
		DataBlock* b = p->getByUrl(refered);

		if (b != nullptr) {
			b->removeReferer(internalUrl());
		}
	}

}

void DataBlock::referedCleared(QVector<qint64> const& referedId) {
	removeRefered(referedId);
}

void DataBlock::addReferer(const QVector<qint64> &refererId) {
	if (!_referers.contains(refererId)) {
		_referers.append(refererId);
	}
}
void DataBlock::removeReferer(QVector<qint64> const& refererId) {

	if (_referers.contains(refererId)) {
		_referers.removeAll(refererId);
	}
}

void DataBlock::addRefered(const QVector<qint64> &referedId) {

	if (isInProject()) {

		if (_referered.contains(referedId)) {
			return;
		}

		Project* p = getProject();
		DataBlock* db = p->getByUrl(referedId);
		if (db != nullptr) {
			db->addReferer(internalUrl());
			_referered.append(referedId);
		}
	}
}
void DataBlock::removeRefered(const QVector<qint64> &referedId) {
	if (isInProject()) {

		if (!_referered.contains(referedId)) {
			return;
		}

		Project* p = getProject();
		DataBlock* db = p->getByUrl(referedId);
		if (db != nullptr) {
			db->removeReferer(internalUrl());
			_referered.removeAll(referedId);
		}
	}
}

qint64 DataBlock::nextAvailableId() const {

	qint64 id = 0;

	QVector<qint64> ids = listAllSubDataBlocks();

	std::sort(ids.begin(), ids.end());

	for (qint64 i : ids) {
		if (id < i) {
			return id;
		}
		id = i+1;
	}

	return id;

}

QJsonArray urlList2Json(QVector<QVector<qint64>> const& urls) {

	QJsonArray arr;

	for (QVector<qint64> const& url : urls) {
		QString surl = "";

		for (qint64 id : url) {
			surl += QString(surl.isEmpty() ? "%1" : "/%1").arg(id);
		}

		arr.push_back(surl);
	}

	return arr;

}

QVector<QVector<qint64>> json2UrlList(QJsonArray const& arr) {

	QVector<QVector<qint64>> urls;

	for (QJsonValue const& v : arr) {
		QString u = v.toString();

		QStringList l = u.split("/", Qt::SkipEmptyParts);

		QVector<qint64> url;
		url.reserve(l.count());

		for(QString const& id : l) {
			url.push_back(QVariant::fromValue(id).toInt());
		}

		urls.push_back(url);
	}

	return urls;
}

QJsonObject DataBlock::toJson() const {

	QJsonObject obj = encodeJson();

	obj.insert("id", internalId());
	obj.insert("name", objectName());

	obj.insert("fixed", (isFixed()) ? "fixed" : "unfixed");
	obj.insert("enabled", (isEnabled()) ? "enabled" : "disabled");

	QJsonArray referers = urlList2Json(_referers);
	QJsonArray referered = urlList2Json(_referered);

	obj.insert("referers", referers);
	obj.insert("referered", referered);

	return obj;

}
void DataBlock::setFromJson(QJsonObject const& obj) {

	bool trackingChanges = _blockChanges;

	stopTrackingChanges(true);

	if (obj.contains("id")) {
		_internalId = obj.value("id").toInt(-1);
	}

	if (obj.contains("name")) {
		setObjectName(obj.value("name").toString());
	}

	if (obj.contains("fixed")) {
		QString f = obj.value("fixed").toString();

		if (f.trimmed().toLower() == "fixed") {
			setFixed(true);
		} else {
			setFixed(false);
		}
	}

	if (obj.contains("enabled")) {
		QString f = obj.value("enabled").toString();

		if (f.trimmed().toLower() == "enabled") {
			setEnabled(true);
		} else {
			setEnabled(false);
		}
	}

	if (obj.contains("referers")) {
		QJsonArray rfrs = obj.value("referers").toArray();
		_referers = json2UrlList(rfrs);
	}

	if (obj.contains("referered")) {
		QJsonArray rfrd = obj.value("referered").toArray();
		_referered = json2UrlList(rfrd);
	}

	configureFromJson(obj);

	clearIsChanged();

	stopTrackingChanges(trackingChanges);

}

void DataBlock::buildDataModel() {

	_dataModel = new ItemDataModel(this);

	ItemDataModel::Category* c = _dataModel->addCategory(tr("Basic properties"));

	c->addCatProperty<QString, DataBlock, true, ItemDataModel::ItemPropertyDescription::PassByRefSignal>(tr("Name"),
																										 &DataBlock::objectName,
																										 &DataBlock::setObjectName,
																										 &DataBlock::dataBlockNameChanged);

}

qint64 DataBlock::internalId() const
{
	return _internalId;
}

DataBlockReference::DataBlockReference() :
	_referedDatablock(nullptr)
{

}

void DataBlockReference::setReferedDatablock(DataBlock* block) {

	if (_referedDatablock != nullptr) {
		_referedDatablock->_nonDatablockReferences.remove(this);
	}

	_referedDatablock = block;

	if (_referedDatablock != nullptr) {
		_referedDatablock->_nonDatablockReferences.insert(this);
	}
}

} // namespace StereoVisionApp
