#include "project.h"

#include <QAction>
#include <QUrl>
#include <QFile>

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>

#include <algorithm>

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

ProjectFactory& ProjectFactory::defaultProjectFactory() {
	return *_defaultProjectFactory;
}


Project::Project(QObject* parent) : QAbstractItemModel(parent)
{

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

	beginResetModel();

	clearImpl();

	QJsonObject proj = doc.object();

	for (QString itemClass : _dataBlocksFactory.keys()) {

		QJsonArray items = proj.value(itemClass).toArray();

		for (QJsonValue v : items) {
			DataBlock* block = _dataBlocksFactory[itemClass]->factorizeDataBlock(this);
			block->setFromJson(v.toObject());

			if (block->internalId() >= 0) {
				_itemCache.insert(block->internalId(), block);
				_idsByTypes[itemClass].append(block->internalId());
			}
		}
	}

	endResetModel();

	return true;
}
bool Project::save(QString const& outFile) {

	QJsonObject proj;

	for (QString itemClass : _idsByTypes.keys()) {

		QJsonArray items;
		QVector<qint64> ids = _idsByTypes.value(itemClass);

		for (qint64 id : ids) {
			DataBlock* block = getById(id);
			QJsonObject bObj = block->toJson();

			items.push_back(bObj);
		}

		proj.insert(itemClass, items);
	}

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

	if (insertImpls(db)) {
		beginInsertRows(createIndex(_installedTypes.indexOf(cn), 0), countTypeInstances(cn), countTypeInstances(cn));
		_idsByTypes[cn].append(db->_internalId);
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

	int ItemRow = _idsByTypes.value(blockClass).indexOf(internalId);

	if (ItemRow < 0) {
		return false;
	}

	QModelIndex parentIndex = createIndex(_installedTypes.indexOf(blockClass), 0);

	beginRemoveRows(parentIndex, ItemRow, ItemRow);

	db->clear();
	_itemCache.remove(internalId);
	_idsByTypes[blockClass].remove(ItemRow);
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
int Project::countTypeInstances(QString type) const {
	return _idsByTypes.value(type).size();
}


QModelIndex Project::indexOfClass(QString const& className) {
	int row = _installedTypes.indexOf(className);

	if (row < 0) {
		return QModelIndex();
	}

	return createIndex(row, 0);
}

QModelIndex Project::indexOfClassInstance(QString const& className, qint64 id) {
	if (!_installedTypes.contains(className)) {
		return QModelIndex();
	}

	int row = _idsByTypes.value(className).indexOf(id);

	if (row < 0) {
		return QModelIndex();
	}

	DataBlockFactory* factory = _dataBlocksFactory.value(className);
	return createIndex(row, 0, factory);
}

QModelIndex Project::index(int row, int column, const QModelIndex &parent) const {

	if (parent == QModelIndex()) { //root items
		if (row < 0 or row >= _installedTypes.count()) {
			return QModelIndex();
		}
		return createIndex(row, column);
	}

	//child items
	if (parent.row() < 0 or parent.row() >= _installedTypes.count()) {
		return QModelIndex();
	}
	if (row < 0 or row >= _idsByTypes[_installedTypes.at(parent.row())].count()) {
		return QModelIndex();
	}
	DataBlockFactory* factory = _dataBlocksFactory.value(_installedTypes.at(parent.row()));
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
			id = _installedTypes.indexOf(i.key());
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
		QString typeId = _installedTypes.at(parent.row());
		return countTypeInstances(typeId);
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

		if (index.row() < 0 or index.row() >= _installedTypes.size()) {
			return QVariant();
		}

		switch (role) {
		case Qt::DisplayRole:
			return _dataBlocksFactory.value(_installedTypes.at(index.row()))->TypeDescrName();
		case ClassRole:
			return _dataBlocksFactory.value(_installedTypes.at(index.row()))->itemClassName();
		default:
			return QVariant();
		}

	} else if (index.parent().parent() == QModelIndex()) {
		QString type = _installedTypes.at(index.parent().row());

		if (index.row() < 0 or index.row() >= countTypeInstances(type)) {
			return QVariant();
		}

		switch (role) {
		case Qt::DisplayRole:
		case Qt::EditRole:
			return getById(_idsByTypes.value(type).at(index.row()))->objectName();
		case ClassRole:
			return getById(_idsByTypes.value(type).at(index.row()))->metaObject()->className();
		case IdRole:
			return _idsByTypes.value(type).at(index.row());
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
		QString type = _installedTypes.at(index.parent().row());

		if (index.row() < 0 or index.row() >= countTypeInstances(type)) {
			return false;
		}

		getById(_idsByTypes.value(type).at(index.row()))->setObjectName(value.toString());
		emit dataChanged(index, index, {Qt::DisplayRole});
	}

	return false;
}
Qt::ItemFlags Project::flags(const QModelIndex &index) const {
	if(index != QModelIndex() and index.parent() != QModelIndex() and index.parent().parent() == QModelIndex()) {
		return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
	}

	return QAbstractItemModel::flags(index);
}

void Project::clear() {
	beginResetModel();
	clearImpl();
	endResetModel();
}
void Project::clearImpl() {

	for (qint64 id : _itemCache.keys()) {
		DataBlock* db = getById(id);
		QString blockClass = db->metaObject()->className();

		int ItemRow = _idsByTypes.value(blockClass).indexOf(id);

		if (ItemRow < 0) {
			continue;
		}

		db->clear();
		_itemCache.remove(id);
		_idsByTypes[blockClass].remove(ItemRow);
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


DataBlockFactory::DataBlockFactory(QObject* parent) : QObject(parent) {

}
DataBlock* DataBlockFactory::factorizeDataBlock(Project *) const {
	return nullptr;
}
DataBlock* DataBlockFactory::factorizeDataBlock(DataBlock *) const {
	return nullptr;
}

QList<QAction*> DataBlockFactory::factorizeClassContextActions(QObject* parent, Project *p) const {
	QAction* add = new QAction(tr("New %1").arg(TypeDescrName()), parent);
	QString cName = itemClassName();
	connect(add, &QAction::triggered, [p, cName] () { p->createDataBlock(cName.toStdString().c_str()); });
	return {add};
}
QList<QAction*> DataBlockFactory::factorizeItemContextActions(QObject* , DataBlock *) const {
	return {};
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

DataBlock::DataBlock(Project *parent) :
	QObject(parent),
	_internalId(-1),
	_hasChanges(false),
	_blockChanges(false)
{

}

DataBlock::DataBlock(DataBlock *parent) :
	QObject(parent),
	_internalId(-1),
	_hasChanges(false),
	_blockChanges(false)
{

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

bool DataBlock::isInProject() {
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

	_itemCache.remove(id);
	_idBySubTypes[cs].removeAll(id);

}

void DataBlock::stopTrackingChanges(bool lock) {
	_blockChanges = lock;
}
void DataBlock::isChanged() {
	if (!hasChanges() and !_blockChanges) {
		_hasChanges = true;
		emit isChangedStatusChanged(true);
		if (isChildItem()) {
			qobject_cast<DataBlock*>(parent())->isChanged();
		}
	}
}

void DataBlock::clear() {

	if (!isInProject()) {
		return;
	}

	Project* p = getProject();

	for (QVector<qint64> referer : _referers) {
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

		QStringList l = u.split("/", QString::SkipEmptyParts);

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

	QJsonArray referers = urlList2Json(_referers);
	QJsonArray referered = urlList2Json(_referered);

	obj.insert("referers", referers);
	obj.insert("referered", referered);

	return obj;

}
void DataBlock::setFromJson(QJsonObject const& obj) {

	bool signalBlocked = signalsBlocked();

	blockSignals(true);

	if (obj.contains("id")) {
		_internalId = obj.value("id").toInt(-1);
	}

	if (obj.contains("name")) {
		setObjectName(obj.value("name").toString());
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

	blockSignals(signalBlocked);

}

qint64 DataBlock::internalId() const
{
	return _internalId;
}

} // namespace StereoVisionApp
