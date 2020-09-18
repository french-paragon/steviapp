#include "itemdatamodel.h"

#include "./project.h"

#include <QFont>

namespace StereoVisionApp {


ItemDataModel::ItemPropertyDescription::ItemPropertyDescription(Category *cat, QString descrName) :
	_cat(cat),
	_collectionManager(nullptr),
	_block(nullptr),
	_name(descrName)
{

}
ItemDataModel::ItemPropertyDescription::ItemPropertyDescription(SubItemCollectionManager* subItemsManager,
																DataBlock* block,
																QString descrName) :
	_cat(nullptr),
	_collectionManager(subItemsManager),
	_block(block),
	_name(descrName)
{

}
ItemDataModel::ItemPropertyDescription::~ItemPropertyDescription() {
	disconnect(vChangedConnection);
}

QString ItemDataModel::ItemPropertyDescription::name() const
{
	return _name;
}

void ItemDataModel::ItemPropertyDescription::setName(const QString &name)
{
	_name = name;
}

QString ItemDataModel::ItemPropertyDescription::targetClass() const
{
	return _targetClass;
}

void ItemDataModel::ItemPropertyDescription::setTargetClass(const QString &targetClass)
{
	_targetClass = targetClass;
}

QStringList ItemDataModel::ItemPropertyDescription::options() const
{
	return _options;
}

void ItemDataModel::ItemPropertyDescription::setOptions(const QStringList &options)
{
	_options = options;
}

bool ItemDataModel::ItemPropertyDescription::hasSecondValue() const {
	return false;
}
QVariant ItemDataModel::ItemPropertyDescription::sencondValue() const {
	return QVariant();
}
bool ItemDataModel::ItemPropertyDescription::setSecondValue(QVariant const&) {
	return false;
}

DataBlock* ItemDataModel::ItemPropertyDescription::block() const {
	if (_cat != nullptr) {
		return _cat->_model->_dataBlock;
	} else {
		return _block;
	}
}

void ItemDataModel::ItemPropertyDescription::valueChanged() {
	if (_cat != nullptr) {
		_cat->_model->itemValueChanged(this);
	}
}

ItemDataModel::SubItemCollectionManager* ItemDataModel::ItemPropertyDescription::collectionManager() const
{
	return _collectionManager;
}

ItemDataModel::Category *ItemDataModel::ItemPropertyDescription::category() const
{
	return _cat;
}

ItemDataModel::ItemPropertyDescriptionFactory::~ItemPropertyDescriptionFactory() {

}

ItemDataModel::Node::Node(ItemDataModel* model, Node* parent) :
	_parent(parent),
	_model(model)
{

}
ItemDataModel::Node::~Node() {

}

QString ItemDataModel::Node::title() const {
	return "";
}

int ItemDataModel::Node::level() const {

	Node* p = _parent;
	int c = 0;

	while (p != nullptr) {
		p = p->_parent;
		c++;
	}

	return c;
}
ItemDataModel::Node::NodeType ItemDataModel::Node::type() const {
	return Root;
}
ItemDataModel::Node* ItemDataModel::Node::parent() const {
	return _parent;
}

ItemDataModel::TopLevelNode::TopLevelNode(ItemDataModel* model, QString descrName) :
	Node(model),
	_name(descrName)
{

}

QString ItemDataModel::TopLevelNode::title() const {
	return _name;
}
ItemDataModel::Node* ItemDataModel::TopLevelNode::parent() const {
	if (_model == nullptr) {
		return nullptr;
	}
	return _model->_internalRootNode;
}
ItemDataModel::Category::Category(ItemDataModel* model, QString descrName) :
	TopLevelNode(model, descrName),
	_itemProperties(0)
{

}
ItemDataModel::Category::~Category() {

}

ItemDataModel::Node::NodeType ItemDataModel::Category::type() const {
	return ItemDataModel::Node::Category;
}


ItemDataModel::SubItemCollectionManager::SubItemCollectionManager(ItemDataModel* model,
																  QString const& descrName,
																  QString const& targetClass,
																  TitleFunc const& titleGetter) :
	TopLevelNode(model, descrName),
	_targetClass(targetClass),
	_titleGetter(titleGetter)
{

	DataBlock* b = model->_dataBlock;

	for (qint64 id : b->listTypedSubDataBlocks(_targetClass)) {
		insertSubitem(id);
	}

	nItemConnection = QObject::connect(b, &DataBlock::newSubItem, [this] (qint64 id) {
		newItem(id);
	});

	rItemConnection = QObject::connect(b, &DataBlock::subItemAboutToBeRemoved, [this] (qint64 id) {
		itemRemoved(id);
	});

}
ItemDataModel::SubItemCollectionManager::~SubItemCollectionManager() {

	QObject::disconnect(nItemConnection);
	QObject::disconnect(rItemConnection);

	for(SubItemNode* n : _subItems) {

		for (ItemPropertyDescription* d : _propertiesDescritions[n->subItemId]) {
			delete d;
		}

		delete n;
	}

	for (ItemPropertyDescriptionFactory* f : _propsDescrFactories) {
		delete f;
	}
}

int ItemDataModel::SubItemCollectionManager::countSubItems() const {
	return _subItems.size();
}
QVector<qint64> ItemDataModel::SubItemCollectionManager::subItemsIds() const {
	QVector<qint64> ids;
	ids.reserve(_subItems.size());

	for (SubItemNode* n : _subItems) {
		ids.push_back(n->subItemId);
	}

	return ids;
}
QString ItemDataModel::SubItemCollectionManager::getSubItemTitleById(qint64 id) {

	if (_model == nullptr) {
		return "";
	}

	DataBlock* b = _model->_dataBlock->getById(id);

	if (b == nullptr) {
		return "";
	}

	return _titleGetter(b);

}


ItemDataModel::SubItemCollectionManager::SubItemNode::SubItemNode(SubItemCollectionManager* parent,
																  qint64 id) :
	Node(parent->_model, parent),
	subItemId(id)
{

}

void ItemDataModel::SubItemCollectionManager::newItem(qint64 id) {

	_model->subItemAboutToBeAdded(this);

	DataBlock* b = _model->_dataBlock;

	DataBlock* s = b->getById(id);

	if (s->metaObject()->className() == _targetClass) {
		insertSubitem(id);
	}

	_model->subItemAdded(this);
}

void ItemDataModel::SubItemCollectionManager::itemRemoved(qint64 id) {

	int n = -1;
	for(int i = 0; i < _subItems.size(); i++) {
		if (_subItems[i]->subItemId == id) {
			n = i;
			break;
		}
	}

	if (n < 0) {
		return;
	}

	_model->subItemAboutToBeRemoved(this, n);

	if (!_propertiesDescritions.contains(id)) {
		return;
	}

	for (ItemPropertyDescription* p : _propertiesDescritions[id]) {
		delete p;
	}

	_propertiesDescritions.remove(id);

	SubItemNode* node = _subItems[n];
	_subItems.removeAt(n);
	delete node;

	_model->subItemRemoved(this, n);

}

void ItemDataModel::SubItemCollectionManager::insertSubitem(qint64 id) {

	SubItemNode* n = new SubItemNode(this, id);
	_subItems.push_back(n);

}

ItemDataModel::Node::NodeType ItemDataModel::SubItemCollectionManager::type() const {
	return SubItems;
}

ItemDataModel::Node* ItemDataModel::SubItemCollectionManager::getNodeForBlock(qint64 id) const {
	for (SubItemNode* n : _subItems) {
		if (n->subItemId == id) {
			return static_cast<Node*>(n);
		}
	}

	return nullptr;
}
ItemDataModel::Node* ItemDataModel::SubItemCollectionManager::getNodeAtRow(int row) const {
	return static_cast<Node*>(_subItems.at(row));
}

int ItemDataModel::SubItemCollectionManager::rowForProperty(qint64 id, ItemPropertyDescription*descr) const {
	if (!_propertiesDescritions.contains(id)) {
		return -1;
	}
	return _propertiesDescritions[id].indexOf(descr);
}

int ItemDataModel::SubItemCollectionManager::rowForItem(Node* itemNode) const {
	if (itemNode->type() != SubItem) {
		return -1;
	}

	SubItemNode* sin = static_cast<SubItemNode*>(itemNode);

	for (int i = 0; i < _subItems.size(); i++) {
		if (_subItems[i] == sin) {
			return i;
		}
	}

	return -1;
}

QString ItemDataModel::SubItemCollectionManager::SubItemNode::title() const {
	if (_parent == nullptr) {
		return "";
	}

	if (_parent->type() != SubItems) {
		return "";
	}

	SubItemCollectionManager* col = static_cast<SubItemCollectionManager*>(_parent);

	if (col == nullptr) {
		return "";
	}

	return col->getSubItemTitleById(subItemId);

}

ItemDataModel::Node::NodeType ItemDataModel::SubItemCollectionManager::SubItemNode::type() const {
	return SubItem;
}


ItemDataModel::ItemDataModel(DataBlock *parent) :
	QAbstractItemModel(parent),
	_dataBlock(parent)
{
	_internalRootNode = new Node(this);
}
ItemDataModel::~ItemDataModel() {
	for (Node* n : _topLevelNodes) {
		delete n;
	}
	delete _internalRootNode;
}

ItemDataModel::Category* ItemDataModel::addCategory(QString const& name) {
	Category* cat = new Category(this, name);
	_topLevelNodes.append(cat);
	return cat;
}
ItemDataModel::SubItemCollectionManager* ItemDataModel::addCollectionManager(const QString &name, QString const& targetClass, const SubItemCollectionManager::TitleFunc &titleGetter) {
	SubItemCollectionManager* manager = new SubItemCollectionManager(this, name, targetClass, titleGetter);
	_topLevelNodes.append(manager);
	return manager;
}

QModelIndex ItemDataModel::index(int row, int column, const QModelIndex &parent) const {

	if (parent == QModelIndex()) {
		return createIndex(row, column, reinterpret_cast<void*>(_internalRootNode));
	}

	Node* npNode = nodeFromIndex(parent);

	return createIndex(row, column, reinterpret_cast<void*>(npNode));
}
QModelIndex ItemDataModel::parent(const QModelIndex &index) const {

	if (index == QModelIndex()) {
		return QModelIndex();
	}

	Node* pNode = reinterpret_cast<Node*>(index.internalPointer());

	if (pNode == nullptr) {
		return QModelIndex();
	}

	switch (pNode->type()) {
	case Node::Root:
		return QModelIndex();
	case Node::Category:
	case Node::SubItems: {
		int row = findTopLevelItemRow(pNode);
		if (row >= 0) {
			return createIndex(row, 0, _internalRootNode);
		}
		return QModelIndex();
	}
	case Node::SubItem: {
		SubItemCollectionManager* m = static_cast<SubItemCollectionManager*>(pNode->parent());
		int row = m->rowForItem(pNode);

		if (row >= 0) {
			return createIndex(row, 0, reinterpret_cast<void*>(static_cast<Node*>(m)));
		} else {
			return QModelIndex();
		}
	}
	}

	return QModelIndex();
}

int ItemDataModel::rowCount(const QModelIndex &parent) const {

	Node* pNode = nodeFromIndex(parent);

	if (pNode == nullptr) {
		return 0;
	}

	switch (pNode->type()) {
	case Node::Root:
		return _topLevelNodes.size();
	case Node::Category: {
		Category* cat = static_cast<Category*>(pNode);
		return cat->_itemProperties.size();
	}
	case Node::SubItems: {
		SubItemCollectionManager* m = static_cast<SubItemCollectionManager*>(pNode);
		return m->_subItems.size();
	}
	case Node::SubItem: {
		SubItemCollectionManager* m = static_cast<SubItemCollectionManager*>(pNode->parent());
		return m->_propsDescrFactories.size();
	}
	}

	return 0;

}
int ItemDataModel::columnCount(const QModelIndex &) const {

	return 3;

}
QVariant ItemDataModel::data(const QModelIndex &index, int role) const {

	if (index == QModelIndex()) {
		return QVariant();
	}

	Node* pNode = reinterpret_cast<Node*>(index.internalPointer());
	Node* node = nodeFromIndex(index);

	if (pNode == nullptr) {
		return QVariant();
	}

	switch (role) {
	case Qt::DisplayRole:
		if (node == nullptr) { // leaf item, this is a property
			ItemPropertyDescription* descr = propDescrFromIndex(index);

			if (descr == nullptr) {
				return QVariant();
			} else {
				if (index.column() == 0) {
					return descr->name();
				} else if (index.column() == 1) {
					return descr->data();
				} else {
					if (descr->hasSecondValue()) {
						return descr->sencondValue();
					} else {
						return QVariant();
					}
				}
			}
		} else {
			if (index.column() == 0) {
				return node->title();
			}
		}
		break;
	case Qt::FontRole :
		QFont font;
		if (node != nullptr) { //non leaf items
			font.setBold(true);
		}
		return font;
	}

	return QVariant();

}

bool ItemDataModel::setData(const QModelIndex &index, const QVariant &value, int role) {

	Node* node = nodeFromIndex(index);

	if (node != nullptr) { //non leaf item
		return false;
	}

	if (role == Qt::EditRole) {
		ItemPropertyDescription* descr = propDescrFromIndex(index);

		if (descr == nullptr) {
			return false;
		} else {
			if (index.column() == 1) {
				return descr->setData(value);
			} else if (index.column() == 2 and descr->hasSecondValue()) {
				return descr->setSecondValue(value);
			} else {
				return false;
			}
		}
	}

	return false;
}
Qt::ItemFlags ItemDataModel::flags(const QModelIndex &index) const {

	if (index.column() == 0) {
		return QAbstractItemModel::flags(index);
	}

	Node* node = nodeFromIndex(index);

	if (node != nullptr) { //non leaf item
		return QAbstractItemModel::flags(index);
	}

	ItemPropertyDescription* descr = propDescrFromIndex(index);

	if (descr == nullptr) {
		return QAbstractItemModel::flags(index);
	}

	if (index.column() == 1) {
		return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
	} else if (index.column() == 2 and descr->hasSecondValue()) {
		return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
	}

	return QAbstractItemModel::flags(index);
}

QVariant ItemDataModel::headerData(int section, Qt::Orientation orientation, int role) const {

	if (orientation == Qt::Horizontal) {
		if (role == Qt::DisplayRole) {
			switch (section) {
			case 0:
				return tr("Name");
			case 1:
				return tr("Value");
			case 2:
				return tr("stddev");
			}
		}
	}

	return QVariant();

}

int ItemDataModel::findTopLevelItemRow(Node* topLevelItem) const {

	for (int i = 0; i < _topLevelNodes.size(); i++) {
		if (_topLevelNodes[i] == topLevelItem) {
			return i;
		}
	}

	return -1;

}

QModelIndex ItemDataModel::getIndexForProperty(ItemPropertyDescription* descr) {
	Category* c = descr->category();
	if (c != nullptr) {
		int row = c->_itemProperties.indexOf(descr);
		if (row >= 0) {
			return createIndex(row, 0, reinterpret_cast<void*>(static_cast<Node*>(c)));
		} else {
			return QModelIndex();
		}
	}

	SubItemCollectionManager* m = descr->collectionManager();
	if (m != nullptr) {
		DataBlock* b = descr->block();
		Node* n = m->getNodeForBlock(b->internalId());

		if (n != nullptr) {
			int row = m->rowForProperty(b->internalId(), descr);

			if (row >= 0) {
				return createIndex(row, 0, reinterpret_cast<void*>(n));
			} else {
				return QModelIndex();
			}
		} else {
			return QModelIndex();
		}
	}

	return QModelIndex();
}
ItemDataModel::Node* ItemDataModel::nodeFromIndex(QModelIndex const& index) const {

	if (index == QModelIndex()) {
		return _internalRootNode;
	}

	Node* pNode = reinterpret_cast<Node*>(index.internalPointer());

	Node* node;

	if (pNode == nullptr) {
		return _internalRootNode;
	} else {
		switch (pNode->type()) {
		case Node::Root:
			node = _topLevelNodes.at(index.row());
			break;
		case Node::SubItems:
			node = static_cast<SubItemCollectionManager*>(pNode)->getNodeAtRow(index.row());
			break;
		case Node::Category:
		case Node::SubItem: //the internal pointer points toward the parent node, so those categories are already leafs
			return nullptr;
		}
	}

	return node;

}

ItemDataModel::ItemPropertyDescription* ItemDataModel::propDescrFromIndex(QModelIndex const& index) const {

	Node* pNode = reinterpret_cast<Node*>(index.internalPointer());

	if (pNode == nullptr) {
		return nullptr;
	}

	ItemPropertyDescription* descr = nullptr;
	switch (pNode->type()) {
	case Node::Category:
		descr = static_cast<Category*>(pNode)->_itemProperties[index.row()];
		break;
	case Node::SubItem: {
		SubItemCollectionManager::SubItemNode* s = static_cast<SubItemCollectionManager::SubItemNode*>(pNode);
		SubItemCollectionManager* m = static_cast<SubItemCollectionManager*>(pNode->parent());
		descr = m->_propertiesDescritions[s->subItemId].at(index.row());
	}
	default:
		break;
	}

	return descr;
}

void ItemDataModel::subItemAboutToBeAdded(SubItemCollectionManager* subItems) {

	int r = findTopLevelItemRow(static_cast<Node*>(subItems));

	if (r >= 0) {
		QModelIndex p = index(r, 0);
		beginInsertRows(p, rowCount(p), rowCount(p));
	}

}
void ItemDataModel::subItemAdded(SubItemCollectionManager* subItems) {

	int r = findTopLevelItemRow(static_cast<Node*>(subItems));

	if (r >= 0) {
		endInsertRows();
	}
}

void ItemDataModel::subItemAboutToBeRemoved(SubItemCollectionManager* subItems, int row) {

	int r = findTopLevelItemRow(static_cast<Node*>(subItems));

	if (r >= 0) {
		QModelIndex p = index(r, 0);
		beginRemoveRows(p, row, row);
	}

}
void ItemDataModel::subItemRemoved(SubItemCollectionManager* subItems, int) {
	int r = findTopLevelItemRow(static_cast<Node*>(subItems));

	if (r >= 0) {
		endRemoveRows();
	}
}

void ItemDataModel::itemValueChanged(ItemPropertyDescription* descr) {

	QModelIndex c0 = getIndexForProperty(descr);

	if (c0.isValid()) {
		QModelIndex c2 = createIndex(c0.row(), (descr->hasSecondValue()) ? 2 : 1, c0.internalPointer());

		emit dataChanged(c0, c2);
	}

}


} // namespace StereoVisionApp
