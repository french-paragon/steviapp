#ifndef STEREOVISIONAPP_ITEMDATAMODEL_H
#define STEREOVISIONAPP_ITEMDATAMODEL_H

#include <QAbstractItemModel>
#include <QSet>

#include "./floatparameter.h"
#include "./project.h"

namespace StereoVisionApp {

class ItemDataModel : public QAbstractItemModel
{
	Q_OBJECT
public:

	class Category;
	class SubItemCollectionManager;

	class ItemPropertyDescription
	{
	public:

		virtual ~ItemPropertyDescription();

		enum PropertyEdtior {
			ValueEditor = 0x0,
			TargetItemByClassEditor = 0x1,
			OptionListEditor = 0x2
		};

		enum SignalType {
			PassByValueSignal,
			PassByRefSignal,
			NoValueSignal
		};

		QString name() const;
		void setName(const QString &name);

		QString targetClass() const;
		void setTargetClass(const QString &targetClass);

		QStringList options() const;
		void setOptions(const QStringList &options);

		virtual PropertyEdtior editor() const = 0;

		virtual QVariant data() const = 0;
		virtual bool setData(QVariant const& d) = 0;

		virtual bool hasSecondValue() const;
		virtual QVariant sencondValue() const;
		virtual bool setSecondValue(QVariant const& d);

		DataBlock* block() const;

		Category* category() const;

		SubItemCollectionManager *collectionManager() const;

	protected:

		ItemPropertyDescription(Category* cat, QString descrName);
		ItemPropertyDescription(SubItemCollectionManager* subItemsManager, DataBlock* block, QString descrName);

		void valueChanged();

		Category* _cat;
		SubItemCollectionManager* _collectionManager;
		DataBlock* _block;
		QString _name;

		QString _targetClass;
		QStringList _options;

		QMetaObject::Connection vChangedConnection;
	};

protected:

	template<typename T, class D, bool refS, ItemPropertyDescription::SignalType signalT>
	class TypedItemPropertyDescr : public ItemPropertyDescription
	{
	public:
		typedef void(D::*SetterType)(typename std::conditional<refS, T const&, T>::type);
		typedef typename std::conditional<signalT==NoValueSignal,
										void(D::*)(),
										typename std::conditional<signalT==PassByRefSignal,
																void(D::*)(T const&),
																void(D::*)(T)>::type>::type SignalType;
		typedef T (D::*GetterType)() const;

		TypedItemPropertyDescr(Category* cat,
							   QString descrName,
							   GetterType getter,
							   SetterType setter,
							   SignalType signal) :
			ItemPropertyDescription(cat, descrName),
			_setter(setter),
			_getter(getter)
		{
			D* d = castedBlock();
			vChangedConnection = QObject::connect(d, signal, [this] () {
				valueChanged();
			});
		}

		TypedItemPropertyDescr(SubItemCollectionManager* subItemsManager,
							   DataBlock* block,
							   QString descrName,
							   GetterType getter,
							   SetterType setter,
							   SignalType signal) :
			ItemPropertyDescription(subItemsManager, block, descrName),
			_setter(setter),
			_getter(getter)
		{
			D* d = castedBlock();
			vChangedConnection = QObject::connect(d, signal, [this] () {
				valueChanged();
			});
		}

		PropertyEdtior editor() const override {
			return ValueEditor;
		}

		QVariant data() const override {
			D* block = castedBlock();
			if (block == nullptr) {
				return QVariant();
			}
			return QVariant::fromValue((block->*_getter)());
		}
		bool setData(QVariant const& d) override {
			if (!d.canConvert(qMetaTypeId<T>())) {
				return false;
			}

			D* block = castedBlock();
			if (block == nullptr) {
				return false;
			}

			T v = qvariant_cast<T>(d);
			(block->*_setter)(v);
			return true;
		}

		D* castedBlock() const {
			DataBlock* b = block();
			return qobject_cast<D*>(b);
		}

	protected:
		SetterType _setter;
		GetterType _getter;
	};

	template<class D, bool refS, ItemPropertyDescription::SignalType signalT>
	class TypedItemPropertyDescr<floatParameter, D, refS, signalT> : public ItemPropertyDescription
	{
	public:
		typedef void(D::*SetterType)(typename std::conditional<refS, floatParameter const&, floatParameter>::type);
		typedef typename std::conditional<signalT==NoValueSignal,
										void(D::*)(),
										void(D::*)(typename std::conditional<signalT==PassByRefSignal,
													floatParameter const&,
													floatParameter>::type)>::type SignalType;
		typedef floatParameter (D::*GetterType)() const;

		TypedItemPropertyDescr(Category* cat,
							   QString descrName,
							   GetterType getter,
							   SetterType setter,
							   SignalType signal) :
			ItemPropertyDescription(cat, descrName),
			_setter(setter),
			_getter(getter)
		{
			D* d = castedBlock();
			vChangedConnection = QObject::connect(d, signal, [this] () {
				valueChanged();
			});
		}

		TypedItemPropertyDescr(SubItemCollectionManager* subItemsManager,
							   DataBlock* block,
							   QString descrName,
							   GetterType getter,
							   SetterType setter,
							   SignalType signal) :
			ItemPropertyDescription(subItemsManager, block, descrName),
			_setter(setter),
			_getter(getter)
		{
			D* d = castedBlock();
			vChangedConnection = QObject::connect(d, signal, [this] () {
				valueChanged();
			});
		}

		PropertyEdtior editor() const override {
			return ValueEditor;
		}

		QVariant data() const override {
			D* block = castedBlock();
			if (block == nullptr) {
				return QVariant();
			}
			return QVariant::fromValue((block->*_getter)().value());
		}
		bool setData(QVariant const& d) override {
			if (!d.canConvert(qMetaTypeId<qreal>())) {
				return false;
			}

			D* block = castedBlock();
			if (block == nullptr) {
				return false;
			}

			pFloatType v = static_cast<pFloatType>(qvariant_cast<qreal>(d));
			floatParameter p = (block->*_getter)();
			p.setIsSet(v);
			(block->*_setter)(p);
			return true;
		}

		D* castedBlock() const {
			DataBlock* b = block();
			return qobject_cast<D*>(b);
		}

		bool hasSecondValue() const {
			return true;
		}
		QVariant sencondValue() const {
			D* block = castedBlock();

			if (block == nullptr) {
				return QVariant();
			}

			floatParameter fp = (block->*_getter)();

			if (!fp.isUncertain()) {
				return QVariant(" ");
			}

			return QVariant::fromValue(fp.stddev());
		}
		bool setSecondValue(QVariant const& d) {
			if (!d.canConvert(qMetaTypeId<qreal>())) {
				return false;
			}

			D* block = castedBlock();
			if (block == nullptr) {
				return false;
			}

			pFloatType v = static_cast<pFloatType>(qvariant_cast<qreal>(d));
			floatParameter p = (block->*_getter)();
			p.setUncertainty(v);
			(block->*_setter)(p);
			return true;
		}

	protected:
		SetterType _setter;
		GetterType _getter;
	};


	class ItemPropertyDescriptionFactory
	{
	public:
		virtual ~ItemPropertyDescriptionFactory();
		virtual ItemPropertyDescription* factorizeDescription(SubItemCollectionManager* subItemsManager,
															  DataBlock* block) = 0;
	};

	template<typename T, class D, bool refS, ItemPropertyDescription::SignalType signalT>
	class TypedItemPropertyDescrFactory : public ItemPropertyDescriptionFactory
	{
	public:

		typedef typename TypedItemPropertyDescr<T,D,refS,signalT>::GetterType GetterType;
		typedef typename TypedItemPropertyDescr<T,D,refS,signalT>::SetterType SetterType;
		typedef typename TypedItemPropertyDescr<T,D,refS,signalT>::SignalType SignalType;

		TypedItemPropertyDescrFactory(
				QString descrName,
				GetterType getter,
				SetterType setter,
				SignalType signal) :
			_descrName(descrName),
			_getter(getter),
			_setter(setter),
			_signal(signal)
		{

		}

		ItemPropertyDescription* factorizeDescription(SubItemCollectionManager* subItemsManager,
													  DataBlock* block) override {
			return new TypedItemPropertyDescr<T,D,refS,signalT>(subItemsManager, block, _descrName, _getter, _setter, _signal);
		}

	protected:

		QString _descrName;
		GetterType _getter;
		SetterType _setter;
		SignalType _signal;
	};

	class Node {
	public:

		Node(ItemDataModel* model, Node* parent = nullptr);
		virtual ~Node();

		enum NodeType{
			Root,
			Category,
			SubItems,
			SubItem,
		};

		virtual QString title() const;
		int level() const;
		virtual NodeType type() const;
		virtual Node* parent() const;

	protected:

		Node* _parent;
		ItemDataModel* _model;
	};

public:

	class TopLevelNode : public Node
	{
	public:
		TopLevelNode(ItemDataModel* model, QString descrName);

		virtual QString title() const;
		virtual Node* parent() const;

	protected:
		QString _name;
	};

	class Category : public TopLevelNode
	{
	public:

		virtual NodeType type() const;

		template<typename T, class D, bool refS, ItemPropertyDescription::SignalType signalT>
		ItemPropertyDescription* addCatProperty(QString name,
							typename TypedItemPropertyDescr<T, D, refS, signalT>::GetterType getter,
							typename TypedItemPropertyDescr<T, D, refS, signalT>::SetterType setter,
							typename TypedItemPropertyDescr<T, D, refS, signalT>::SignalType signal) {

			ItemPropertyDescription* prop = new TypedItemPropertyDescr<T, D, refS, signalT>(this,
																							name,
																							getter,
																							setter,
																							signal);

			_itemProperties.push_back(prop);
		}

	protected:
		Category(ItemDataModel* model, QString descrName);
		virtual ~Category();

		QVector<ItemPropertyDescription*> _itemProperties;

		friend class ItemDataModel;
	};

	class SubItemCollectionManager : public TopLevelNode
	{
	protected:

		struct SubItemNode : public Node {
		public:

			SubItemNode(SubItemCollectionManager* parent, qint64 id);

			qint64 subItemId;
			virtual QString title() const;
			virtual NodeType type() const;
		};

	public:

		typedef std::function<QString(DataBlock*)> TitleFunc ;

		virtual ~SubItemCollectionManager();

		int countSubItems() const;
		QVector<qint64> subItemsIds() const;
		QString getSubItemTitleById(qint64 id);

		virtual NodeType type() const;

		template<typename T, class D, bool refS, ItemPropertyDescription::SignalType signalT>
		ItemPropertyDescription* addCatProperty(QString name,
							typename TypedItemPropertyDescr<T, D, refS, signalT>::GetterType getter,
							typename TypedItemPropertyDescr<T, D, refS, signalT>::SetterType setter,
							typename TypedItemPropertyDescr<T, D, refS, signalT>::SignalType signal) {

			ItemPropertyDescriptionFactory* f = new TypedItemPropertyDescrFactory<T,D,refS,signalT>(name,
																									getter,
																									setter,
																									signal);

			_propsDescrFactories.push_back(f);

			for (SubItemNode* n : _subItems) {
				DataBlock* b = _model->_dataBlock->getById(n->subItemId);
				_propertiesDescritions[n->subItemId].push_back(f->factorizeDescription(this, b));
			}

		}

	protected:

		SubItemCollectionManager(ItemDataModel* model,
								 QString const& descrName,
								 QString const& targetClass,
								 const TitleFunc &titleGetter = [] (DataBlock* b) { return b->objectName(); });

		void newItem(qint64 id);
		void itemRemoved(qint64 id);

		void insertSubitem(qint64 id);

		Node* getNodeForBlock(qint64 id) const;
		Node* getNodeAtRow(int row) const;
		int rowForProperty(qint64 id, ItemPropertyDescription* descr) const;
		int rowForItem(Node* itemNode) const;

		QString _targetClass;
		TitleFunc _titleGetter;

		QVector<SubItemNode*> _subItems;
		QVector<ItemPropertyDescriptionFactory*> _propsDescrFactories;
		QMap<qint64, QVector<ItemPropertyDescription*>> _propertiesDescritions;

		QMetaObject::Connection nItemConnection;
		QMetaObject::Connection rItemConnection;

		friend class ItemDataModel;
	};

public:
	explicit ItemDataModel(DataBlock *parent = nullptr);
	~ItemDataModel();

	Category* addCategory(const QString &name);
	SubItemCollectionManager* addCollectionManager(const QString &name, QString const& targetClass, typename SubItemCollectionManager::TitleFunc const& titleGetter);

	QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const override;
	QModelIndex parent(const QModelIndex &index) const override;
	int rowCount(const QModelIndex &parent = QModelIndex()) const override;
	int columnCount(const QModelIndex &parent = QModelIndex()) const override;
	QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

	bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) override;
	Qt::ItemFlags flags(const QModelIndex &index) const override;

	QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

signals:

protected:

	int findTopLevelItemRow(Node* topLevelItem) const;
	QModelIndex getIndexForProperty(ItemPropertyDescription* descr);
	Node* nodeFromIndex(QModelIndex const& index) const;
	ItemPropertyDescription* propDescrFromIndex(QModelIndex const& index) const;

	void subItemAboutToBeAdded(SubItemCollectionManager* subItems);
	void subItemAdded(SubItemCollectionManager* subItems);

	void subItemAboutToBeRemoved(SubItemCollectionManager* subItems, int row);
	void subItemRemoved(SubItemCollectionManager* subItems, int row);

	void itemValueChanged(ItemPropertyDescription* descr);

	DataBlock* _dataBlock;

	Node* _internalRootNode;
	QVector<Node*> _topLevelNodes;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_ITEMDATAMODEL_H
