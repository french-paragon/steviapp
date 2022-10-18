#ifndef STEREOVISIONAPP_DATABLOCK_H
#define STEREOVISIONAPP_DATABLOCK_H

#include <QObject>
#include <QSet>
#include <QAbstractItemModel>

class QAction;

namespace StereoVisionApp {

class DataBlock;
class DataBlockFactory;
class ItemDataModel;
class Project;

class ProjectFactory : public QObject
{
	Q_OBJECT
public:
	explicit ProjectFactory (QObject* parent = nullptr);

	Project* createProject(QObject* parent = nullptr) const;
	bool addType(DataBlockFactory* factory);
	bool hasType(QString const& blockType) const;
	QString typeDescr(QString const& blockType) const;
	QVector<QString> installedTypes() const;

	static ProjectFactory& defaultProjectFactory();

protected:
	static ProjectFactory* _defaultProjectFactory;

	QVector<QString> _installedTypes;
	QVector<DataBlockFactory*> _factories;
};

class Project : public QAbstractItemModel
{
	Q_OBJECT
public:

	static const QString PROJECT_FILE_EXT;

	enum SpecialRoles {
		ClassRole = Qt::UserRole,
		IdRole = Qt::UserRole+1
	};

	explicit Project(QObject* parent);

	virtual bool load(QString const& inFile);
	virtual bool save();
	virtual bool save(QString const& outFile);

	qint64 createDataBlock(const char* classname);
	DataBlock* getById(qint64 internalId) const;
	DataBlock* getByUrl(QVector<qint64> const& internalUrl) const;
	DataBlockFactory* getFactoryForClass(QString cName) const;
	virtual bool clearById(qint64 internalId);

	template<class T>
	T* getDataBlock(qint64 internalId) const {
		return qobject_cast<T*>(getById(internalId));
	}

	template<class T>
	T* getDataBlockByName(QString name) const {

		for (QString type : _idsByTypes.keys()) {
			for (qint64 id : _idsByTypes.value(type)) {
				T* cand = getDataBlock<T>(id);

				if (cand != nullptr) {
					if (cand->objectName() == name) {
						return cand;
					}
				}
			}
		}

		return nullptr;
	}

	QVector<qint64> getIds() const;
	QVector<qint64> getIdsByClass(QString const& className) const;
	int countTypeInstances(QString type) const;

	QModelIndex indexOfClass(QString const& className);
	QModelIndex indexOfClassInstance(QString const& className, qint64 id);

	QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const;
	QModelIndex parent(const QModelIndex &index) const;
	int rowCount(const QModelIndex &parent = QModelIndex()) const;
	int columnCount(const QModelIndex &parent = QModelIndex()) const;
	QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;

	bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole);
	Qt::ItemFlags flags(const QModelIndex &index) const;

	void clear();
	void clearOptimized();
	void clearOptimizedState();

	QString source() const;
	void setSource(QString const& source);

	bool hasSolution() const;

Q_SIGNALS:

	void projectChanged();

protected:
	void clearImpl();

	bool addType(DataBlockFactory* factory);
	void setDataBlockId(DataBlock* b, qint64 id) const;

	virtual bool loadImpls(qint64 internalId);
	virtual bool insertImpls(DataBlock* db);
	virtual qint64 nextAvailableId() const;

	QMap<qint64, DataBlock*> _itemCache;
	QMap<QString, QVector<qint64>> _idsByTypes;

	QVector<QString> _installedTypes;
	QMap<QString, DataBlockFactory*> _dataBlocksFactory;

	QString _source;

	friend class ProjectFactory;

};

class DataBlockFactory : public QObject
{
	Q_OBJECT
public:
	explicit DataBlockFactory(QObject* parent = nullptr);

	enum FactorizableFlag {
		NoDataBlock = 0x0,
		RootDataBlock = 0x1,
		ChildDataBlock = 0x2
	};

	Q_DECLARE_FLAGS(FactorizableFlags, FactorizableFlag)
	Q_FLAG(FactorizableFlags)

	virtual QString TypeDescrName() const = 0;
	virtual FactorizableFlags factorizable() const = 0;
	virtual DataBlock* factorizeDataBlock(Project *parent = nullptr) const;
	virtual DataBlock* factorizeDataBlock(DataBlock *parent) const;

	virtual QString itemClassName() const;
};

class DataBlockReference;

class DataBlock : public QObject
{
	Q_OBJECT
public:

	enum OptimizationStep {
		Unset = 0,
		Initialized = 1,
		Optimised = 2
	};

	Q_ENUM(OptimizationStep)

	explicit DataBlock(Project *parent = nullptr);
	explicit DataBlock(DataBlock *parent = nullptr);
	virtual ~DataBlock();

	qint64 internalId() const;

	Project* getProject() const;
	bool isInProject() const;
	bool isRootItem();
	bool isChildItem();

	bool hasChanges() const;

	void clearIsChanged();

	QVector<qint64> internalUrl() const;
	QStringList subTypes() const;
	DataBlock* getById(qint64 internalId) const;

	QVector<qint64> listAllSubDataBlocks() const;
	QVector<qint64> listTypedSubDataBlocks(QString const& type) const;

	void stopTrackingChanges(bool);

	ItemDataModel * getDataModel();
	ItemDataModel const* getDataModel() const;

	virtual void clearOptimized();
	void deepClearOptimized();

	virtual bool hasOptimizedParameters() const;
	bool hierarchyHasOptimizedParameters() const;

	OptimizationStep optimizationStep() const;
	void setOptimizationStep(OptimizationStep step);

Q_SIGNALS:

	void isChangedStatusChanged(bool status);

	void newSubItem(qint64 id);
	void subItemAboutToBeRemoved(qint64 id);

	void dataBlockNameChanged(QString const& name);

protected:
	void isChanged();
	void clear();
	virtual void referedCleared(QVector<qint64> const& referedId);

	void insertSubItem(DataBlock*);
	void clearSubItem(qint64 id, QString className="");

	void addReferer(QVector<qint64> const& refererId);
	void removeReferer(QVector<qint64> const& refererId);
	void addRefered(QVector<qint64> const& referedId);
	void removeRefered(QVector<qint64> const& referedId);

	virtual qint64 nextAvailableId() const;

	QJsonObject toJson() const;
	void setFromJson(QJsonObject const& obj);

	virtual QJsonObject encodeJson() const = 0;
	virtual void configureFromJson(QJsonObject const& data) = 0;

	void buildDataModel();

	qint64 _internalId;

	QVector<QVector<qint64>> _referers;
	QVector<QVector<qint64>> _referered;

	QSet<DataBlockReference*> _nonDatablockReferences;

	QMap<qint64, DataBlock*> _itemCache;
	QMap<QString, QVector<qint64>> _idBySubTypes;

	bool _hasChanges;
	bool _blockChanges;

	OptimizationStep _opt_step;

	ItemDataModel* _dataModel;

	friend class Project;
	friend class DataBlockFactory;
	friend class DataBlockReference;
};

/*!
 * \brief The EditableItemReference class allow for a non-datablock class to keep a reference to a given datablock.
 *
 * The datablock when it gets deleted, will notify the EditableItemReference by replacing the pointer by nullptr.
 */
class DataBlockReference
{
public:

	void setReferedDatablock(DataBlock* block);
	inline DataBlock* getReferredDatablock() const {
		return _referedDatablock;
	}

protected:
	DataBlockReference();

	DataBlock* _referedDatablock;

	friend class DataBlock;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_DATABLOCK_H
