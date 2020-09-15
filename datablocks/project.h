#ifndef STEREOVISIONAPP_DATABLOCK_H
#define STEREOVISIONAPP_DATABLOCK_H

#include <QObject>
#include <QAbstractItemModel>

class QAction;

namespace StereoVisionApp {

class DataBlock;
class DataBlockFactory;
class Project;

class ProjectFactory : public QObject
{
	Q_OBJECT
public:
	explicit ProjectFactory (QObject* parent = nullptr);

	Project* createProject(QObject* parent = nullptr) const;
	bool addType(DataBlockFactory* factory);

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

	enum SepcialRoles {
		ClassRole = Qt::UserRole,
		IdRole = Qt::UserRole+1
	};

	explicit Project(QObject* parent);

	virtual bool load(QString const& datasource);
	virtual bool save(QString const& datasource);

	qint64 createDataBlock(const char* classname);
	DataBlock* getById(qint64 internalId) const;
	DataBlock* getByUrl(QVector<qint64> const& internalUrl) const;
	DataBlockFactory* getFactoryForClass(QString cName) const;
	virtual bool clearById(qint64 internalId);

	QVector<qint64> getIds() const;
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

Q_SIGNALS:

protected:

	bool addType(DataBlockFactory* factory);
	void setDataBlockId(DataBlock* b, qint64 id) const;

	virtual bool loadImpls(qint64 internalId);
	virtual bool insertImpls(DataBlock* db);
	virtual qint64 nextAvailableId() const;

	QMap<qint64, DataBlock*> _itemCache;
	QMap<QString, QVector<qint64>> _idsByTypes;

	QVector<QString> _installedTypes;
	QMap<QString, DataBlockFactory*> _dataBlocksFactory;

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

	virtual QList<QAction*> factorizeClassContextActions(QObject* parent, Project* p) const;
	virtual QList<QAction*> factorizeItemContextActions(QObject* parent, DataBlock* p) const;

	virtual QString itemClassName() const;
};

class DataBlock : public QObject
{
	Q_OBJECT
public:
	explicit DataBlock(Project *parent = nullptr);
	explicit DataBlock(DataBlock *parent = nullptr);

	qint64 internalId() const;

	Project* getProject() const;
	bool isInProject();
	bool isRootItem();
	bool isChildItem();

	bool hasChanges() const;

	void clearIsChanged();

	QVector<qint64> internalUrl() const;
	QStringList subTypes() const;
	DataBlock* getById(qint64 internalId);

	QVector<qint64> listAllSubDataBlocks() const;
	QVector<qint64> listTypedSubDataBlocks(QString const& type) const;

	void stopTrackingChanges(bool);

Q_SIGNALS:

	void isChangedStatusChanged(bool status);

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

	qint64 _internalId;

	QVector<QVector<qint64>> _referers;
	QVector<QVector<qint64>> _referered;

	QMap<qint64, DataBlock*> _itemCache;
	QMap<QString, QVector<qint64>> _idBySubTypes;

	bool _hasChanges;
	bool _blockChanges;

	friend class Project;
	friend class DataBlockFactory;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_DATABLOCK_H
