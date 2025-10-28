#ifndef STEREOVISIONAPP_DATABLOCK_H
#define STEREOVISIONAPP_DATABLOCK_H

#include <QObject>
#include <QSet>
#include <QAbstractItemModel>

#include <StereoVision/geometry/rotations.h>

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

	DataBlock* getByName(QString const& name, QString const& typeHint = "") const;

	template<class T>
	T* getDataBlock(qint64 internalId) const {
		return qobject_cast<T*>(getById(internalId));
	}

	template<class T>
	T* getDataBlockByName(QString name) const {

		auto types = _idsByTypes.keys();
		for (QString const& type : qAsConst(types)) {
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
    QString relativeFilePathToAbsolute(QString const& relative);
    /*!
     * \brief relativePath return the relative path of a file w.r.t the project folder
     * \param path the path of the file
     * \param onlyIfNested if true, the path will be relative to the project only if the file is nested within the project directory.
     * \return a path to the file, relative to the project directory unless onlyIfNested is true and the file is not nested in the project directory
     */
    QString relativePath(QString const& path, bool onlyIfNested = true);

	bool hasSolution() const;

	const QVector<QString> &installedTypes() const;

    inline QString defaultProjectCRS() const {
        return _defaultProjectCRS;
    }

    inline void setDefaultProjectCRS(QString defaultProjectCRS) {
        _defaultProjectCRS = defaultProjectCRS;
    }

    /*!
     * \brief hasLocalCoordinateFrame indicate that the project has a local coordinate frame for optimisation
     * \return true if the local coordinate system exist.
     *
     * The local coordinate frame is usefull mostly when working with goegraphic coordinates.
     * The optimization will be done in a shifted ECEF frame, rather than ECEF, for numerical stability.
     */
    inline bool hasLocalCoordinateFrame() const {
        return _hasLocalCoordinateFrame;
    }

    inline StereoVision::Geometry::AffineTransform<double> ecef2local() const {
        return _ecef2local;
    }

    inline void setLocalCoordinateFrame(StereoVision::Geometry::AffineTransform<double> const& reference2local) {
        _hasLocalCoordinateFrame = true;
        _ecef2local = reference2local;

        Q_EMIT projectDataChanged();
        Q_EMIT localCooordinateFrameChanged();
    }

Q_SIGNALS:
	/*!
	 * \brief projectChanged is a signal emitted when the general structure of the project change
	 */
	void projectChanged();

	/*!
	 * \brief projectDataChanged is a signal emitted when anything in the project change (either the project structure, or any datablock)
	 */
	void projectDataChanged();

    /*!
     * \brief localCooordinateFrameChanged signal indicate that the local frame of the project change.
     *
     * The local frame represent the frame in which geometric reconstruction occurs, and is encoded as the transform from a reference frame.
     * In the case of georeferenced data, the reference frame will be ECEF.
     * Else, it is an implicit frame in which the landmarks, images and other datablocks are positioned.
     */
    void localCooordinateFrameChanged();

protected:
	void clearImpl();

	bool addType(DataBlockFactory* factory);
	void setDataBlockId(DataBlock* b, qint64 id) const;

	virtual bool loadImpls(qint64 internalId);
	virtual bool insertImpls(DataBlock* db);
	virtual qint64 nextAvailableId() const;

    QMap<qint64, DataBlock*> _itemCache;
    QMap<QString, QVector<qint64>> _idsByTypes;
    QMap<QString, QVector<qint64>> _idsByProxyTypes;

	QVector<QString> _installedTypes;
    QVector<QString> _proxyTypes;
	QMap<QString, DataBlockFactory*> _dataBlocksFactory;

	QString _source;

    QString _defaultProjectCRS;
    bool _hasLocalCoordinateFrame; //indicate if the project has a local crs
    StereoVision::Geometry::AffineTransform<double> _ecef2local; //represent the transformation from world (ECEF) and local coordinate system

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

    /*!
     * \brief proxyClassName give a class the item should be considered as when displaying the project hierarchy
     * \return the class the item should be categorized as.
     *
     * When sorting the project items
     */
    virtual QString proxyClassName() const;
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

    //allows to access the list of referers, usefull in certain cases.
    inline QVector<QVector<qint64>> const& listReferers() const {
        return _referers;
    }

	void stopTrackingChanges(bool);

	ItemDataModel * getDataModel();
	ItemDataModel const* getDataModel() const;

	virtual void clearOptimized();
	void deepClearOptimized();

	virtual bool hasOptimizedParameters() const;
	bool hierarchyHasOptimizedParameters() const;

	OptimizationStep optimizationStep() const;
	void setOptimizationStep(OptimizationStep step);

	bool isFixed() const;
	void setFixed(bool fixed);

	bool isEnabled() const;
	void setEnabled(bool enabled);



	/*!
	 * \brief getJsonRepresentation return a JsonObject containing some parameters of the datablock that needs to be reconfigurable from a file.
	 * \return a JsonObject containing the parameters.
	 *
	 * N.B this function is not the one used to save the datablock in a project, it is instead use for import/export of some datablocks.
	 * Default implementation return the result of the toJsonFunction.
	 */
	virtual QJsonObject getJsonRepresentation() const;

	/*!
	 * \brief setParametersFromJsonRepresenation set the parameters from a json representation
	 * \param rep the json object with the parameters.
	 *
	 * N.B this function is not the one used to load the datablock from a project, it is instead use for import/export of some datablocks.
	 * Default implementation use the configureFromJson function.
	 */
    virtual void setParametersFromJsonRepresentation(QJsonObject const& rep);

    virtual QJsonObject encodeJson() const = 0;
    virtual void configureFromJson(QJsonObject const& data) = 0;

Q_SIGNALS:

	/*!
	 * \brief the datablockChanged signal is emmited when anything in the datablock changed
	 */
	void datablockChanged();
	/*!
	 * \brief the isChangedStatusChanged signal is emmited when the changed status of the datablock change
	 */
	void isChangedStatusChanged(bool status);

	void newSubItem(qint64 id);
	void subItemAboutToBeRemoved(qint64 id);

	void dataBlockNameChanged(QString const& name);

	void isFixedChanged(bool fixed);
	void isEnabledChanged(bool enabled);

protected:
	void isChanged();
	void childrenChanged(bool changed);
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
	bool _isFixed;
	bool _isEnabled;

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
