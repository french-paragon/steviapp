#ifndef STEREOVISIONAPP_ACTIONMANAGER_H
#define STEREOVISIONAPP_ACTIONMANAGER_H

#include <QObject>
#include <QModelIndexList>

class QAction;

namespace StereoVisionApp {

class Project;
class DataBlock;
class ProjectFactory;

class DatablockActionManager : public QObject
{
	Q_OBJECT
public:
	explicit DatablockActionManager(QObject *parent = nullptr);

	virtual QString ActionManagerClassName() const = 0;
	virtual QString itemClassName() const = 0;

	virtual QList<QAction*> factorizeClassContextActions(QObject* parent, Project* p) const;
	virtual QList<QAction*> factorizeItemContextActions(QObject* parent, DataBlock* p) const;
	virtual QList<QAction*> factorizeMultiItemsContextActions(QObject* parent, Project* p, QModelIndexList const& projectIndex) const;

signals:

protected:

	QString TypeDescrName() const;

private:

};

class ActionManagersLibrary: public QObject
{
	Q_OBJECT
public:
	explicit ActionManagersLibrary(QObject* parent);

	virtual ~ActionManagersLibrary();

	bool registerDatablockActionManager(DatablockActionManager* block);

	QList<QAction*> factorizeDatablockClassContextActions(QString itemClassname, QObject* parent, Project* p) const;
	QList<QAction*> factorizeDatablockItemContextActions(QString itemClassname, QObject* parent, DataBlock* p) const;
	QList<QAction*> factorizeDatablockMultiItemsContextActions(QString itemClassname, QObject* parent, Project* p, QModelIndexList const& projectIndex) const;

	static ActionManagersLibrary& defaultActionManagersLibrary();

protected:

	QMultiMap<QString, DatablockActionManager*> _dataBlockActionManagers;

private:

	static ActionManagersLibrary* _defaultActionManagersLibrary;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_ACTIONMANAGER_H
