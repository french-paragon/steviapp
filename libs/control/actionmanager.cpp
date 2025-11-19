#include "actionmanager.h"

#include <QAction>
#include <QCoreApplication>

#include "datablocks/project.h"

namespace StereoVisionApp {

DatablockActionManager::DatablockActionManager(QObject *parent) :
	QObject(parent)
{

}

QString DatablockActionManager::TypeDescrName() const {

	ProjectFactory* f = qobject_cast<ProjectFactory*>(parent());

	if (f != nullptr) {
		return f->typeDescr(itemClassName());
	}

	return ProjectFactory::defaultProjectFactory().typeDescr(itemClassName());
}

QList<QAction*> DatablockActionManager::factorizeClassContextActions(QObject* parent, Project* p) const {
	QAction* add = new QAction(tr("New %1").arg(TypeDescrName()), parent);
	QString cName = itemClassName();
	connect(add, &QAction::triggered, [p, cName] () { p->createDataBlock(cName.toStdString().c_str()); });
	return {add};
}
QList<QAction*> DatablockActionManager::factorizeItemContextActions(QObject* parent, DataBlock* block) const {

    if (block == nullptr) {
        return {};
    }

    QList<QAction*> lst;

    QAction* remove = new QAction(tr("Remove"), parent);
    connect(remove, &QAction::triggered, [block] () {
        Project* p = block->getProject();

        if (p != nullptr) {
            p->clearById(block->internalId());
        }
    });
    lst.append(remove);

    return lst;
}
QList<QAction*> DatablockActionManager::factorizeMultiItemsContextActions(QObject* parent, Project* p, QModelIndexList const& projectIndex) const {

	Q_UNUSED(parent);
	Q_UNUSED(p);
	Q_UNUSED(projectIndex);
	return {};
}

void DatablockActionManager::registerAppHeadlessActions(StereoVisionApplication* application) const {
    Q_UNUSED(application);
    return;
}


ActionManagersLibrary* ActionManagersLibrary::_defaultActionManagersLibrary = nullptr;

ActionManagersLibrary::ActionManagersLibrary(QObject* parent) :
	QObject(parent)
{

}

ActionManagersLibrary::~ActionManagersLibrary() {
	if (this == _defaultActionManagersLibrary) {
		_defaultActionManagersLibrary = nullptr;
	}
}

bool ActionManagersLibrary::registerDatablockActionManager(DatablockActionManager* block) {

	QString c = block->itemClassName();

	if (_dataBlockActionManagers.contains(c)) {

		for (DatablockActionManager* m : _dataBlockActionManagers.values(c)) {
			if (m->ActionManagerClassName() == block->ActionManagerClassName()) {
				return false;
			}
		}

	}

	_dataBlockActionManagers.insert(c, block);

    StereoVisionApplication* app = StereoVisionApplication::GetAppInstance();

    if (app != nullptr) {
        block->registerAppHeadlessActions(app);
    }

	return true;

}

QList<QAction*> ActionManagersLibrary::factorizeDatablockClassContextActions(QString itemClassname, QObject* parent, Project* p) const {

	QList<QAction*> actions;

	for (DatablockActionManager* m : _dataBlockActionManagers.values(itemClassname)) {
		actions << m->factorizeClassContextActions(parent, p);
	}

	return actions;

}
QList<QAction*> ActionManagersLibrary::factorizeDatablockItemContextActions(QString itemClassname, QObject* parent, DataBlock* p) const {

	QList<QAction*> actions;

	for (DatablockActionManager* m : _dataBlockActionManagers.values(itemClassname)) {
		actions << m->factorizeItemContextActions(parent, p);
	}

	return actions;

}
QList<QAction*> ActionManagersLibrary::factorizeDatablockMultiItemsContextActions(QString itemClassname, QObject* parent, Project* p, QModelIndexList const& projectIndex) const {

	QList<QAction*> actions;

	for (DatablockActionManager* m : _dataBlockActionManagers.values(itemClassname)) {
		actions << m->factorizeMultiItemsContextActions(parent, p, projectIndex);
	}

	return actions;

}

ActionManagersLibrary& ActionManagersLibrary::defaultActionManagersLibrary() {

	if (_defaultActionManagersLibrary == nullptr) {
		_defaultActionManagersLibrary = new ActionManagersLibrary(QCoreApplication::instance());
	}

	return *_defaultActionManagersLibrary;
}

} // namespace StereoVisionApp
