#include "correspondencessetactionsmanager.h"

#include "datablocks/correspondencesset.h"

#include "control/mainwindow.h"

#include <QAction>
#include <QFileDialog>

#include "correspondencessetactions.h"

namespace StereoVisionApp {

CorrespondencesSetActionsManager::CorrespondencesSetActionsManager(QObject *parent) :
    DatablockActionManager(parent)
{

}

QString CorrespondencesSetActionsManager::ActionManagerClassName() const {
    return CorrespondencesSetActionsManager::staticMetaObject.className();
}
QString CorrespondencesSetActionsManager::itemClassName() const {
    return CorrespondencesSet::staticMetaObject.className();
}

QList<QAction*> CorrespondencesSetActionsManager::factorizeClassContextActions(QObject* parent, Project* p) const {

    Q_UNUSED(parent);

    QString classname = itemClassName();

    MainWindow* mw = MainWindow::getActiveMainWindow();

    QList<QAction*> lst;

    if (mw != nullptr) {
        QAction* append = new QAction(tr("Import correspondences"), parent);
        connect(append, &QAction::triggered, [p] () {

            importCorrespondancesFromTxt(p, -1);

        });
        lst.append(append);
    }

    QAction* add = new QAction(tr("Add a new Correspondences set"), parent);
    connect(add, &QAction::triggered, [classname, p] () {
        qint64 block_id = p->createDataBlock(classname.toStdString().c_str());
        if (block_id > 0) {
            DataBlock* b = p->getById(block_id);
            b->setObjectName(tr("new correspondences set"));
        }
    });

    lst.append(add);

    return lst;
}
QList<QAction*> CorrespondencesSetActionsManager::factorizeItemContextActions(QObject* parent, DataBlock* p) const {

    CorrespondencesSet* correspSet = qobject_cast<CorrespondencesSet*>(p);

    if (correspSet == nullptr) {
        return {};
    }

    QWidget* w = qobject_cast<QWidget*>(parent);

    if (w != nullptr) {
        w = w->window();
    }

    MainWindow* mw = qobject_cast<MainWindow*>(w);

    QList<QAction*> lst;

    if (mw != nullptr) {

        QAction* exportCorresps = new QAction(tr("export correspondences"), parent);
        connect(exportCorresps, &QAction::triggered, [correspSet] () {

            exportCorrespondancesToTxt(correspSet->getProject(), correspSet->internalId());

        });
        lst.append(exportCorresps);
    }

    QAction* remove = new QAction(tr("Remove"), parent);
    connect(remove, &QAction::triggered, [correspSet] () {
        Project* p = correspSet->getProject();

        if (p != nullptr) {
            p->clearById(correspSet->internalId());
        }
    });
    lst.append(remove);

    return lst;

}

} // namespace StereoVisionApp
