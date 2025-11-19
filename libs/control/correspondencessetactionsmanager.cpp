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

            importCorrespondencesFromTxt(p, -1);

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

            exportCorrespondencesToTxt(correspSet->getProject(), correspSet->internalId());

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

void CorrespondencesSetActionsManager::registerAppHeadlessActions(StereoVisionApplication* application) const {

    constexpr char const* CorrespondencesSetNamespace = "CorrespondencesSet";

    application->registerHeadlessAction(CorrespondencesSetNamespace,"exportSet", [] (QMap<QString,QString> const& kwargs, QStringList const& argv) {

        bool ok = true;

        qint64 id = argv[0].toInt(&ok);

        if (!ok) {
            return StatusOptionalReturn<void>::error("Invalid set id provided!");
        }

        QString outPath = argv[1];

        StereoVisionApplication* app = StereoVisionApplication::GetAppInstance();

        if (app == nullptr) {
            return StatusOptionalReturn<void>::error("No active app instance!");
        }

        Project* p = app->getCurrentProject();

        if (p == nullptr) {
            return StatusOptionalReturn<void>::error("No active project!");
        }

        ok = exportCorrespondencesToTxt(p, id, outPath);

        if (!ok) {
            return StatusOptionalReturn<void>::error("Unknwon error!");
        }

        return StatusOptionalReturn<void>();
    });
}
} // namespace StereoVisionApp
