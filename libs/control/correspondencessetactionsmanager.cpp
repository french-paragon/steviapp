#include "correspondencessetactionsmanager.h"

#include "datablocks/correspondencesset.h"

#include "control/mainwindow.h"

#include <QAction>
#include <QFileDialog>

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
        QAction* append = new QAction(tr("Import correspondences"), parent);
        connect(append, &QAction::triggered, [mw, correspSet] () {

            QString inFile = QFileDialog::getOpenFileName(mw, tr("Import correspondences file"));

            if (inFile.isEmpty()) {
                return;
            }

            QFile in(inFile);

            if (!in.open(QFile::ReadOnly)) {
                return;
            }

            QTextStream stream(&in);

            while (!stream.atEnd()) {
                QString line = stream.readLine();

                auto opt = Correspondences::GenericPair::fromString(line);

                if (!opt.has_value()) {
                    continue;
                }

                if (!opt->isValid()) {
                    continue;
                }

                correspSet->addCorrespondence(opt.value());
            }

        });
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
