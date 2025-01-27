#include "pushbroomcameraactionmanager.h"

#include "datablocks/cameras/pushbroompinholecamera.h"

#include "gui/lenseditor.h"

#include "mainwindow.h"

#include <QWidget>
#include <QAction>

namespace StereoVisionApp {

PushBroomCameraActionManager::PushBroomCameraActionManager(QObject* parent) :
    DatablockActionManager(parent)
{

}

QString PushBroomCameraActionManager::ActionManagerClassName() const {
    return PushBroomCameraActionManager::staticMetaObject.className();
}
QString PushBroomCameraActionManager::itemClassName() const {
    return PushBroomPinholeCamera::staticMetaObject.className();
}

QList<QAction*> PushBroomCameraActionManager::factorizeClassContextActions(QObject* parent, Project* p) const {
    return QList<QAction*>();
}
QList<QAction*> PushBroomCameraActionManager::factorizeItemContextActions(QObject* parent, DataBlock* item) const {

    if (item == nullptr) {
        return {};
    }

    PushBroomPinholeCamera* cam = qobject_cast<PushBroomPinholeCamera*>(item);

    QWidget* w = qobject_cast<QWidget*>(parent);

    if (w != nullptr) {
        w = w->window();
    }

    MainWindow* mw = qobject_cast<MainWindow*>(w);

    QList<QAction*> lst;

    if (mw != nullptr) {
        if (cam != nullptr) {
            QAction* edit = new QAction(tr("Edit"), parent);
            connect(edit, &QAction::triggered, [mw, cam] () {

                Editor* e = mw->openEditor(PushBroomLenseEditor::staticMetaObject.className());
                PushBroomLenseEditor* le = qobject_cast<PushBroomLenseEditor*>(e);

                le->setCamera(cam);

            });
            lst << edit;
        }
    }

    QAction* clearOptimized = new QAction(tr("Clear optimized"), parent);
    connect(clearOptimized, &QAction::triggered, [item] () {
        item->clearOptimized();
    });
    lst.append(clearOptimized);

    QAction* remove = new QAction(tr("Remove"), parent);
    connect(remove, &QAction::triggered, [item] () {
        Project* p = item->getProject();

        if (p != nullptr) {
            p->clearById(item->internalId());
        }
    });
    lst.append(remove);

    return lst;
}

} // namespace StereoVisionApp
