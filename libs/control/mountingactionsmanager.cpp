#include "mountingactionsmanager.h"

#include "datablocks/mounting.h"

#include <QAction>

namespace StereoVisionApp {

MountingActionsManager::MountingActionsManager(QObject *parent) :
    DatablockActionManager(parent)
{

}

QString MountingActionsManager::ActionManagerClassName() const {
    return MountingActionsManager::staticMetaObject.className();
}

QString MountingActionsManager::itemClassName() const {
    return Mounting::staticMetaObject.className();
}

} // namespace StereoVisionApp
