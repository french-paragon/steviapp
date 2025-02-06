#ifndef STEREOVISIONAPP_MOUNTINGACTIONSMANAGER_H
#define STEREOVISIONAPP_MOUNTINGACTIONSMANAGER_H

#include "./actionmanager.h"

namespace StereoVisionApp {

class MountingActionsManager : public DatablockActionManager
{
public:
    MountingActionsManager(QObject* parent);

    QString ActionManagerClassName() const override;
    QString itemClassName() const override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_MOUNTINGACTIONSMANAGER_H
