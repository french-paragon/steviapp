#ifndef STEREOVISIONAPP_PUSHBROOMCAMERAACTIONMANAGER_H
#define STEREOVISIONAPP_PUSHBROOMCAMERAACTIONMANAGER_H

#include "./actionmanager.h"

namespace StereoVisionApp {

class PushBroomCameraActionManager : public DatablockActionManager
{
    Q_OBJECT
public:
    PushBroomCameraActionManager(QObject* parent = nullptr);

    QString ActionManagerClassName() const override;
    QString itemClassName() const override;

    QList<QAction*> factorizeClassContextActions(QObject* parent, Project* p) const override;
    QList<QAction*> factorizeItemContextActions(QObject* parent, DataBlock* p) const override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_PUSHBROOMCAMERAACTIONMANAGER_H
