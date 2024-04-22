#ifndef STEREOVISIONAPP_TRAJECTORYACTIONMANAGER_H
#define STEREOVISIONAPP_TRAJECTORYACTIONMANAGER_H

#include "./actionmanager.h"

namespace StereoVisionApp {

class TrajectoryActionManager : public DatablockActionManager
{
    Q_OBJECT
public:
    TrajectoryActionManager(QObject *parent = nullptr);

    QList<QAction*> factorizeClassContextActions(QObject* parent, Project* p) const override;
    QList<QAction*> factorizeItemContextActions(QObject* parent, DataBlock* item) const override;

    QString ActionManagerClassName() const override;
    QString itemClassName() const override;

protected:

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYACTIONMANAGER_H
