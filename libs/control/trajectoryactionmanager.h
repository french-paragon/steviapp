#ifndef STEREOVISIONAPP_TRAJECTORYACTIONMANAGER_H
#define STEREOVISIONAPP_TRAJECTORYACTIONMANAGER_H

#include "./actionmanager.h"

class QAction;

namespace StereoVisionApp {

class Trajectory;

class TrajectoryActionManager : public DatablockActionManager
{
    Q_OBJECT
public:
    TrajectoryActionManager(QObject *parent = nullptr);

    QList<QAction*> factorizeClassContextActions(QObject* parent, Project* p) const override;
    QList<QAction*> factorizeItemContextActions(QObject* parent, DataBlock* item) const override;
    QList<QAction*> factorizeMultiItemsContextActions(QObject* parent, Project* p, QModelIndexList const& projectIndex) const override;

    QString ActionManagerClassName() const override;
    QString itemClassName() const override;

protected:

    enum AssignMountMode {
        GPS,
        INS
    };
    QAction* createAssignToMountingAction(QObject* parent,
                                 StereoVisionApp::Project* p,
                                 QVector<Trajectory*> const& trajs,
                                 AssignMountMode mode) const;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORYACTIONMANAGER_H
