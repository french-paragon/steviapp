#ifndef STEREOVISIONAPP_CORRESPONDENCESSETACTIONSMANAGER_H
#define STEREOVISIONAPP_CORRESPONDENCESSETACTIONSMANAGER_H

#include "./actionmanager.h"

namespace StereoVisionApp {

class CorrespondencesSetActionsManager : public DatablockActionManager
{
    Q_OBJECT
public:
    CorrespondencesSetActionsManager(QObject* parent = nullptr);

    QString ActionManagerClassName() const override;
    QString itemClassName() const override;

    QList<QAction*> factorizeClassContextActions(QObject* parent, Project* p) const override;
    QList<QAction*> factorizeItemContextActions(QObject* parent, DataBlock* p) const override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CORRESPONDENCESSETACTIONSMANAGER_H
