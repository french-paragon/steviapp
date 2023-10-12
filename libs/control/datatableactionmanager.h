#ifndef STEREOVISIONAPP_DATATABLEACTIONMANAGER_H
#define STEREOVISIONAPP_DATATABLEACTIONMANAGER_H

#include "./actionmanager.h"


namespace StereoVisionApp {

class DataTableActionManager : public DatablockActionManager
{
public:
    DataTableActionManager(QObject* parent = nullptr);

    QString ActionManagerClassName() const override;
    QString itemClassName() const override;

    QList<QAction*> factorizeClassContextActions(QObject* parent, Project* p) const override;
    QList<QAction*> factorizeItemContextActions(QObject* parent, DataBlock* p) const override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_DATATABLEACTIONMANAGER_H
