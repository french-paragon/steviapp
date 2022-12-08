#ifndef STEREOVISIONAPP_STEREORIGACTIONMANAGER_H
#define STEREOVISIONAPP_STEREORIGACTIONMANAGER_H

#include "./actionmanager.h"

namespace StereoVisionApp {

class StereoRig;

class StereoRigActionManager : public DatablockActionManager
{
	Q_OBJECT
public:
	StereoRigActionManager(QObject *parent = nullptr);

	QList<QAction*> factorizeClassContextActions(QObject* parent, Project* p) const override;
	QList<QAction*> factorizeItemContextActions(QObject* parent, DataBlock* item) const override;

	QString ActionManagerClassName() const override;
	QString itemClassName() const override;

protected:

	QAction* createAlignImageActions(QObject* parent, Project* p, StereoRig* rig) const;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_STEREORIGACTIONMANAGER_H
