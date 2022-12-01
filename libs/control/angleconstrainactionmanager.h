#ifndef STEREOVISIONAPP_ANGLECONSTRAINACTIONMANAGER_H
#define STEREOVISIONAPP_ANGLECONSTRAINACTIONMANAGER_H

#include "./actionmanager.h"

namespace StereoVisionApp {

class AngleConstrainActionManager : public DatablockActionManager
{
	Q_OBJECT
public:
	AngleConstrainActionManager(QObject* parent = nullptr);

	QString ActionManagerClassName() const override;
	QString itemClassName() const override;

	QList<QAction*> factorizeItemContextActions(QObject* parent, DataBlock* p) const override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_ANGLECONSTRAINACTIONMANAGER_H
