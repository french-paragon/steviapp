#ifndef STEREOVISIONAPP_DISTANCECONSTRAINACTIONMANAGER_H
#define STEREOVISIONAPP_DISTANCECONSTRAINACTIONMANAGER_H

#include "./actionmanager.h"

namespace StereoVisionApp {

class DistanceConstrainActionManager : public DatablockActionManager
{
	Q_OBJECT
public:
	DistanceConstrainActionManager(QObject* parent = nullptr);

	QString ActionManagerClassName() const override;
	QString itemClassName() const override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_DISTANCECONSTRAINACTIONMANAGER_H
