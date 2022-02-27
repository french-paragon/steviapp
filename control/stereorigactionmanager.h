#ifndef STEREOVISIONAPP_STEREORIGACTIONMANAGER_H
#define STEREOVISIONAPP_STEREORIGACTIONMANAGER_H

#include "./actionmanager.h"

namespace StereoVisionApp {

class StereoRigActionManager : public DatablockActionManager
{
	Q_OBJECT
public:
	StereoRigActionManager(QObject *parent = nullptr);

	QString ActionManagerClassName() const override;
	QString itemClassName() const override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_STEREORIGACTIONMANAGER_H
