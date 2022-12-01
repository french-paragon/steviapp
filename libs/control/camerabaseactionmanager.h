#ifndef STEREOVISIONAPP_CAMERABASEACTIONMANAGER_H
#define STEREOVISIONAPP_CAMERABASEACTIONMANAGER_H

#include "./actionmanager.h"

namespace StereoVisionApp {

class CameraBaseActionManager : public DatablockActionManager
{
	Q_OBJECT
public:
	explicit CameraBaseActionManager(QObject *parent = nullptr);

	QString ActionManagerClassName() const override;
	QString itemClassName() const override;

	QList<QAction*> factorizeClassContextActions(QObject* parent, Project* p) const override;
	QList<QAction*> factorizeItemContextActions(QObject* parent, DataBlock* p) const override;

signals:

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CAMERABASEACTIONMANAGER_H
