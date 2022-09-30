#ifndef STEREOVISIONAPP_CAMERACALIBRATIONACTIONMANAGER_H
#define STEREOVISIONAPP_CAMERACALIBRATIONACTIONMANAGER_H

#include "./actionmanager.h"

namespace StereoVisionApp {

class CameraCalibrationActionManager : public DatablockActionManager
{
public:
	explicit CameraCalibrationActionManager(QObject *parent = nullptr);

	QString ActionManagerClassName() const override;
	QString itemClassName() const override;

	QList<QAction*> factorizeClassContextActions(QObject* parent, Project* p) const override;
	QList<QAction*> factorizeItemContextActions(QObject* parent, DataBlock* p) const override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CAMERACALIBRATIONACTIONMANAGER_H
