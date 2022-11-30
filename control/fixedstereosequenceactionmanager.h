#ifndef STEREOVISIONAPP_FIXEDSTEREOSEQUENCEACTIONMANAGER_H
#define STEREOVISIONAPP_FIXEDSTEREOSEQUENCEACTIONMANAGER_H


#include "./actionmanager.h"

namespace StereoVisionApp {

class FixedStereoPlusColorSequenceActionManager: public DatablockActionManager
{
	Q_OBJECT
public:
	FixedStereoPlusColorSequenceActionManager(QObject* parent);

	QString ActionManagerClassName() const override;
	QString itemClassName() const override;

	QList<QAction*> factorizeItemContextActions(QObject* parent, DataBlock* p) const override;
};

class FixedColorStereoSequenceActionManager: public DatablockActionManager
{
	Q_OBJECT
public:
	FixedColorStereoSequenceActionManager(QObject* parent);

	QString ActionManagerClassName() const override;
	QString itemClassName() const override;

	QList<QAction*> factorizeItemContextActions(QObject* parent, DataBlock* p) const override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_FIXEDSTEREOSEQUENCEACTIONMANAGER_H
