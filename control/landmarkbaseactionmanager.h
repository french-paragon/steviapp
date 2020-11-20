#ifndef STEREOVISIONAPP_LANDMARKBASEACTIONMANAGER_H
#define STEREOVISIONAPP_LANDMARKBASEACTIONMANAGER_H

#include "./actionmanager.h"

namespace StereoVisionApp {

class LandmarkBaseActionManager : public DatablockActionManager
{
	Q_OBJECT
public:
	explicit LandmarkBaseActionManager(QObject *parent = nullptr);

	QString ActionManagerClassName() const override;
	QString itemClassName() const override;

signals:

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LANDMARKBASEACTIONMANAGER_H
