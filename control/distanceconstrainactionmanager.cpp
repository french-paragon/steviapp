#include "distanceconstrainactionmanager.h"

#include "datablocks/distanceconstrain.h"

namespace StereoVisionApp {

DistanceConstrainActionManager::DistanceConstrainActionManager(QObject *parent) :
	DatablockActionManager(parent)
{

}

QString DistanceConstrainActionManager::ActionManagerClassName() const {
	return DistanceConstrainActionManager::staticMetaObject.className();
}
QString DistanceConstrainActionManager::itemClassName() const {
	return DistanceConstrain::staticMetaObject.className();
}

} // namespace StereoVisionApp
