#include "angleconstrainactionmanager.h"

#include "datablocks/angleconstrain.h"

namespace StereoVisionApp {

AngleConstrainActionManager::AngleConstrainActionManager(QObject* parent) :
	DatablockActionManager(parent)
{

}

QString AngleConstrainActionManager::ActionManagerClassName() const {
	return AngleConstrainActionManager::staticMetaObject.className();
}
QString AngleConstrainActionManager::itemClassName() const {
	return AngleConstrain::staticMetaObject.className();
}

} // namespace StereoVisionApp
