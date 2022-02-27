#include "stereorigactionmanager.h"

#include "datablocks/stereorig.h"

#include <QWidget>

namespace StereoVisionApp {

StereoRigActionManager::StereoRigActionManager(QObject* parent) :
	DatablockActionManager(parent)
{

}

QString StereoRigActionManager::ActionManagerClassName() const {
	return "StereoVisionApp::StereoRigActionManager";
}
QString StereoRigActionManager::itemClassName() const {
	return StereoRigFactory::StereoRigClassName();
}

} // namespace StereoVisionApp
