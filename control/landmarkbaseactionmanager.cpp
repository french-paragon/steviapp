#include "landmarkbaseactionmanager.h"

#include "datablocks/landmark.h"

namespace StereoVisionApp {

LandmarkBaseActionManager::LandmarkBaseActionManager(QObject *parent) :
	DatablockActionManager(parent)
{

}

QString LandmarkBaseActionManager::ActionManagerClassName() const {
	return "StereoVisionApp::LandmarkBaseActionManager";
}
QString LandmarkBaseActionManager::itemClassName() const {
	return LandmarkFactory::landmarkClassName();
}

} // namespace StereoVisionApp
