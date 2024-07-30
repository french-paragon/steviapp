#ifndef PROJECTACTIONS_H
#define PROJECTACTIONS_H

namespace StereoVisionApp {

class Project;
class MainWindow;

bool setDefaultProjectCRS(Project* p);
bool estimateLocalCoordinateSystem(Project* p);

} //namespace StereoVisionApp

#endif // PROJECTACTIONS_H
