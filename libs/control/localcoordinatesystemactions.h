#ifndef STEREOVISIONAPP_LOCALCOORDINATESYSTEMACTIONS_H
#define STEREOVISIONAPP_LOCALCOORDINATESYSTEMACTIONS_H

#include <QList>

namespace StereoVisionApp {

class Project;

int alignLocalCoordinateSystemToPoints(QList<qint64> lcsIds, Project* p);

} //namespace StereoVisionApp

#endif // LOCALCOORDINATESYSTEMACTIONS_H
