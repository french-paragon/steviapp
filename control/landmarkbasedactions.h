#ifndef LANDMARKBASEDACTIONS_H
#define LANDMARKBASEDACTIONS_H

#include <QVector>
#include <QString>

namespace StereoVisionApp {

class Project;

void exportLandmarksToCsv(Project* p, QVector<qint64> const& landmarks, QString const& file);
void attachLandmarkToLocalCoordinateSystem(Project* p, QVector<qint64> const& landmarks, qint64 localCoordinateSystem);

} //namespace StereoVisionApp

#endif // LANDMARKBASEDACTIONS_H
