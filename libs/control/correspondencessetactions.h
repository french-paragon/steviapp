#ifndef STEREOVISIONAPP_CORRESPONDENCESSETACTIONS_H
#define STEREOVISIONAPP_CORRESPONDENCESSETACTIONS_H

#include <QtGlobal>
#include <QString>

namespace StereoVisionApp {

class Project;

bool importCorrespondencesFromTxt(Project* project, qint64 correspSetId, QString const& importFilePath = "");
bool exportCorrespondencesToTxt(Project* project, qint64 correspSetId, QString const& exportFilePath = "");

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CORRESPONDENCESSETACTIONS_H
