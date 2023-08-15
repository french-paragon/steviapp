#ifndef STEREOVISIONAPP_CORNERDETECTIONACTIONS_H
#define STEREOVISIONAPP_CORNERDETECTIONACTIONS_H

#include <QtGlobal>

namespace StereoVisionApp {

class Project;

void detectCornerInImage(Project* proj, qint64 imgId);

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CORNERDETECTIONACTIONS_H
