#ifndef STEREOVISIONAPP_STEREORIGACTIONS_H
#define STEREOVISIONAPP_STEREORIGACTIONS_H

#include <QtCore/qglobal.h>

namespace StereoVisionApp {

class Project;

void alignImagesInRig(Project* p, qint64 rig_id, qint64 ref_image_id, qint64 unaligned_image_id);

} //namespace StereoVisionApp
#endif // STEREOVISIONAPP_STEREORIGACTIONS_H
