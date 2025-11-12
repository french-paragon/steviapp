#ifndef STEREOVISIONAPP_CORNERDETECTIONACTIONS_H
#define STEREOVISIONAPP_CORNERDETECTIONACTIONS_H

#include <QtGlobal>

namespace StereoVisionApp {

class Project;

void detectCornerInTestImage();
void detectCornerInImage(Project* proj, qint64 imgId);

void matchCornersInTestImagePair();
void matchCornersInImagePair(Project* proj, qint64 imgId1, qint64 imgId2);

void addImages2MatchCornersEditor(Project* proj, QVector<qint64> const& imgs);

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CORNERDETECTIONACTIONS_H
