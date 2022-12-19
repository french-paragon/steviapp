#ifndef STEREOVISIONAPP_FIXEDSTEREOSEQUENCEACTIONS_H
#define STEREOVISIONAPP_FIXEDSTEREOSEQUENCEACTIONS_H

#include <QVector>

namespace StereoVisionApp {

class FixedColorStereoSequence;
class FixedStereoPlusColorSequence;

void exportColoredStereoImagesRectifiedImages(FixedColorStereoSequence* sequence, QVector<int> rows);
void exportStereoImagesPlusColorRectifiedImages(FixedStereoPlusColorSequence* sequence, QVector<int> rows);

void exportColoredStereoImagesPointCloud(FixedColorStereoSequence* sequence, QVector<int> rows);
void exportStereoImagesPlusColorPointCloud(FixedStereoPlusColorSequence* sequence, QVector<int> rows);


} //namespace StereoVisionApp

#endif // FIXEDSTEREOSEQUENCEACTIONS_H
