#ifndef STEREOVISIONAPP_STEREORIGACTIONS_H
#define STEREOVISIONAPP_STEREORIGACTIONS_H

#include <QtCore/qglobal.h>
#include <QString>

#include <optional>
#include <array>

namespace StereoVisionApp {

class Project;

void alignImagesInRig(Project* p, qint64 rig_id, qint64 ref_image_id, qint64 unaligned_image_id);

void exportRectifiedImages(Project* p, qint64 rig_id, qint64 image_pair_id);

void exportRigDetails(Project* p, qint64 rig_id,
					  QString outPath = "",
					  std::optional<float> cam1_f = std::nullopt,
					  std::optional<std::array<float,2>> cam1_pp = std::nullopt,
					  std::optional<float> cam2_f = std::nullopt,
					  std::optional<std::array<float,2>> cam2_pp = std::nullopt);

} //namespace StereoVisionApp
#endif // STEREOVISIONAPP_STEREORIGACTIONS_H
