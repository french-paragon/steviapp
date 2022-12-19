#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include "datablocks/image.h"
#include "datablocks/landmark.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "geometry/core.h"
#include "geometry/rotations.h"
#include "geometry/pointcloudalignment.h"

#include <optional>

namespace StereoVisionApp {

inline std::optional<StereoVision::Geometry::ShapePreservingTransform<float>> getImageToWorldTransform(Image* img) {

	if (img == nullptr) {
		return std::nullopt;
	}

	if (img->optPos().isSet() and img->optRot().isSet()) {

		floatParameterGroup<3> pos = img->optPos();
		floatParameterGroup<3> rot = img->optRot();

		Eigen::Vector3f r;
		r.x() = rot.value(0);
		r.y() = rot.value(1);
		r.z() = rot.value(2);

		Eigen::Vector3f t(pos.value(0),
						  pos.value(1),
						  pos.value(2));

		StereoVision::Geometry::ShapePreservingTransform<float> imgToWorld(r, t, 1.);

		return imgToWorld;

	}

	return std::nullopt;
}

inline std::optional<StereoVision::Geometry::ShapePreservingTransform<float>> getWorldToImageTransform(Image* img) {

	std::optional<StereoVision::Geometry::ShapePreservingTransform<float>> res = getImageToWorldTransform(img);

	if (res.has_value()) {
		StereoVision::Geometry::ShapePreservingTransform<float> inv = res.value().inverse();
		return inv;
	}

	return std::nullopt;

}

inline std::optional<Eigen::Vector3f> getPointPosition(Landmark* lm) {

	if (lm == nullptr) {
		return std::nullopt;
	}

	if (lm->optPos().isSet()) {
		floatParameterGroup<3> pos = lm->optPos();

		Eigen::Vector3f t(pos.value(0),
						  pos.value(1),
						  pos.value(2));

		return t;
	}

	return std::nullopt;
}

} // namespace StereoVisionApp

#endif // HELPERFUNCTIONS_H
