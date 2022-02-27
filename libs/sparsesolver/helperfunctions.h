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

inline std::optional<StereoVision::Geometry::ShapePreservingTransform> getImageToWorldTransform(Image* img) {

	if (img == nullptr) {
		return std::nullopt;
	}

	if (img->optPos().isSet() and img->optRot().isSet()) {

		floatParameterGroup<3> pos = img->optPos();
		floatParameterGroup<3> rot = img->optRot();

		Eigen::Matrix3f R = (Eigen::AngleAxisf(rot.value(0)/180*M_PI, Eigen::Vector3f(1,0,0))*
					Eigen::AngleAxisf(rot.value(1)/180*M_PI, Eigen::Vector3f(0,1,0))*
					Eigen::AngleAxisf(rot.value(2)/180*M_PI, Eigen::Vector3f(0,0,1))).toRotationMatrix();

		Eigen::Vector3f t(pos.value(0),
						  pos.value(1),
						  pos.value(2));

		Eigen::Vector3f r = StereoVision::Geometry::inverseRodriguezFormula(R);

		StereoVision::Geometry::ShapePreservingTransform imgToWorld(r, t, 1.);

		return imgToWorld;

	}

	return std::nullopt;
}

inline std::optional<StereoVision::Geometry::ShapePreservingTransform> getWorldToImageTransform(Image* img) {

	std::optional<StereoVision::Geometry::ShapePreservingTransform> res = getImageToWorldTransform(img);

	if (res.has_value()) {
		StereoVision::Geometry::ShapePreservingTransform inv = res.value().inverse();
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
