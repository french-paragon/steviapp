#include "utils_functions.h"

#include "geometry/lensdistortion.h"
#include "geometry/stereorigrectifier.h"

#include "datablocks/stereorig.h"
#include "datablocks/image.h"
#include "datablocks/camera.h"

#include "sparsesolver/helperfunctions.h"

namespace StereoVisionApp {

std::unique_ptr<StereoVision::Geometry::ImageRectifier<float>> configureRectifierForSingleCamera(Camera* cam, bool useOptimizedParametersSet) {

	if (cam == nullptr) {
		return nullptr;
	}

	Eigen::Vector2f f;

	if (useOptimizedParametersSet) {

		if (!cam->optimizedFLen().isSet()) {
			return nullptr;
		}

		f << cam->optimizedFLen().value(), cam->optimizedFLen().value();
	} else {

		if (!cam->fLen().isSet()) {
			return nullptr;
		}

		f << cam->fLen().value(), cam->fLen().value();
	}

	Eigen::Vector2f pp;

	if (useOptimizedParametersSet) {

		if (!cam->optimizedOpticalCenterX().isSet() or !cam->optimizedOpticalCenterY().isSet()) {
			return nullptr;
		}

		pp << cam->optimizedOpticalCenterX().value(), cam->optimizedOpticalCenterY().value();
	} else {

		if (!cam->opticalCenterX().isSet() or !cam->opticalCenterY().isSet()) {
			return nullptr;
		}

		pp << cam->opticalCenterX().value(), cam->opticalCenterY().value();
	}

	Eigen::Vector2i size(cam->imHeight(), cam->imWidth());

	std::optional<Eigen::Vector3f> k123 = std::nullopt;
	std::optional<Eigen::Vector2f> t12 = std::nullopt;
	std::optional<Eigen::Vector2f> B12 = std::nullopt;

	if (useOptimizedParametersSet) {

		if (cam->useRadialDistortionModel() and
				cam->optimizedK1().isSet() and
				cam->optimizedK2().isSet() and
				cam->optimizedK3().isSet()) {
			Eigen::Vector3f k;
			k << cam->optimizedK1().value(),cam->optimizedK2().value(), cam->optimizedK3().value();
			k123 = k;
		}

		if (cam->useTangentialDistortionModel() and
				cam->optimizedP1().isSet() and
				cam->optimizedP2().isSet()) {
			Eigen::Vector2f t;
			t << cam->optimizedP1().value(),cam->optimizedP2().value();
			t12 = t;
		}

		if (cam->useSkewDistortionModel() and
				cam->optimizedB1().isSet() and
				cam->optimizedB2().isSet()) {
			Eigen::Vector2f B;
			B << cam->optimizedB1().value(),cam->optimizedB2().value();
			B12 = B;
		}
	} else {

		if (cam->useRadialDistortionModel() and
				cam->k1().isSet() and
				cam->k2().isSet() and
				cam->k3().isSet()) {
			Eigen::Vector3f k;
			k << cam->k1().value(),cam->k2().value(), cam->k3().value();
			k123 = k;
		}

		if (cam->useTangentialDistortionModel() and
				cam->p1().isSet() and
				cam->p2().isSet()) {
			Eigen::Vector2f t;
			t << cam->p1().value(),cam->p2().value();
			t12 = t;
		}

		if (cam->useSkewDistortionModel() and
				cam->B1().isSet() and
				cam->B2().isSet()) {
			Eigen::Vector2f B;
			B << cam->B1().value(),cam->B2().value();
			B12 = B;
		}

	}

	return std::make_unique<StereoVision::Geometry::ImageRectifier<float>>
			(f,
			 pp,
			 size,
			 k123,
			 t12,
			 B12);

}

std::unique_ptr<StereoVision::Geometry::StereoRigRectifier> configureRectifierForStereoPair(ImagePair* pair, bool useOptimizedParametersSet) {

	if (pair == nullptr) {
		return nullptr;
	}

	Project* p = pair->getProject();

	if (p == nullptr) {
		return nullptr;
	}

	Image* image1 = p->getDataBlock<Image>(pair->idImgCam1());
	Image* image2 = p->getDataBlock<Image>(pair->idImgCam2());

	auto img2toWorld = getImageToWorldTransform(image2);
	auto worldToImg1 = getWorldToImageTransform(image1);

	if (!img2toWorld.has_value() or !worldToImg1.has_value()) {
		return nullptr;
	}

	StereoVision::Geometry::ShapePreservingTransform cam2ToCam1 = worldToImg1.value()*img2toWorld.value();

	Camera* cam1 = p->getDataBlock<Camera>(image1->assignedCamera());
	Camera* cam2 = p->getDataBlock<Camera>(image2->assignedCamera());

	if (cam1 == nullptr or cam2 == nullptr) {
		return nullptr;
	}

	float fCam1 = cam1->fLen().value();
	float fCam2 = cam2->fLen().value();

	Eigen::Vector2f ppCam1(cam1->opticalCenterX().value(), cam1->opticalCenterY().value());
	Eigen::Vector2f ppCam2(cam2->opticalCenterX().value(), cam2->opticalCenterY().value());

	auto stmp = cam1->imSize();
	Eigen::Vector2i imSizeCam1(stmp.width(), stmp.height());

	stmp = cam2->imSize();
	Eigen::Vector2i imSizeCam2(stmp.width(), stmp.height());

	std::optional<Eigen::Vector3f> kCam1 = std::nullopt;
	std::optional<Eigen::Vector3f> kCam2 = std::nullopt;

	std::optional<Eigen::Vector2f> tCam1 = std::nullopt;
	std::optional<Eigen::Vector2f> tCam2 = std::nullopt;

	std::optional<Eigen::Vector2f> BCam1 = std::nullopt;
	std::optional<Eigen::Vector2f> BCam2 = std::nullopt;

	if (useOptimizedParametersSet) {

		if (cam1->optimizedFLen().isSet()) {
			fCam1 = cam1->optimizedFLen().value();
		}

		if (cam2->optimizedFLen().isSet()) {
			fCam2 = cam2->optimizedFLen().value();
		}

		if (cam1->optimizedOpticalCenterX().isSet() and
				cam1->optimizedOpticalCenterY().isSet()) {

			ppCam1.x() = cam1->optimizedOpticalCenterX().value();
			ppCam1.y() = cam1->optimizedOpticalCenterY().value();
		}

		if (cam2->optimizedOpticalCenterX().isSet() and
				cam2->optimizedOpticalCenterY().isSet()) {

			ppCam2.x() = cam2->optimizedOpticalCenterX().value();
			ppCam2.y() = cam2->optimizedOpticalCenterY().value();
		}

		if (cam1->useRadialDistortionModel()) {
			Eigen::Vector3f k(cam1->optimizedK1().value(), cam1->optimizedK2().value(), cam1->optimizedK3().value());
			kCam1 = k;
			k = Eigen::Vector3f(cam2->optimizedK1().value(), cam2->optimizedK2().value(), cam2->optimizedK3().value());
			kCam2 = k;
		}

		if (cam1->useTangentialDistortionModel()) {
			Eigen::Vector2f t(cam1->optimizedP1().value(), cam1->optimizedP2().value());
			tCam1 = t;
			t = Eigen::Vector2f(cam2->optimizedP1().value(), cam2->optimizedP2().value());
			tCam2 = t;
		}

		if (cam1->useSkewDistortionModel()) {
			Eigen::Vector2f b(cam1->optimizedB1().value(), cam1->optimizedB2().value());
			BCam1 = b;
			b = Eigen::Vector2f(cam2->optimizedB1().value(), cam2->optimizedB2().value());
			BCam2 = b;
		}

	} else {

		if (cam1->useRadialDistortionModel()) {
			Eigen::Vector3f k(cam1->k1().value(), cam1->k2().value(), cam1->k3().value());
			kCam1 = k;
			k = Eigen::Vector3f(cam2->k1().value(), cam2->k2().value(), cam2->k3().value());
			kCam2 = k;
		}

		if (cam1->useTangentialDistortionModel()) {
			Eigen::Vector2f t(cam1->p1().value(), cam1->p2().value());
			tCam1 = t;
			t = Eigen::Vector2f(cam2->p1().value(), cam2->p2().value());
			tCam2 = t;
		}

		if (cam1->useSkewDistortionModel()) {
			Eigen::Vector2f b(cam1->B1().value(), cam1->B2().value());
			BCam1 = b;
			b = Eigen::Vector2f(cam2->B1().value(), cam2->B2().value());
			BCam2 = b;
		}

	}

	return std::make_unique<StereoVision::Geometry::StereoRigRectifier>
			(cam2ToCam1,
			 fCam1,
			 ppCam1,
			 imSizeCam1,
			 kCam1,
			 tCam1,
			 BCam1,
			 fCam2,
			 ppCam2,
			 imSizeCam2,
			 kCam2,
			 tCam2,
			 BCam2);

}

}
