#include "stereorigactions.h"

#include "datablocks/project.h"
#include "datablocks/stereorig.h"
#include "datablocks/image.h"

namespace StereoVisionApp {

void alignImagesInRig(Project* p, qint64 rig_id, qint64 ref_image_id, qint64 unaligned_image_id) {

	StereoRig* rig = p->getDataBlock<StereoRig>(rig_id);

	if (rig == nullptr) {
		return;
	}

	ImagePair* pair = rig->getPairForImage(ref_image_id);

	if (pair == nullptr) {
		return;
	}

	if (pair->idImgCam1() != unaligned_image_id and pair->idImgCam2() != unaligned_image_id) {
		return;
	}

	auto cam2tocam1cand = rig->getOptTransform();

	if (!cam2tocam1cand.has_value()) {
		return;
	}

	StereoVision::Geometry::AffineTransform<float> cam2tocam1 = cam2tocam1cand.value();

	StereoVision::Geometry::AffineTransform<float> ImgUnalignedToRefImg = cam2tocam1;

	if (pair->idImgCam1() == unaligned_image_id) { //we need cam1tocam2 instead
		ImgUnalignedToRefImg = StereoVision::Geometry::AffineTransform<float>(cam2tocam1.R.transpose(), -cam2tocam1.R.transpose()*cam2tocam1.t);
	}

	Image* refImage = p->getDataBlock<Image>(ref_image_id);
	Image* unalignedImage = p->getDataBlock<Image>(unaligned_image_id);

	if (refImage == nullptr or unalignedImage == nullptr) {
		return;
	}

	auto refCamToWorldcand = refImage->getOptTransform();

	if (!refCamToWorldcand.has_value()) {
		return;
	}

	StereoVision::Geometry::AffineTransform refCamToWorld = refCamToWorldcand.value();
	StereoVision::Geometry::AffineTransform unalignedCamToWorld = refCamToWorld*ImgUnalignedToRefImg;

	unalignedImage->setOptTransform(unalignedCamToWorld);

}

} //namespace StereoVisionApp
