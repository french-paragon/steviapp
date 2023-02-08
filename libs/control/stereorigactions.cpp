#include "stereorigactions.h"

#include "datablocks/project.h"
#include "datablocks/stereorig.h"
#include "datablocks/image.h"

#include "interpolation/interpolation.h"
#include "interpolation/lensdistortionsmap.h"

#include "geometry/stereorigrectifier.h"

#include "vision/imageio.h"

#include "utils_functions.h"

#include "mainwindow.h"

#include <QFileDialog>

#include <QDebug>

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


void exportRectifiedImages(Project* p, qint64 rig_id, qint64 image_pair_id) {

	QString functionName = "exportRectifiedImage: ";

	if (p == nullptr) {
		qDebug() << functionName << "missing sequence";
		return;
	}

	MainWindow* mw = MainWindow::getActiveMainWindow();

	if (mw == nullptr) {
		qDebug() << functionName << "missing main windows";
		return;
	}

	QString outFolder = QFileDialog::getExistingDirectory(mw, QObject::tr("Export directory"));

	if (outFolder.isEmpty()) {
		qDebug() << functionName << "no out folder selected";
		return;
	}

	QDir out(outFolder);

	if (!out.exists()) {
		qDebug() << functionName << "non existent out folder";
		return;
	}

	StereoRig* rig = p->getDataBlock<StereoRig>(rig_id);

	if (rig == nullptr) {
		return;
	}

	ImagePair* pair = rig->getImagePair(image_pair_id);

	if (pair == nullptr) {
		return;
	}

	qint64 image1_id = pair->idImgCam1();
	qint64 image2_id = pair->idImgCam2();

	Image* img1 = p->getDataBlock<Image>(image1_id);
	Image* img2 = p->getDataBlock<Image>(image2_id);

	if (img1 == nullptr or img2 == nullptr) {
		return;
	}

	bool useOptimizedParametersSet = true;

	auto stereo_rectifier = configureRectifierForStereoPair(pair, useOptimizedParametersSet);

	if (stereo_rectifier == nullptr) {
		return;
	}

	//compute the rectification such that no part of the images are missing and the resolution is maximal.

	auto roiSetMethod = StereoVision::Geometry::StereoRigRectifier::TargetRangeSetMethod::Minimal;
	auto resolutionSetMethod = StereoVision::Geometry::StereoRigRectifier::TargetRangeSetMethod::Maximal;

	bool ok = stereo_rectifier->compute(roiSetMethod,
								 resolutionSetMethod);

	if (!ok) {
		return;
	}

	QString path1 = out.absoluteFilePath(img1->objectName() + ".png");
	QString path2 = out.absoluteFilePath(img2->objectName() + ".png");

	float gamma = 1.0;

	int originalFormatLeft;
	auto arrayIm1 = getImageData(img1->getImageFile(), gamma, &originalFormatLeft);
	int originalFormatRight;
	auto arrayIm2 = getImageData(img2->getImageFile(), gamma, &originalFormatRight);

	if (arrayIm1.empty() or arrayIm2.empty()) {
		qDebug() << functionName << "Could not open images";
		return;
	}

	auto transformedCam1 = StereoVision::Interpolation::interpolateImage(arrayIm1, stereo_rectifier->backWardMapCam1());
	auto transformedCam2 = StereoVision::Interpolation::interpolateImage(arrayIm2, stereo_rectifier->backWardMapCam2());

	saveImageData(path1, transformedCam1, gamma);
	saveImageData(path2, transformedCam2, gamma);

}

} //namespace StereoVisionApp
