#include "stereorigactions.h"

#include "datablocks/project.h"
#include "datablocks/stereorig.h"
#include "datablocks/image.h"
#include "datablocks/camera.h"

#include <StereoVision/interpolation/interpolation.h>
#include <StereoVision/interpolation/lensdistortionsmap.h>

#include <StereoVision/geometry/stereorigrectifier.h>

#include "vision/imageio.h"

#include "utils_functions.h"

#include "mainwindow.h"

#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>

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
		qDebug() << functionName << "missing project";
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


void exportRigDetails(Project* p,
					  qint64 rig_id,
					  QString outPath,
					  std::optional<float> cam1_f,
					  std::optional<std::array<float, 2>> cam1_pp,
					  std::optional<float> cam2_f,
					  std::optional<std::array<float, 2>> cam2_pp) {

	QString functionName = "exportRigDetails: ";

	if (p == nullptr) {
		qDebug() << functionName << "missing project";
		return;
	}

	QString rigFilePath;

	if (outPath.isEmpty()) {
		MainWindow* mw = MainWindow::getActiveMainWindow();

		if (mw == nullptr) {
			qDebug() << functionName << "missing main windows";
			return;
		}

		rigFilePath = QFileDialog::getSaveFileName(mw, QObject::tr("Export rig details"), QString(), {QObject::tr("rig files (*.rig)")});

	} else {
		rigFilePath = outPath;
	}

	if (rigFilePath.isEmpty()) {
		qDebug() << functionName << "no out file selected";
		return;
	}



	StereoRig* rig = p->getDataBlock<StereoRig>(rig_id);

	if (rig == nullptr) {
		qDebug() << functionName << "could not find rig";
		return;
	}

	auto imgPairs = rig->listTypedSubDataBlocks(ImagePair::staticMetaObject.className());

	if (imgPairs.size() <= 0) {
		qDebug() << functionName << "selected rig has no image pairs";
		return;
	}

	ImagePair* pair = rig->getImagePair(imgPairs[0]);

	if (pair == nullptr) {
		qDebug() << functionName << "selected rig has an invalid image pair";
		return;
	}

	qint64 image1_id = pair->idImgCam1();
	qint64 image2_id = pair->idImgCam2();

	Image* img1 = p->getDataBlock<Image>(image1_id);
	Image* img2 = p->getDataBlock<Image>(image2_id);

	if (img1 == nullptr or img2 == nullptr) {
		qDebug() << functionName << "could not find selected images";
		return;
	}

	qint64 img1_cam_id = img1->assignedCamera();
	qint64 img2_cam_id = img2->assignedCamera();

	Camera* cam1 = p->getDataBlock<Camera>(img1_cam_id);
	Camera* cam2 = p->getDataBlock<Camera>(img2_cam_id);

	if (cam1 == nullptr or cam2 == nullptr) {
		qDebug() << functionName << "could not get the associated camera to the selected images";
		return;
	}

	QJsonObject rep;

	rep.insert("f1", (cam1_f.has_value()) ? cam1_f.value() : cam1->optimizedFLen().value());

	QJsonArray pp1Array;

	if (cam1_pp.has_value()) {
		std::array<float, 2> pp = cam1_pp.value();
		pp1Array.push_back(pp[0]);
		pp1Array.push_back(pp[1]);
	} else {
		pp1Array.push_back(cam1->optimizedOpticalCenterX().value());
		pp1Array.push_back(cam1->optimizedOpticalCenterY().value());
	}
	rep.insert("pp1", pp1Array);

	rep.insert("f2", (cam2_f.has_value()) ? cam2_f.value() : cam2->optimizedFLen().value());

	QJsonArray pp2Array;

	if (cam2_pp.has_value()) {
		std::array<float, 2> pp = cam2_pp.value();
		pp2Array.push_back(pp[0]);
		pp2Array.push_back(pp[1]);
	} else {
		pp2Array.push_back(cam2->optimizedOpticalCenterX().value());
		pp2Array.push_back(cam2->optimizedOpticalCenterY().value());
	}
	rep.insert("pp2", pp2Array);

	auto opttransform = rig->getOptTransform();

	if (opttransform.has_value()) {
		StereoVision::Geometry::AffineTransform<float> transform = opttransform.value();

		QJsonArray Rarray;

		for (int i = 0; i < 3; i++) {
			QJsonArray line;

			for (int j = 0; j < 3; j++) {
				line.push_back(transform.R(i,j));
			}

			Rarray.push_back(line);
		}

		rep.insert("R", Rarray);

		QJsonArray tarray;

		for (int i = 0; i < 3; i++) {
			tarray.push_back(transform.t[i]);
		}

		rep.insert("t", tarray);

	}

	QFile outfile(rigFilePath);
	bool opened = outfile.open(QIODevice::WriteOnly);

	if (!opened) {
		qDebug() << functionName << "could not open out file";
		return;
	}

	QJsonDocument doc(rep);
	QByteArray data = doc.toJson();

	outfile.write(data);
	outfile.close();

}

} //namespace StereoVisionApp
