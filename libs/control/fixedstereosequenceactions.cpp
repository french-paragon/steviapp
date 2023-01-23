#include "fixedstereosequenceactions.h"

#include "datablocks/fixedcolorstereosequence.h"
#include "datablocks/fixedstereopluscolorsequence.h"

#include "datablocks/stereorig.h"
#include "datablocks/image.h"
#include "datablocks/camera.h"

#include "interpolation/interpolation.h"
#include "interpolation/lensdistortionsmap.h"

#include "geometry/stereorigrectifier.h"
#include "geometry/lensdistortion.h"
#include "geometry/imagecoordinates.h"
#include "geometry/alignement.h"

#include "correlation/unfold.h"
#include "correlation/correlation_base.h"
#include "correlation/cross_correlations.h"
#include "correlation/cost_based_refinement.h"
#include "correlation/disparity_plus_background_segmentation.h"

#include "imageProcessing/morphologicalOperators.h"

#include "vision/imageio.h"
#include "vision/pointcloudio.h"

#include "sparsesolver/helperfunctions.h"

#include "gui/stereosequenceexportoptiondialog.h"

#include "mainwindow.h"
#include "utils_functions.h"

#include <QFileDialog>

#include <QDebug>

namespace StereoVisionApp {

void exportColoredStereoImagesRectifiedImages(FixedColorStereoSequence* sequence, QVector<int> rows) {

	QString functionName = "exportColoredStereoImagesRectifiedImages: ";

	if (sequence == nullptr) {
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
		qDebug() << functionName << "Not output folder";
		return;
	}

	QDir outDir(outFolder);
	QDir inDir(sequence->baseFolder());

	Project* p = sequence->getProject();

	if (p == nullptr) {
		qDebug() << functionName << "missing project";
		return;
	}

	qint64 leftImgId = sequence->leftViewId();
	qint64 rightImgId = sequence->rightViewId();

	if (leftImgId < 0 or rightImgId < 0) {
		qDebug() << functionName << "missing images idxs";
		return;
	}

	if (leftImgId == rightImgId) {
		qDebug() << functionName << "two images are the same";
		return;
	}

	Image* leftImg = p->getDataBlock<Image>(leftImgId);
	Image* rightImg = p->getDataBlock<Image>(rightImgId);

	if (leftImg == nullptr or rightImg == nullptr) {
		qDebug() << functionName << "cannot find images in project";
		return;
	}

	ImagePair* stereoPair = ImagePair::getImagePairInProject(p, leftImgId, rightImgId);

	if (stereoPair == nullptr) {
		qDebug() << functionName << "cannot find stereo pair in project";
		return;
	}

	bool useOptimizedParametersSet = true;

	auto stereo_rectifier = configureRectifierForStereoPair(stereoPair, useOptimizedParametersSet);

	if (stereo_rectifier == nullptr) {
		qDebug() << functionName << "Cannot configure stereo rectifier";
		return;
	}

	//compute the rectification such that no part of the images are missing and the resolution is maximal.

	auto roiSetMethod = StereoVision::Geometry::StereoRigRectifier::TargetRangeSetMethod::Minimal;
	auto resolutionSetMethod = StereoVision::Geometry::StereoRigRectifier::TargetRangeSetMethod::Maximal;

	bool ok = stereo_rectifier->compute(roiSetMethod,
								 resolutionSetMethod);

	if (!ok) {
		qDebug() << functionName << "could not compute the stereo rectifier";
		return;
	}

	QVector<FixedColorStereoSequence::ImagePair> pairs = sequence->imgsPairs();
	float gamma = 1;

	StereoVision::Geometry::AffineTransform<float> cl_2_w = getImageToWorldTransform(leftImg).value().toAffineTransform();
	StereoVision::Geometry::AffineTransform<float> cr_2_w = getImageToWorldTransform(rightImg).value().toAffineTransform();

	Eigen::Matrix3f cRleft = (stereoPair->idImgCam1() == leftImgId) ? stereo_rectifier->CorrRCam1() : stereo_rectifier->CorrRCam2();
	Eigen::Matrix3f cRright = (stereoPair->idImgCam1() == rightImgId) ? stereo_rectifier->CorrRCam1() : stereo_rectifier->CorrRCam2();

	Eigen::Matrix3f nRleft = cl_2_w.R*cRleft;
	Eigen::Matrix3f nRright = cl_2_w.R*cRright;

	Eigen::Vector2f ppLeft = (stereoPair->idImgCam1() == leftImgId) ? stereo_rectifier->newPrincipalPointCam1() : stereo_rectifier->newPrincipalPointCam2();
	Eigen::Vector2f ppRight = (stereoPair->idImgCam1() == rightImgId) ? stereo_rectifier->newPrincipalPointCam1() : stereo_rectifier->newPrincipalPointCam2();

	QString infoPath = outDir.absoluteFilePath("infos.txt");

	QFile infos(infoPath);

	if (!infos.open(QFile::WriteOnly)) {
		return;
	}

	QTextStream infos_strm(&infos);

	infos_strm << "left2world:" << "\n";
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			infos_strm << nRleft(i,j) << ' ';
		}
		infos_strm << cl_2_w.t[i] << '\n';
	}
	infos_strm << "f_left:" << stereo_rectifier->reprojectionFLen() << "\n";
	infos_strm << "pp_left:" << "\n";
	for (int i = 0; i < 2; i++) {
		infos_strm << ppLeft[i] << ' ';
	}
	infos_strm << '\n';
	infos_strm << '\n';

	infos_strm << "right2world:" << "\n";
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			infos_strm << nRright(i,j) << ' ';
		}
		infos_strm << cr_2_w.t[i] << '\n';
	}
	infos_strm << "f_right:" << stereo_rectifier->reprojectionFLen() << "\n";
	infos_strm << "pp_right:" << "\n";
	for (int i = 0; i < 2; i++) {
		infos_strm << ppRight[i] << ' ';
	}
	infos_strm << '\n';
	infos_strm << '\n';

	infos_strm << "nbaseline:" << stereo_rectifier->normalizedBasline() << "\n";
	infos_strm << "disp_delta:" << stereo_rectifier->dispDelta() << "\n";

	for (int row : rows) {

		if (row < 0) {
			continue;
		}

		if (row >= pairs.size()) {
			continue;
		}

		FixedColorStereoSequence::ImagePair& pair = pairs[row];

		QString fileName;

		QFileInfo leftImgFileInfos(pair.StereoLeftImgPath);
		QFileInfo rightImgFileInfos(pair.StereoRightImgPath);

		QString leftBasename = leftImgFileInfos.baseName();
		QString rightBasename = rightImgFileInfos.baseName();

		int maxSize = std::max(leftBasename.size(), rightBasename.size());
		int minSize = std::min(leftBasename.size(), rightBasename.size());

		fileName.reserve(maxSize);

		for (int i = 0; i < maxSize; i++) {

			if (i >= minSize) {
				break;
			}

			if (leftBasename[i] != rightBasename[i]) {
				break;
			}

			fileName += leftBasename[i];
		}

		QString leftPath = outDir.absoluteFilePath(fileName + "rectified_left.png");
		QString rightPath = outDir.absoluteFilePath(fileName + "rectified_right.png");

		int originalFormatLeft;
		StereoVision::ImageArray arrayImLeft = getImageData(inDir.absoluteFilePath(pair.StereoLeftImgPath), gamma, &originalFormatLeft);
		int originalFormatRight;
		StereoVision::ImageArray arrayImRight = getImageData(inDir.absoluteFilePath(pair.StereoRightImgPath), gamma, &originalFormatRight);

		if (arrayImLeft.empty() or arrayImRight.empty()) {
			continue;
		}

		StereoVision::ImageArray transformedCamLeft;
		if (stereoPair->idImgCam1() == leftImgId) {
			transformedCamLeft = StereoVision::Interpolation::interpolateImage(arrayImLeft, stereo_rectifier->backWardMapCam1());
		} else {
			transformedCamLeft = StereoVision::Interpolation::interpolateImage(arrayImLeft, stereo_rectifier->backWardMapCam2());
		}

		StereoVision::ImageArray transformedCamRight;
		if (stereoPair->idImgCam1() == rightImgId) {
			transformedCamRight = StereoVision::Interpolation::interpolateImage(arrayImRight, stereo_rectifier->backWardMapCam1());
		} else {
			transformedCamRight = StereoVision::Interpolation::interpolateImage(arrayImRight, stereo_rectifier->backWardMapCam2());
		}

		saveImageData(leftPath, transformedCamLeft, gamma);
		saveImageData(rightPath, transformedCamRight, gamma);

	}
}
void exportStereoImagesPlusColorRectifiedImages(FixedStereoPlusColorSequence* sequence, QVector<int> rows) {

	QString functionName = "exportStereoImagesPlusColorRectifiedImages: ";

	if (sequence == nullptr) {
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
		qDebug() << functionName << "Not output folder";
		return;
	}

	QDir outDir(outFolder);
	QDir inDir(sequence->baseFolder());

	Project* p = sequence->getProject();

	if (p == nullptr) {
		qDebug() << functionName << "missing project";
		return;
	}

	qint64 leftImgId = sequence->leftViewId();
	qint64 rgbImgId = sequence->rgbViewId();
	qint64 rightImgId = sequence->rightViewId();

	if (leftImgId < 0 or rgbImgId < 0 or rightImgId < 0) {
		qDebug() << functionName << "missing images idxs";
		return;
	}

	if (leftImgId == rgbImgId or leftImgId == rightImgId or rightImgId  == rgbImgId) {
		qDebug() << functionName << "two images are the same";
		return;
	}

	Image* leftImg = p->getDataBlock<Image>(leftImgId);
	Image* rgbImg = p->getDataBlock<Image>(rgbImgId);
	Image* rightImg = p->getDataBlock<Image>(rightImgId);

	if (leftImg == nullptr or rgbImg == nullptr or rightImg == nullptr) {
		qDebug() << functionName << "cannot find images in project";
		return;
	}

	ImagePair* stereoPair = ImagePair::getImagePairInProject(p, leftImgId, rightImgId);

	if (stereoPair == nullptr) {
		qDebug() << functionName << "cannot find stereo pair in project";
		return;
	}

	bool useOptimizedParametersSet = true;

	auto stereo_rectifier = configureRectifierForStereoPair(stereoPair, useOptimizedParametersSet);

	if (stereo_rectifier == nullptr) {
		qDebug() << functionName << "Cannot configure stereo rectifier";
		return;
	}

	Camera* camRgb = p->getDataBlock<Camera>(rgbImg->assignedCamera());

	if (camRgb == nullptr) {
		qDebug() << functionName << "cannot get the rgb camera";
		return;
	}

	auto color_rectifier = configureRectifierForSingleCamera(camRgb, useOptimizedParametersSet);

	if (color_rectifier == nullptr) {
		qDebug() << functionName << "Cannot configure color rectifier";
		return;
	}

	if (!camRgb->optimizedFLen().isSet()) {
		qDebug() << functionName << "missing focal lenght in rgb camera";
		return;
	}

	if (!camRgb->optimizedOpticalCenterX().isSet() or !camRgb->optimizedOpticalCenterY().isSet()) {
		qDebug() << functionName << "missing principal point in rgb camera";
		return;
	}

	float rgbFlen = camRgb->optimizedFLen().value();

	Eigen::Vector2f rgbPP(camRgb->optimizedOpticalCenterX().value(), camRgb->optimizedOpticalCenterY().value());

	std::optional<Eigen::Vector3f> r_dist = std::nullopt;

	if (camRgb->optimizedK1().isSet() and camRgb->optimizedK2().isSet() and camRgb->optimizedK3().isSet()) {
		r_dist = Eigen::Vector3f(camRgb->optimizedK1().value(), camRgb->optimizedK2().value(), camRgb->optimizedK3().value());
	}

	std::optional<Eigen::Vector2f> t_dist = std::nullopt;

	if (camRgb->optimizedP1().isSet() and camRgb->optimizedP2().isSet()) {
		t_dist = Eigen::Vector2f(camRgb->optimizedP1().value(), camRgb->optimizedP2().value());
	}

	//compute the rectification such that no part of the images are missing and the resolution is maximal.

	auto roiSetMethod = StereoVision::Geometry::StereoRigRectifier::TargetRangeSetMethod::Minimal;
	auto resolutionSetMethod = StereoVision::Geometry::StereoRigRectifier::TargetRangeSetMethod::Maximal;

	bool ok = stereo_rectifier->compute(roiSetMethod,
								 resolutionSetMethod);

	if (!ok) {
		qDebug() << functionName << "could not compute the stereo rectifier";
		return;
	}

	ok = color_rectifier->compute(StereoVision::Geometry::ImageRectifier<float>::TargetRangeSetMethod::Minimal);

	if (!ok) {
		qDebug() << functionName << "could not compute the color rectifier";
		return;
	}

	QVector<FixedStereoPlusColorSequence::ImageTriplet> triplets = sequence->imgsTriplets();
	float gamma = 1;

	StereoVision::Geometry::AffineTransform<float> cl_2_w = getImageToWorldTransform(leftImg).value().toAffineTransform();
	StereoVision::Geometry::AffineTransform<float> cr_2_w = getImageToWorldTransform(rightImg).value().toAffineTransform();

	StereoVision::Geometry::AffineTransform<float> crgb_2_w = getImageToWorldTransform(rgbImg).value().toAffineTransform();

	Eigen::Matrix3f cRleft = (stereoPair->idImgCam1() == leftImgId) ? stereo_rectifier->CorrRCam1() : stereo_rectifier->CorrRCam2();
	Eigen::Matrix3f cRright = (stereoPair->idImgCam1() == rightImgId) ? stereo_rectifier->CorrRCam1() : stereo_rectifier->CorrRCam2();

	Eigen::Matrix3f nRleft = cl_2_w.R*cRleft;
	Eigen::Matrix3f nRright = cl_2_w.R*cRright;

	Eigen::Vector2f ppLeft = (stereoPair->idImgCam1() == leftImgId) ? stereo_rectifier->newPrincipalPointCam1() : stereo_rectifier->newPrincipalPointCam2();
	Eigen::Vector2f ppRight = (stereoPair->idImgCam1() == rightImgId) ? stereo_rectifier->newPrincipalPointCam1() : stereo_rectifier->newPrincipalPointCam2();

	QString infoPath = outDir.absoluteFilePath("infos.txt");

	QFile infos(infoPath);

	if (!infos.open(QFile::WriteOnly)) {
		return;
	}

	QTextStream infos_strm(&infos);

	infos_strm << "left2world:" << "\n";
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			infos_strm << nRleft(i,j) << ' ';
		}
		infos_strm << cl_2_w.t[i] << '\n';
	}
	infos_strm << "f_left:" << stereo_rectifier->reprojectionFLen() << "\n";
	infos_strm << "pp_left:" << "\n";
	for (int i = 0; i < 2; i++) {
		infos_strm << ppLeft[i] << ' ';
	}
	infos_strm << '\n';
	infos_strm << '\n';

	infos_strm << "right2world:" << "\n";
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			infos_strm << nRright(i,j) << ' ';
		}
		infos_strm << cr_2_w.t[i] << '\n';
	}
	infos_strm << "f_right:" << stereo_rectifier->reprojectionFLen() << "\n";
	infos_strm << "pp_right:" << "\n";
	for (int i = 0; i < 2; i++) {
		infos_strm << ppRight[i] << ' ';
	}
	infos_strm << '\n';
	infos_strm << '\n';

	infos_strm << "rgb2world:" << "\n";
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			infos_strm << crgb_2_w.R(i,j) << ' ';
		}
		infos_strm << crgb_2_w.t[i] << '\n';
	}
	infos_strm << "f_rgb:" << rgbFlen << "\n";
	infos_strm << "pp_rgb:" << "\n";
	for (int i = 0; i < 2; i++) {
		infos_strm << rgbPP[i] << ' ';
	}
	infos_strm << '\n';
	infos_strm << '\n';

	infos_strm << "nbaseline:" << stereo_rectifier->normalizedBasline() << "\n";
	infos_strm << "disp_delta:" << stereo_rectifier->dispDelta() << "\n";

	for (int row : rows) {

		if (row < 0) {
			continue;
		}

		if (row >= triplets.size()) {
			continue;
		}

		FixedStereoPlusColorSequence::ImageTriplet& triplet = triplets[row];

		QString fileName;

		QFileInfo leftImgFileInfos(triplet.StereoLeftImgPath);
		QFileInfo rightImgFileInfos(triplet.StereoRightImgPath);
		QFileInfo rgbImgFileInfos(triplet.ColorImagePath);

		QString leftBasename = leftImgFileInfos.baseName();
		QString rightBasename = rightImgFileInfos.baseName();
		QString colorBasename = rgbImgFileInfos.baseName();

		int maxSize = std::max(colorBasename.size(), std::max(leftBasename.size(), rightBasename.size()));
		int minSize = std::min(colorBasename.size(), std::min(leftBasename.size(), rightBasename.size()));

		fileName.reserve(maxSize);

		for (int i = 0; i < maxSize; i++) {

			if (i >= minSize) {
				break;
			}

			if (leftBasename[i] != rightBasename[i]) {
				break;
			}

			if (leftBasename[i] != colorBasename[i]) {
				break;
			}

			fileName += leftBasename[i];
		}

		QString rgbPath = outDir.absoluteFilePath(fileName + "rectified_rgb.png");
		QString leftPath = outDir.absoluteFilePath(fileName + "rectified_left.png");
		QString rightPath = outDir.absoluteFilePath(fileName + "rectified_right.png");

		int originalFormatLeft;
		StereoVision::ImageArray arrayImLeft = getImageData(inDir.absoluteFilePath(triplet.StereoLeftImgPath), gamma, &originalFormatLeft);
		int originalFormatRight;
		StereoVision::ImageArray arrayImRight = getImageData(inDir.absoluteFilePath(triplet.StereoRightImgPath), gamma, &originalFormatRight);
		int originalFormatRgb;
		StereoVision::ImageArray arrayImRgb = getImageData(inDir.absoluteFilePath(triplet.ColorImagePath), gamma, &originalFormatRgb);

		if (arrayImLeft.empty() or arrayImRight.empty() or arrayImRgb.empty()) {
			continue;
		}

		StereoVision::ImageArray transformedCamLeft;
		if (stereoPair->idImgCam1() == leftImgId) {
			transformedCamLeft = StereoVision::Interpolation::interpolateImage(arrayImLeft, stereo_rectifier->backWardMapCam1());
		} else {
			transformedCamLeft = StereoVision::Interpolation::interpolateImage(arrayImLeft, stereo_rectifier->backWardMapCam2());
		}

		StereoVision::ImageArray transformedCamRight;
		if (stereoPair->idImgCam1() == rightImgId) {
			transformedCamRight = StereoVision::Interpolation::interpolateImage(arrayImRight, stereo_rectifier->backWardMapCam1());
		} else {
			transformedCamRight = StereoVision::Interpolation::interpolateImage(arrayImRight, stereo_rectifier->backWardMapCam2());
		}

		StereoVision::ImageArray rectifiedImRgb = StereoVision::Interpolation::interpolateImage(arrayImRgb, color_rectifier->backWardMap());

		saveImageData(rgbPath, rectifiedImRgb, gamma);
		saveImageData(leftPath, transformedCamLeft, gamma);
		saveImageData(rightPath, transformedCamRight, gamma);

	}

}

void exportColoredStereoImagesPointCloud(FixedColorStereoSequence* sequence, QVector<int> rows) {

	constexpr Multidim::AccessCheck Nc = Multidim::AccessCheck::Nocheck;

	if (sequence == nullptr) {
		return;
	}

	MainWindow* mw = MainWindow::getActiveMainWindow();

	if (mw == nullptr) {
		return;
	}

	QString outFolder = QFileDialog::getExistingDirectory(mw, QObject::tr("Export directory"));

	if (outFolder.isEmpty()) {
		return;
	}

	QDir outDir(outFolder);
	QDir inDir(sequence->baseFolder());

	Project* p = sequence->getProject();

	if (p == nullptr) {
		return;
	}

	qint64 leftImgId = sequence->leftViewId();
	qint64 rightImgId = sequence->rightViewId();

	if (leftImgId < 0 or rightImgId < 0) {
		return;
	}

	if (leftImgId == rightImgId) {
		return;
	}

	Image* leftImg = p->getDataBlock<Image>(leftImgId);
	Image* rightImg = p->getDataBlock<Image>(rightImgId);

	if (leftImg == nullptr or rightImg == nullptr) {
		return;
	}

	ImagePair* stereoPair = ImagePair::getImagePairInProject(p, leftImgId, rightImgId);

	if (stereoPair == nullptr) {
		return;
	}

	bool useOptimizedParametersSet = true;

	auto rectifier = configureRectifierForStereoPair(stereoPair, useOptimizedParametersSet);

	if (rectifier == nullptr) {
		return;
	}

	//compute the rectification such that no part of the images are missing and the resolution is maximal.

	auto roiSetMethod = StereoVision::Geometry::StereoRigRectifier::TargetRangeSetMethod::Minimal;
	auto resolutionSetMethod = StereoVision::Geometry::StereoRigRectifier::TargetRangeSetMethod::Maximal;

	bool ok = rectifier->compute(roiSetMethod,
								 resolutionSetMethod);

	if (!ok) {
		return;
	}

	QVector<FixedColorStereoSequence::ImagePair> pairs = sequence->imgsPairs();
	float gamma = 1;

	uint8_t search_radius = 3;
	uint8_t search_width = 250;

	constexpr StereoVision::Correlation::matchingFunctions matchFunc = StereoVision::Correlation::matchingFunctions::SAD;
	using T_L = float;
	using T_R = float;
	constexpr int nImDim = 3;
	constexpr StereoVision::Correlation::dispDirection dDir = StereoVision::Correlation::dispDirection::RightToLeft;
	using T_CV = float;

	using disp_t = int;

	StereoVision::Geometry::AffineTransform cl_2_w = getImageToWorldTransform(leftImg).value().toAffineTransform();
	StereoVision::Geometry::AffineTransform cr_2_w = getImageToWorldTransform(rightImg).value().toAffineTransform();

	for (int row : rows) {

		if (row < 0) {
			continue;
		}

		if (row >= pairs.size()) {
			continue;
		}

		FixedColorStereoSequence::ImagePair& pair = pairs[row];

		QString fileName;
		QString filePath;

		QFileInfo leftImgFileInfos(pair.StereoLeftImgPath);
		QFileInfo rightImgFileInfos(pair.StereoRightImgPath);

		QString leftBasename = leftImgFileInfos.baseName();
		QString rightBasename = rightImgFileInfos.baseName();

		int maxSize = std::max(leftBasename.size(), rightBasename.size());
		int minSize = std::min(leftBasename.size(), rightBasename.size());

		fileName.reserve(maxSize);

		for (int i = 0; i < maxSize; i++) {

			if (i >= minSize) {
				break;
			}

			if (leftBasename[i] != rightBasename[i]) {
				break;
			}

			fileName += leftBasename[i];
		}

		filePath = outDir.absoluteFilePath(fileName + "PointsCloud.pcd");

		int originalFormatLeft;
		StereoVision::ImageArray arrayImLeft = getImageData(inDir.absoluteFilePath(pair.StereoLeftImgPath), gamma, &originalFormatLeft);
		int originalFormatRight;
		StereoVision::ImageArray arrayImRight = getImageData(inDir.absoluteFilePath(pair.StereoRightImgPath), gamma, &originalFormatRight);

		if (arrayImLeft.empty() or arrayImRight.empty()) {
			continue;
		}

		StereoVision::ImageArray transformedCamLeft;
		if (stereoPair->idImgCam1() == leftImgId) {
			transformedCamLeft = StereoVision::Interpolation::interpolateImage(arrayImLeft, rectifier->backWardMapCam1());
		} else {
			transformedCamLeft = StereoVision::Interpolation::interpolateImage(arrayImLeft, rectifier->backWardMapCam2());
		}

		StereoVision::ImageArray transformedCamRight;
		if (stereoPair->idImgCam1() == rightImgId) {
			transformedCamRight = StereoVision::Interpolation::interpolateImage(arrayImRight, rectifier->backWardMapCam1());
		} else {
			transformedCamRight = StereoVision::Interpolation::interpolateImage(arrayImRight, rectifier->backWardMapCam2());
		}

		float dispDelta = rectifier->dispDelta();
		float normalizedBasline = rectifier->normalizedBasline();
		float reprojectionFLen = rectifier->reprojectionFLen();

		Eigen::Matrix3f RCamR;
		if (stereoPair->idImgCam1() == rightImgId) {
			RCamR = cr_2_w.R*rectifier->CorrRCam1();
		} else {
			RCamR = cr_2_w.R*rectifier->CorrRCam2();
		}

		Eigen::Vector3f tCamL = cl_2_w.t;
		Eigen::Vector3f tCamR = cr_2_w.t;

		Eigen::Vector2f ppCamR;
		if (stereoPair->idImgCam1() == rightImgId) {
			ppCamR = rectifier->newPrincipalPointCam1();
		} else {
			ppCamR = rectifier->newPrincipalPointCam2();
		}

		Multidim::Array<T_CV, 3> cost_volume =
		StereoVision::Correlation::unfoldBasedCostVolume<matchFunc, T_L, T_R, nImDim, dDir, T_CV>
				(transformedCamLeft, transformedCamRight, search_radius, search_radius, search_width);


		constexpr StereoVision::Correlation::dispExtractionStartegy extractionStrategy =
				StereoVision::Correlation::MatchingFunctionTraits<matchFunc>::extractionStrategy;

		Multidim::Array<disp_t, 2> raw_disp = StereoVision::Correlation::extractSelectedIndex<extractionStrategy>(cost_volume);

		Multidim::Array<T_CV, 3> truncated_cost_volume = StereoVision::Correlation::truncatedCostVolume(cost_volume, raw_disp, search_radius, search_radius, 1);

		constexpr StereoVision::Correlation::InterpolationKernel kernel =
				StereoVision::Correlation::InterpolationKernel::Equiangular;

		Multidim::Array<float, 2> refined_disp = StereoVision::Correlation::refineDispCostInterpolation<kernel>(truncated_cost_volume, raw_disp);

		auto shape = raw_disp.shape();

		Multidim::Array<float, 3> Points(shape[0], shape[1], 3);
		Multidim::Array<float, 3> PointsColor(shape[0], shape[1], 3);

		for (int i = 0; i < shape[0]; i++) {
			for (int j = 0; j < shape[1]; j++) {

				float d = refined_disp.value<Nc>(i,j);

				float z = normalizedBasline/(d - dispDelta);

				float x = z*(j - ppCamR.x())/reprojectionFLen;
				float y = z*(i - ppCamR.y())/reprojectionFLen;

				Eigen::Vector3f camCoord(x,y,z);
				Eigen::Vector3f worldCoord = RCamR*camCoord + tCamR;

				float r = arrayImRight.value<Nc>(i,j,0);
				float g = arrayImRight.value<Nc>(i,j,1);
				float b = arrayImRight.value<Nc>(i,j,2);

				Points.at<Nc>(i,j,0) = worldCoord.x();
				Points.at<Nc>(i,j,1) = worldCoord.y();
				Points.at<Nc>(i,j,2) = worldCoord.z();

				PointsColor.at<Nc>(i,j,0) = r;
				PointsColor.at<Nc>(i,j,1) = g;
				PointsColor.at<Nc>(i,j,2) = b;
			}
		}

		float colorScale = 1.0;
		bool correctgamma = false;

		saveXYZRGBpointcloud(filePath, Points, PointsColor, colorScale, correctgamma);
	}


}

void exportStereoImagesPlusColorPointCloud(FixedStereoPlusColorSequence* sequence, QVector<int> rows) {

	constexpr Multidim::AccessCheck Nc = Multidim::AccessCheck::Nocheck;

	QString functionName = "exportStereoImagesPlusColorPointCloud: ";

	if (sequence == nullptr) {
		qDebug() << functionName << "missing sequence";
		return;
	}

	MainWindow* mw = MainWindow::getActiveMainWindow();

	if (mw == nullptr) {
		qDebug() << functionName << "missing main windows";
		return;
	}

	StereoSequenceExportOptionDialog exportDialog(mw);

	exportDialog.setSearchRadius(5);
	exportDialog.setSearchWidth(150);
	exportDialog.setMaxDist(3000);
	exportDialog.setErodingDistance(0);
	exportDialog.setOpeningDistance(0);

	int code = exportDialog.exec();

	if (code != QDialog::Accepted) {
		qDebug() << functionName << "export cancelled";
		return;
	}

	QString outFolder = exportDialog.exportDir();

	if (outFolder.isEmpty()) {
		qDebug() << functionName << "Not output folder";
		return;
	}

	QDir outDir(outFolder);
	QDir inDir(sequence->baseFolder());

	Project* p = sequence->getProject();

	if (p == nullptr) {
		qDebug() << functionName << "missing project";
		return;
	}

	qint64 leftImgId = sequence->leftViewId();
	qint64 rgbImgId = sequence->rgbViewId();
	qint64 rightImgId = sequence->rightViewId();

	if (leftImgId < 0 or rgbImgId < 0 or rightImgId < 0) {
		qDebug() << functionName << "missing images idxs";
		return;
	}

	if (leftImgId == rgbImgId or leftImgId == rightImgId or rightImgId  == rgbImgId) {
		qDebug() << functionName << "two images are the same";
		return;
	}

	Image* leftImg = p->getDataBlock<Image>(leftImgId);
	Image* rgbImg = p->getDataBlock<Image>(rgbImgId);
	Image* rightImg = p->getDataBlock<Image>(rightImgId);

	if (leftImg == nullptr or rgbImg == nullptr or rightImg == nullptr) {
		qDebug() << functionName << "cannot find images in project";
		return;
	}

	ImagePair* stereoPair = ImagePair::getImagePairInProject(p, leftImgId, rightImgId);

	if (stereoPair == nullptr) {
		qDebug() << functionName << "cannot find stereo pair in project";
		return;
	}

	bool useOptimizedParametersSet = true;

	auto stereo_rectifier = configureRectifierForStereoPair(stereoPair, useOptimizedParametersSet);

	if (stereo_rectifier == nullptr) {
		qDebug() << functionName << "Cannot configure stereo rectifier";
		return;
	}

	Camera* camRgb = p->getDataBlock<Camera>(rgbImg->assignedCamera());

	if (camRgb == nullptr) {
		qDebug() << functionName << "cannot get the rgb camera";
		return;
	}

	auto color_rectifier = configureRectifierForSingleCamera(camRgb, useOptimizedParametersSet);

	if (color_rectifier == nullptr) {
		qDebug() << functionName << "Cannot configure color rectifier";
		return;
	}

	if (!camRgb->optimizedFLen().isSet()) {
		qDebug() << functionName << "missing focal lenght in rgb camera";
		return;
	}

	if (!camRgb->optimizedOpticalCenterX().isSet() or !camRgb->optimizedOpticalCenterY().isSet()) {
		qDebug() << functionName << "missing principal point in rgb camera";
		return;
	}

	float rgbFlen = camRgb->optimizedFLen().value();

	Eigen::Vector2f rgbPP(camRgb->optimizedOpticalCenterX().value(), camRgb->optimizedOpticalCenterY().value());

	std::optional<Eigen::Vector3f> r_dist = std::nullopt;

	if (camRgb->optimizedK1().isSet() and camRgb->optimizedK2().isSet() and camRgb->optimizedK3().isSet()) {
		r_dist = Eigen::Vector3f(camRgb->optimizedK1().value(), camRgb->optimizedK2().value(), camRgb->optimizedK3().value());
	}

	std::optional<Eigen::Vector2f> t_dist = std::nullopt;

	if (camRgb->optimizedP1().isSet() and camRgb->optimizedP2().isSet()) {
		t_dist = Eigen::Vector2f(camRgb->optimizedP1().value(), camRgb->optimizedP2().value());
	}

	//compute the rectification such that no part of the images are missing and the resolution is maximal.

	auto roiSetMethod = StereoVision::Geometry::StereoRigRectifier::TargetRangeSetMethod::Minimal;
	auto resolutionSetMethod = StereoVision::Geometry::StereoRigRectifier::TargetRangeSetMethod::Maximal;

	bool ok = stereo_rectifier->compute(roiSetMethod,
								 resolutionSetMethod);

	if (!ok) {
		qDebug() << functionName << "could not compute the stereo rectifier";
		return;
	}

	ok = color_rectifier->compute(StereoVision::Geometry::ImageRectifier<float>::TargetRangeSetMethod::Minimal);

	if (!ok) {
		qDebug() << functionName << "could not compute the color rectifier";
		return;
	}

	QVector<FixedStereoPlusColorSequence::ImageTriplet> triplets = sequence->imgsTriplets();
	float gamma = 1;

	uint8_t search_radius = exportDialog.searchRadius();
	StereoVision::Correlation::disp_t search_width = exportDialog.searchWidth();
	float max_dist = exportDialog.maxDist();

	constexpr StereoVision::Correlation::matchingFunctions matchFunc = StereoVision::Correlation::matchingFunctions::NCC;
	using T_F = float;
	using T_L = T_F;
	using T_R = T_F;
	constexpr int nImDim = 3;
	constexpr StereoVision::Correlation::dispDirection dDir = StereoVision::Correlation::dispDirection::RightToLeft;
	using T_CV = float;

	using disp_t = int;

	StereoVision::Geometry::AffineTransform<float> cl_2_w = getImageToWorldTransform(leftImg).value().toAffineTransform();
	StereoVision::Geometry::AffineTransform<float> cr_2_w = getImageToWorldTransform(rightImg).value().toAffineTransform();

	StereoVision::Geometry::AffineTransform<float> cw_2_rgb = getWorldToImageTransform(rgbImg).value().toAffineTransform();

	float dispDelta = stereo_rectifier->dispDelta();
	float normalizedBasline = stereo_rectifier->normalizedBasline();
	float reprojectionFLen = stereo_rectifier->reprojectionFLen();

	Eigen::Matrix3f RCamR;
	if (stereoPair->idImgCam1() == rightImgId) {
		RCamR = cr_2_w.R*stereo_rectifier->CorrRCam1();
	} else {
		RCamR = cr_2_w.R*stereo_rectifier->CorrRCam2();
	}

	Eigen::Vector3f tCamL = cl_2_w.t;
	Eigen::Vector3f tCamR = cr_2_w.t;

	Eigen::Vector2f ppCamR;
	if (stereoPair->idImgCam1() == rightImgId) {
		ppCamR = stereo_rectifier->newPrincipalPointCam1();
	} else {
		ppCamR = stereo_rectifier->newPrincipalPointCam2();
	}

	Eigen::Vector2f newPpRight = (stereoPair->idImgCam1() == rightImgId) ? stereo_rectifier->newPrincipalPointCam1() : stereo_rectifier->newPrincipalPointCam2();

	StereoVision::Geometry::ImageToImageReprojector<float> rightToColorReprojector
			(stereo_rectifier->reprojectionFLen(),
			 newPpRight,
			 rgbFlen,
			 color_rectifier->targetPP(),
			 cw_2_rgb*cr_2_w);

	using MatcherT = StereoVision::Correlation::DisparityEstimatorWithBackgroundRemoval<matchFunc, T_CV, T_F>;

	constexpr float relative_threshold = 1.0;
	constexpr disp_t disp_threshold = 2;
	MatcherT matcher(relative_threshold, disp_threshold);


	int originalFormatBgLeft;
	StereoVision::ImageArray bgImLeft = getImageData(inDir.absoluteFilePath(leftImg->getImageFile()), gamma, &originalFormatBgLeft);
	int originalFormatBgRight;
	StereoVision::ImageArray bgImRight = getImageData(inDir.absoluteFilePath(rightImg->getImageFile()), gamma, &originalFormatBgRight);
	int originalFormatBgRgb;
	StereoVision::ImageArray bgImRgb = getImageData(inDir.absoluteFilePath(rgbImg->getImageFile()), gamma, &originalFormatBgRgb);

	if (bgImLeft.empty() or bgImRight.empty() or bgImRgb.empty()) {
		qDebug() << functionName << "could not load the background images";
		return;
	}

	StereoVision::ImageArray transformedBgLeft;
	if (stereoPair->idImgCam1() == leftImgId) {
		transformedBgLeft = StereoVision::Interpolation::interpolateImage(bgImLeft, stereo_rectifier->backWardMapCam1());
	} else {
		transformedBgLeft = StereoVision::Interpolation::interpolateImage(bgImLeft, stereo_rectifier->backWardMapCam2());
	}

	StereoVision::ImageArray transformedBgRight;
	if (stereoPair->idImgCam1() == rightImgId) {
		transformedBgRight = StereoVision::Interpolation::interpolateImage(bgImRight, stereo_rectifier->backWardMapCam1());
	} else {
		transformedBgRight = StereoVision::Interpolation::interpolateImage(bgImRight, stereo_rectifier->backWardMapCam2());
	}

	Multidim::Array<T_F, 3> fLeftBg = StereoVision::Correlation::unfold(search_radius, search_radius, transformedBgLeft);
	Multidim::Array<T_F, 3> fRightBg = StereoVision::Correlation::unfold(search_radius, search_radius, transformedBgRight);

	StereoVision::Correlation::searchOffset<1> search_offset(0, search_width);
	bool computed = matcher.computeBackgroundDisp(fRightBg, fLeftBg, search_offset);

	if (!computed) {
		qDebug() << functionName << "could not compute the background disp";
		return;
	}


	for (int row : rows) {

		if (row < 0) {
			continue;
		}

		if (row >= triplets.size()) {
			continue;
		}

		FixedStereoPlusColorSequence::ImageTriplet& triplet = triplets[row];

		QString fileName;
		QString filePath;

		QFileInfo leftImgFileInfos(triplet.StereoLeftImgPath);
		QFileInfo rightImgFileInfos(triplet.StereoRightImgPath);
		QFileInfo rgbImgFileInfos(triplet.ColorImagePath);

		QString leftBasename = leftImgFileInfos.baseName();
		QString rightBasename = rightImgFileInfos.baseName();
		QString colorBasename = rgbImgFileInfos.baseName();

		int maxSize = std::max(colorBasename.size(), std::max(leftBasename.size(), rightBasename.size()));
		int minSize = std::min(colorBasename.size(), std::min(leftBasename.size(), rightBasename.size()));

		fileName.reserve(maxSize);

		for (int i = 0; i < maxSize; i++) {

			if (i >= minSize) {
				break;
			}

			if (leftBasename[i] != rightBasename[i]) {
				break;
			}

			if (leftBasename[i] != colorBasename[i]) {
				break;
			}

			fileName += leftBasename[i];
		}

		filePath = outDir.absoluteFilePath(fileName + "PointsCloud.pcd");

		int originalFormatLeft;
		StereoVision::ImageArray arrayImLeft = getImageData(inDir.absoluteFilePath(triplet.StereoLeftImgPath), gamma, &originalFormatLeft);
		int originalFormatRight;
		StereoVision::ImageArray arrayImRight = getImageData(inDir.absoluteFilePath(triplet.StereoRightImgPath), gamma, &originalFormatRight);
		int originalFormatRgb;
		StereoVision::ImageArray arrayImRgb = getImageData(inDir.absoluteFilePath(triplet.ColorImagePath), gamma, &originalFormatRgb);

		if (arrayImLeft.empty() or arrayImRight.empty() or arrayImRgb.empty()) {
			continue;
		}

		StereoVision::ImageArray transformedCamLeft;
		if (stereoPair->idImgCam1() == leftImgId) {
			transformedCamLeft = StereoVision::Interpolation::interpolateImage(arrayImLeft, stereo_rectifier->backWardMapCam1());
		} else {
			transformedCamLeft = StereoVision::Interpolation::interpolateImage(arrayImLeft, stereo_rectifier->backWardMapCam2());
		}

		StereoVision::ImageArray transformedCamRight;
		if (stereoPair->idImgCam1() == rightImgId) {
			transformedCamRight = StereoVision::Interpolation::interpolateImage(arrayImRight, stereo_rectifier->backWardMapCam1());
		} else {
			transformedCamRight = StereoVision::Interpolation::interpolateImage(arrayImRight, stereo_rectifier->backWardMapCam2());
		}

		StereoVision::ImageArray rectifiedImRgb = StereoVision::Interpolation::interpolateImage(arrayImRgb, color_rectifier->backWardMap());



		Multidim::Array<T_F, 3> fLeft = StereoVision::Correlation::unfold(search_radius, search_radius, transformedCamLeft);
		Multidim::Array<T_F, 3> fRight = StereoVision::Correlation::unfold(search_radius, search_radius, transformedCamRight);

		using searchSpaceType = MatcherT::template OnDemandCVT<Multidim::NonConstView, Multidim::NonConstView>::SearchSpaceType;
		searchSpaceType searchSpace(StereoVision::Correlation::SearchSpaceBase::IgnoredDim(),
									StereoVision::Correlation::SearchSpaceBase::SearchDim(0, search_width),
									StereoVision::Correlation::SearchSpaceBase::FeatureDim());

		MatcherT::template OnDemandCVT<Multidim::NonConstView, Multidim::NonConstView> onDemandCv(fRight, fLeft, searchSpace);


		constexpr StereoVision::Correlation::StereoDispWithBgMask::MaskInfo Foreground =  StereoVision::Correlation::StereoDispWithBgMask::Foreground;
		constexpr StereoVision::Correlation::StereoDispWithBgMask::MaskInfo Background =  StereoVision::Correlation::StereoDispWithBgMask::Background;

		auto computed = matcher.computeDispAndForegroundMask(onDemandCv);
		Multidim::Array<disp_t, 2>const& raw_disp = computed.disp;
		Multidim::Array<StereoVision::Correlation::StereoDispWithBgMask::MaskInfo, 2>const& m_info = computed.fg_mask;
		Multidim::Array<uint8_t, 2> mask_info = m_info.cast<uint8_t>();

		int openDistance = exportDialog.openingDistance();
		int erodeDistance = exportDialog.erodingDistance();

		Multidim::Array<uint8_t, 2> filtered;
		const Multidim::Array<uint8_t, 2>* sMask;

		if (openDistance > 0 and erodeDistance > 0) {
			filtered = StereoVision::ImageProcessing::erosion(erodeDistance, erodeDistance,
															  StereoVision::ImageProcessing::dilation(openDistance, openDistance,
																									  StereoVision::ImageProcessing::erosion(openDistance, openDistance, mask_info)));
			sMask = &filtered;
		} else if (openDistance > 0) {
			filtered = StereoVision::ImageProcessing::dilation(openDistance, openDistance,
															   StereoVision::ImageProcessing::erosion(openDistance, openDistance, mask_info));
			sMask = &filtered;
		} else if (erodeDistance > 0) {
			filtered = StereoVision::ImageProcessing::erosion(erodeDistance, erodeDistance, mask_info);
			sMask = &filtered;
		} else {
			sMask = &mask_info;
		}

		Multidim::Array<T_CV, 3> truncated_cost_volume = onDemandCv.truncatedCostVolume(raw_disp);


		auto shape = raw_disp.shape();

		constexpr StereoVision::Correlation::InterpolationKernel kernel =
				StereoVision::Correlation::InterpolationKernel::Equiangular;

		Multidim::Array<float, 2> refined_disp = StereoVision::Correlation::refineDispCostInterpolation<kernel>(truncated_cost_volume, raw_disp);

		Multidim::Array<float, 3> Points(shape[0], shape[1], 3);
		Multidim::Array<float, 3> PointsColor(shape[0], shape[1], 3);

		for (int i = 0; i < shape[0]; i++) {
			for (int j = 0; j < shape[1]; j++) {

				disp_t r_d = raw_disp.value<Nc>(i,j);
				float d = refined_disp.value<Nc>(i,j);

				float z = normalizedBasline/(d - dispDelta);

				if (z > max_dist or z < 0 or r_d < 0) {
					z = std::nanf("");
				}

				float x = z*(j - newPpRight.x())/reprojectionFLen;
				float y = z*(i - newPpRight.y())/reprojectionFLen;

				Eigen::Vector3f camCoord(x,y,z);
				Eigen::Vector3f worldCoord = RCamR*camCoord + tCamR;

				Eigen::Vector2f rgbPixCoord =
						StereoVision::Geometry::World2DistortedImageCoordinates(worldCoord, cw_2_rgb, rgbFlen, color_rectifier->targetPP(), r_dist, t_dist);

				int col_i = std::round(rgbPixCoord.y());
				int col_j = std::round(rgbPixCoord.x());

				float r = rectifiedImRgb.valueOrAlt({col_i,col_j,0}, std::nanf(""));
				float g = rectifiedImRgb.valueOrAlt({col_i,col_j,1}, std::nanf(""));
				float b = rectifiedImRgb.valueOrAlt({col_i,col_j,2}, std::nanf(""));

				auto m = sMask->value<Nc>(i,j);

				if (m == Foreground) {
					Points.at<Nc>(i,j,0) = worldCoord.x();
					Points.at<Nc>(i,j,1) = worldCoord.y();
					Points.at<Nc>(i,j,2) = worldCoord.z();
				} else {
					Points.at<Nc>(i,j,0) = std::nanf("");
					Points.at<Nc>(i,j,1) = std::nanf("");
					Points.at<Nc>(i,j,2) = std::nanf("");
				}

				PointsColor.at<Nc>(i,j,0) = r;
				PointsColor.at<Nc>(i,j,1) = g;
				PointsColor.at<Nc>(i,j,2) = b;
			}
		}

		float colorScale = 255.0;
		bool correctgamma = false;

		saveXYZRGBpointcloud(filePath, Points, PointsColor, colorScale, correctgamma);
	}

	qDebug() << functionName << "finished task";


}

} //namespace StereoVisionApp
