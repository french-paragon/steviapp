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
#include "correlation/hierarchical.h"

#include "imageProcessing/morphologicalOperators.h"
#include "imageProcessing/foregroundSegmentation.h"

#include "io/image_io.h"

#include "vision/imageio.h"
#include "vision/pointcloudio.h"

#include "sparsesolver/helperfunctions.h"

#include "gui/stereosequenceexportoptiondialog.h"
#include "gui/stereosequenceimageexportoptiondialog.h"

#include "mainwindow.h"
#include "utils_functions.h"

#include <array>
#include <vector>
#include <algorithm>

#include <QFileDialog>

#include <QDebug>

namespace StereoVisionApp {


bool writePointCloudAsStevimgs(QString folderName,
							   QString fileBaseName,
							   Multidim::Array<float, 3> const& coloredImg,
							   Multidim::Array<float, 2> const& disparityMap,
							   Multidim::Array<uint8_t, 2> const& mask) {

	QDir path(folderName);

	bool status = path.mkpath(".");

	if (!status) {
		qDebug() << Q_FUNC_INFO << "Failed to create output directory";
		return false;
	}

	QString colorImageFilename = path.filePath(fileBaseName + ((fileBaseName.endsWith("_")) ? "" : "_") + "color.stevimg");
	status &= StereoVision::IO::writeStevimg<float, float, 3>(colorImageFilename.toStdString(), coloredImg);

	QString disparityMapFilename = path.filePath(fileBaseName + ((fileBaseName.endsWith("_")) ? "" : "_") + "disp.stevimg");
	status &= StereoVision::IO::writeStevimg<float, float, 2>(disparityMapFilename.toStdString(), disparityMap);

	QString maskFilename = path.filePath(fileBaseName + ((fileBaseName.endsWith("_")) ? "" : "_") + "mask.stevimg");
	status &= StereoVision::IO::writeStevimg<uint8_t, uint8_t, 2>(maskFilename.toStdString(), mask);

	return status;

}

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
	exportDialog.setMaxDispDelta(15);
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


		constexpr StereoVision::Correlation::StereoDispWithBgMask::MaskInfo Foreground =  StereoVision::ImageProcessing::FgBgSegmentation::Foreground;
		constexpr StereoVision::Correlation::StereoDispWithBgMask::MaskInfo Background =  StereoVision::ImageProcessing::FgBgSegmentation::Background;

		auto computed = matcher.computeDispAndForegroundMask(onDemandCv);
		Multidim::Array<disp_t, 2>const& raw_disp = computed.disp;
		Multidim::Array<StereoVision::Correlation::StereoDispWithBgMask::MaskInfo, 2>& m_info = computed.fg_mask;

		auto shape = raw_disp.shape();

		//remove sharp disparity edges to isolate small edge artifacts.
		if (exportDialog.maxDispDelta() > 0) {

			int maxDispDelta = exportDialog.maxDispDelta();

			for (int i = 1; i < shape[0]-1; i++) {
				for (int j = 1; j < shape[1]-1; j++) {
					if (std::fabs(raw_disp.valueUnchecked(i-1,j) - raw_disp.valueUnchecked(i,j)) > maxDispDelta or
							std::fabs(raw_disp.valueUnchecked(i+1,j) - raw_disp.valueUnchecked(i,j)) > maxDispDelta or
							std::fabs(raw_disp.valueUnchecked(i,j-1) - raw_disp.valueUnchecked(i,j)) > maxDispDelta or
							std::fabs(raw_disp.valueUnchecked(i,j+1) - raw_disp.valueUnchecked(i,j)) > maxDispDelta) {
						m_info.atUnchecked(i,j) = Background;
					}
				}
			}

		}

		Multidim::Array<uint8_t, 2> mask_info = m_info.cast<uint8_t>();

		int openDistance = exportDialog.openingDistance();
		int erodeDistance = exportDialog.erodingDistance();

		Multidim::Array<uint8_t, 2> filtered;
		const Multidim::Array<uint8_t, 2>* sMask;

		if (openDistance > 0 and erodeDistance > 0) {
			filtered = StereoVision::ImageProcessing::dilation(openDistance, openDistance,
													StereoVision::ImageProcessing::erosion(openDistance+erodeDistance, openDistance+erodeDistance, mask_info));
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

		if (exportDialog.exportImages()) {
			writePointCloudAsStevimgs(outDir.absolutePath(), fileName, PointsColor, refined_disp, *sMask);
		} else {
			saveXYZRGBpointcloud(filePath, Points, PointsColor, colorScale, correctgamma);
		}
	}

	qDebug() << functionName << "finished task";


}

void exportSegmentedColoredStereoImages(FixedColorStereoSequence* sequence, QVector<int> rows) {

	constexpr Multidim::AccessCheck Nc = Multidim::AccessCheck::Nocheck;

	QString functionName = "exportSegmentedColoredStereoImages: ";

	if (sequence == nullptr) {
		qDebug() << functionName << "missing sequence";
		return;
	}

	MainWindow* mw = MainWindow::getActiveMainWindow();

	if (mw == nullptr) {
		qDebug() << functionName << "missing main windows";
		return;
	}

	Project* p = sequence->getProject();

	StereoSequenceImageExportOptionDialog exportDialog(mw);

	int seqLeftBgId = sequence->leftViewId();
	int seqRightBgId = sequence->rightViewId();

	Image* seqImgLeft = p->getDataBlock<Image>(seqLeftBgId);
	Image* seqImgRight = p->getDataBlock<Image>(seqRightBgId);

	if (seqImgLeft != nullptr) {
		exportDialog.setLeftBackgroundImage(seqImgLeft->getImageFile());
	}

	if (seqImgRight != nullptr) {
		exportDialog.setRightBackgroundImage(seqImgRight->getImageFile());
	}

	exportDialog.setSearchRadius(5);
	exportDialog.setSearchWidth(150);

	exportDialog.setHiearchicalLevel(4);
	exportDialog.setTransitionCostWeight(2);

	exportDialog.setVisualWeight(3);
	exportDialog.setVisualPatchRadius(1);
	exportDialog.setVisualThreshold(2.0);

	exportDialog.setDepthWeight(5);
	exportDialog.setDepthThreshold(2);

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

	int nLevels = exportDialog.hiearchicalLevel();
	float globalSwitchCost = 1.5;
	int scaleStep = 2;

	int visualPatchRadius = 0;

	int erosion_rad = 2;
	int dilation_rad = 50;
	int extension_rad = 4;

	int histBins = 50;
	int cutOffHistBins = 15;


	ImagePair* stereoPair = ImagePair::getImagePairInProject(p, seqLeftBgId, seqRightBgId);

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
		qDebug() << functionName << "Cannot compute stereo rectifier";
		return;
	}

	int dispDelta = stereo_rectifier->dispDelta(); //new principal point cam2 x - new principal point cam1 x

	if (stereoPair->idImgCam1() == seqLeftBgId and stereoPair->idImgCam2() == seqRightBgId) {
		dispDelta = -dispDelta;
	}
	// now disp delta is garanteed to be = new principal point cam_left x - new principal point cam_right x


	QVector<FixedColorStereoSequence::ImagePair> pairs = sequence->imgsPairs();
	float gamma = 1;

	using SegmentationValue = StereoVision::ImageProcessing::FgBgSegmentation::MaskInfo;

	constexpr StereoVision::ImageProcessing::FgBgSegmentation::MaskInfo Foreground =  StereoVision::ImageProcessing::FgBgSegmentation::Foreground;
	constexpr StereoVision::ImageProcessing::FgBgSegmentation::MaskInfo Background =  StereoVision::ImageProcessing::FgBgSegmentation::Background;

	constexpr StereoVision::Correlation::matchingFunctions comparisonMatchFunc = StereoVision::Correlation::matchingFunctions::SAD;

	using T_F = float;
	using T_L = T_F;
	using T_R = T_F;
	using T_CV = float;

	using disp_t = StereoVision::Correlation::disp_t;

	constexpr StereoVision::Correlation::dispDirection dDir = StereoVision::Correlation::dispDirection::RightToLeft;

	int originalFormatBgLeft;
	StereoVision::ImageArray bgImLeft = getImageData(exportDialog.leftBackgroundImage(), gamma, &originalFormatBgLeft);
	int originalFormatBgRight;
	StereoVision::ImageArray bgImRight = getImageData(exportDialog.rightBackgroundImage(), gamma, &originalFormatBgRight);

	if (bgImLeft.empty() or bgImRight.empty()) {
		qDebug() << functionName << "could not load the background images";
		return;
	}



	StereoVision::ImageArray transformedBgCamLeft;
	if (stereoPair->idImgCam1() == seqLeftBgId) {
		transformedBgCamLeft = StereoVision::Interpolation::interpolateImage(bgImLeft, stereo_rectifier->backWardMapCam1());
	} else {
		transformedBgCamLeft = StereoVision::Interpolation::interpolateImage(bgImLeft, stereo_rectifier->backWardMapCam2());
	}

	StereoVision::ImageArray transformedBgCamRight;
	if (stereoPair->idImgCam1() == seqRightBgId) {
		transformedBgCamRight = StereoVision::Interpolation::interpolateImage(bgImRight, stereo_rectifier->backWardMapCam1());
	} else {
		transformedBgCamRight = StereoVision::Interpolation::interpolateImage(bgImRight, stereo_rectifier->backWardMapCam2());
	}


	auto visualFeaturePyramidLeftDispBg = StereoVision::Correlation::buildFeaturePyramid(transformedBgCamLeft, visualPatchRadius, visualPatchRadius, nLevels);
	auto visualFeaturePyramidRightDispBg = StereoVision::Correlation::buildFeaturePyramid(transformedBgCamRight, visualPatchRadius, visualPatchRadius, nLevels);

	for (int row : rows) {

		if (row < 0) {
			continue;
		}

		if (row >= pairs.size()) {
			continue;
		}

		FixedColorStereoSequence::ImagePair& pair = pairs[row];

		QString fileBaseName;
		QString filePath;

		QFileInfo leftImgFileInfos(pair.StereoLeftImgPath);
		QFileInfo rightImgFileInfos(pair.StereoRightImgPath);

		QString leftBasename = leftImgFileInfos.baseName();
		QString rightBasename = rightImgFileInfos.baseName();

		int maxSize = std::max(leftBasename.size(), rightBasename.size());
		int minSize = std::min(leftBasename.size(), rightBasename.size());

		fileBaseName.reserve(maxSize);

		for (int i = 0; i < maxSize; i++) {

			if (i >= minSize) {
				break;
			}

			if (leftBasename[i] != rightBasename[i]) {
				break;
			}

			fileBaseName += leftBasename[i];
		}


		int originalFormatLeft;
		StereoVision::ImageArray imLeft = getImageData(inDir.absoluteFilePath(pair.StereoLeftImgPath), gamma, &originalFormatLeft);
		int originalFormatRight;
		StereoVision::ImageArray imRight = getImageData(inDir.absoluteFilePath(pair.StereoRightImgPath), gamma, &originalFormatRight);

		if (imLeft.empty() or imRight.empty()) {
			continue;
		}

		StereoVision::ImageArray transformedCamLeft;
		if (stereoPair->idImgCam1() == seqLeftBgId) {
			transformedCamLeft = StereoVision::Interpolation::interpolateImage(imLeft, stereo_rectifier->backWardMapCam1());
		} else {
			transformedCamLeft = StereoVision::Interpolation::interpolateImage(imLeft, stereo_rectifier->backWardMapCam2());
		}

		StereoVision::ImageArray transformedCamRight;
		if (stereoPair->idImgCam1() == seqRightBgId) {
			transformedCamRight = StereoVision::Interpolation::interpolateImage(imRight, stereo_rectifier->backWardMapCam1());
		} else {
			transformedCamRight = StereoVision::Interpolation::interpolateImage(imRight, stereo_rectifier->backWardMapCam2());
		}


		auto visualFeaturePyramidLeftDisp = StereoVision::Correlation::buildFeaturePyramid(transformedCamLeft, visualPatchRadius, visualPatchRadius, nLevels);
		auto visualFeaturePyramidRightDisp = StereoVision::Correlation::buildFeaturePyramid(transformedCamRight, visualPatchRadius, visualPatchRadius, nLevels);

		std::vector<Multidim::Array<T_CV, 3>> seg_costs_L(nLevels);
		std::vector<Multidim::Array<T_CV, 3>> seg_costs_R(nLevels);

		std::vector<StereoVision::ImageProcessing::MaskCostPolicy<T_CV> const*> costs_policies_L(nLevels);
		std::vector<StereoVision::ImageProcessing::MaskCostPolicy<T_CV> const*> costs_policies_R(nLevels);

		std::vector<Multidim::Array<T_CV,2>> logCompCostsL(nLevels);
		std::vector<Multidim::Array<T_CV,2>> logCompCostsR(nLevels);

		for (int l = 0; l < nLevels; l++) {

			std::array<int,3> shapeL = visualFeaturePyramidLeftDisp[l].shape();
			std::array<int,3> shapeR = visualFeaturePyramidRightDisp[l].shape();

			Multidim::Array<T_CV,3> compCostL =
					StereoVision::Correlation::featureVolume2CostVolume<comparisonMatchFunc,T_L,T_R,disp_t,dDir,T_CV>
					(visualFeaturePyramidLeftDisp[l], visualFeaturePyramidLeftDispBg[l], 1);

			Multidim::Array<T_CV,3> compCostR =
					StereoVision::Correlation::featureVolume2CostVolume<comparisonMatchFunc,T_L,T_R,disp_t,dDir,T_CV>
					(visualFeaturePyramidRightDisp[l], visualFeaturePyramidRightDispBg[l], 1);

			logCompCostsL[l] = Multidim::Array<T_CV,2>(shapeL[0], shapeL[1]);
			logCompCostsR[l] = Multidim::Array<T_CV,2>(shapeR[0], shapeR[1]);

			T_CV maxLogCostL = 0.0;
			T_CV maxLogCostR = 0.0;

			for (int i = 0; i < shapeL[0]; i++) {
				for (int j = 0; j < shapeL[1]; j++) {

					T_CV diffL = compCostL.valueUnchecked(i,j,0);

					T_CV logDiffL = std::log(1.0 + diffL);

					if (logDiffL > maxLogCostL) {
						maxLogCostL = logDiffL;
					}

					logCompCostsL[l].atUnchecked(i,j) = logDiffL;
				}
			}

			for (int i = 0; i < shapeR[0]; i++) {
				for (int j = 0; j < shapeR[1]; j++) {

					T_CV diffR = compCostR.valueUnchecked(i,j,0);

					T_CV logDiffR = std::log(1.0 + diffR);

					if (logDiffR > maxLogCostR) {
						maxLogCostR = logDiffR;
					}

					logCompCostsR[l].atUnchecked(i,j) = logDiffR;
				}
			}

		}

		StereoVision::ImageProcessing::Histogram<T_CV> costHistogramL =
				StereoVision::ImageProcessing::Histogram<T_CV>::getHistogramWithNBins(logCompCostsL.back(), histBins);
		StereoVision::ImageProcessing::Histogram<T_CV> costHistogramR =
				StereoVision::ImageProcessing::Histogram<T_CV>::getHistogramWithNBins(logCompCostsR.back(), histBins);

		std::optional<T_CV> autoThresholdL = costHistogramL.getBinLowerBound(cutOffHistBins);
		std::optional<T_CV> autoThresholdR = costHistogramR.getBinLowerBound(cutOffHistBins);
		//std::optional<T_CV> otsuThresholdL = StereoVision::ImageProcessing::computeBalancedHistogramThreshold(costHistogramL);
		//std::optional<T_CV> otsuThresholdR = StereoVision::ImageProcessing::computeBalancedHistogramThreshold(costHistogramR);

		if (!autoThresholdL.has_value() or !autoThresholdR.has_value()) {

			for (int i = 0; i < nLevels; i++) {
				delete costs_policies_L[i];
				delete costs_policies_R[i];
			}

			qDebug() << functionName << "otsu threshold could not be computed";

			continue;
		}

		for (int l = 0; l < nLevels; l++) {

			std::array<int,3> shapeL = visualFeaturePyramidLeftDisp[l].shape();
			std::array<int,3> shapeR = visualFeaturePyramidRightDisp[l].shape();

			Multidim::Array<T_CV, 3> seg_cost_L(shapeL[0], shapeL[1], 2);
			Multidim::Array<T_CV, 3> seg_cost_R(shapeR[0], shapeR[1], 2);

			for (int i = 0; i < shapeL[0]; i++) {
				for (int j = 0; j < shapeL[1]; j++) {

					T_CV FgCost = std::max(autoThresholdL.value() - logCompCostsL[l].valueUnchecked(i,j), T_CV(0));
					T_CV BgCost = std::max(logCompCostsL[l].valueUnchecked(i,j) - autoThresholdL.value(), T_CV(0));

					seg_cost_L.atUnchecked(i,j,Foreground) = FgCost;
					seg_cost_L.atUnchecked(i,j,Background) = BgCost;
				}
			}

			for (int i = 0; i < shapeR[0]; i++) {
				for (int j = 0; j < shapeR[1]; j++) {

					T_CV FgCost = std::max(autoThresholdR.value() - logCompCostsR[l].valueUnchecked(i,j), T_CV(0));
					T_CV BgCost = std::max(logCompCostsR[l].valueUnchecked(i,j) - autoThresholdR.value(), T_CV(0));

					seg_cost_R.atUnchecked(i,j,Foreground) = FgCost;
					seg_cost_R.atUnchecked(i,j,Background) = BgCost;
				}
			}

			seg_costs_L[l] = std::move(seg_cost_L);
			seg_costs_R[l] = std::move(seg_cost_R);

			costs_policies_L[l] = new StereoVision::ImageProcessing::GuidedMaskCostPolicy<T_CV, T_L>(globalSwitchCost, visualFeaturePyramidLeftDisp[l]);
			costs_policies_R[l] = new StereoVision::ImageProcessing::GuidedMaskCostPolicy<T_CV, T_R>(globalSwitchCost, visualFeaturePyramidRightDisp[l]);

		}

		auto smallShapeL = logCompCostsL.back().shape();
		auto smallShapeR = logCompCostsR.back().shape();

		Multidim::Array<SegmentationValue, 2> initialMaskL(smallShapeL[0], smallShapeL[1]);
		Multidim::Array<SegmentationValue, 2> initialMaskR(smallShapeR[0], smallShapeR[1]);

		for (int i = 0; i < smallShapeL[0]; i++) {
			for (int j = 0; j < smallShapeL[1]; j++) {

				initialMaskL.atUnchecked(i,j) = (logCompCostsL.back().valueUnchecked(i,j) > autoThresholdL.value()) ? Foreground : Background;
			}
		}

		for (int i = 0; i < smallShapeR[0]; i++) {
			for (int j = 0; j < smallShapeR[1]; j++) {

				initialMaskR.atUnchecked(i,j) = (logCompCostsR.back().valueUnchecked(i,j) > autoThresholdR.value()) ? Foreground : Background;
			}
		}


		QString costPathL = outDir.absoluteFilePath(fileBaseName + "nCostL.png");

		T_CV max = costHistogramL.getBinLowerBound(histBins);
		Multidim::Array<float, 2> nCost(smallShapeL);

		for (int i = 0; i < smallShapeL[0]; i++) {
			for (int j = 0; j < smallShapeL[1]; j++) {
				nCost.atUnchecked(i,j) = logCompCostsL.back().valueUnchecked(i,j)/max;
			}
		}

		saveImageData(costPathL, nCost, gamma);


		QString initialMaskPathL = outDir.absoluteFilePath(fileBaseName + "initial_mask_l.png");
		QString initialMaskPathR = outDir.absoluteFilePath(fileBaseName + "initial_mask_r.png");

		saveImageData(initialMaskPathL, initialMaskL.cast<float>(), gamma);
		saveImageData(initialMaskPathR, initialMaskR.cast<float>(), gamma);

		int div = 1;
		for (int l = 1; l < nLevels; l++) {
			div *= scaleStep;
		}

		int eRad = erosion_rad/div;
		int dRad = dilation_rad/div;

		StereoVision::ImageProcessing::StructuralElement erodeElement1 = StereoVision::ImageProcessing::buildCircularStructuralElement(eRad);
		StereoVision::ImageProcessing::StructuralElement dilateElement = StereoVision::ImageProcessing::buildCircularStructuralElement(dRad + eRad);
		StereoVision::ImageProcessing::StructuralElement erodeElement2 = StereoVision::ImageProcessing::buildCircularStructuralElement(dRad + eRad);

		QString erodedPathL = outDir.absoluteFilePath(fileBaseName + "eroded_mask.png");
		QString dilatedPathL = outDir.absoluteFilePath(fileBaseName + "dilated_mask.png");

		Multidim::Array<SegmentationValue, 2> post_processed_smallMaskL =
				StereoVision::ImageProcessing::erosion(erodeElement1, initialMaskL);


		saveImageData(erodedPathL, post_processed_smallMaskL.cast<float>(), gamma);

		post_processed_smallMaskL = StereoVision::ImageProcessing::dilation(dilateElement, post_processed_smallMaskL);

		saveImageData(dilatedPathL, post_processed_smallMaskL.cast<float>(), gamma);

		post_processed_smallMaskL = StereoVision::ImageProcessing::erosion(erodeElement2, post_processed_smallMaskL);

		Multidim::Array<SegmentationValue, 2> post_processed_smallMaskR =
				StereoVision::ImageProcessing::erosion(erodeElement1, initialMaskR);

		post_processed_smallMaskR = StereoVision::ImageProcessing::dilation(dilateElement, post_processed_smallMaskR);
		post_processed_smallMaskR = StereoVision::ImageProcessing::erosion(erodeElement2, post_processed_smallMaskR);

		Multidim::Array<SegmentationValue, 2> maskL = post_processed_smallMaskL;
		Multidim::Array<SegmentationValue, 2> maskR = post_processed_smallMaskR;


		QString smallMaskPathL = outDir.absoluteFilePath(fileBaseName + "small_mask_l.png");
		QString smallMaskPathR = outDir.absoluteFilePath(fileBaseName + "small_mask_r.png");

		saveImageData(smallMaskPathL, maskL.cast<float>(), gamma);
		saveImageData(smallMaskPathR, maskR.cast<float>(), gamma);

		for (int l = 0; l < nLevels; l++) {
			int level = nLevels-l-1;

			std::array<int,3> shapeL = visualFeaturePyramidLeftDisp[level].shape();
			std::array<int,3> shapeR = visualFeaturePyramidRightDisp[level].shape();

			Multidim::Array<bool, 2> erodedL = StereoVision::ImageProcessing::erosion<SegmentationValue, bool>(extension_rad, extension_rad, maskL);
			Multidim::Array<bool, 2> dilatedL = StereoVision::ImageProcessing::dilation<SegmentationValue, bool>(extension_rad, extension_rad, maskL);

			Multidim::Array<bool, 2> erodedR = StereoVision::ImageProcessing::erosion<SegmentationValue, bool>(extension_rad, extension_rad, maskR);
			Multidim::Array<bool, 2> dilatedR = StereoVision::ImageProcessing::dilation<SegmentationValue, bool>(extension_rad, extension_rad, maskR);

			Multidim::Array<bool, 2> optimizablesL(shapeL[0], shapeL[1]);
			Multidim::Array<bool, 2> optimizablesR(shapeR[0], shapeR[1]);

			for (int i = 0; i < shapeL[0]; i++) {
				for (int j = 0; j < shapeL[1]; j++) {

					optimizablesL.atUnchecked(i,j) = erodedL.valueUnchecked(i,j) != dilatedL.valueUnchecked(i,j);
				}
			}

			for (int i = 0; i < shapeR[0]; i++) {
				for (int j = 0; j < shapeR[1]; j++) {

					optimizablesR.atUnchecked(i,j) = erodedR.valueUnchecked(i,j) != dilatedR.valueUnchecked(i,j);
				}
			}

			maskL = StereoVision::ImageProcessing::getPartialGlobalRefinedMask(seg_costs_L[level], *costs_policies_L[level], optimizablesL, maskL);
			maskR = StereoVision::ImageProcessing::getPartialGlobalRefinedMask(seg_costs_R[level], *costs_policies_R[level], optimizablesR, maskR);

			QString lvlMaskPathL = outDir.absoluteFilePath(fileBaseName + QString("mask_lvl%1_l.png").arg(level));
			QString lvlMaskPathR = outDir.absoluteFilePath(fileBaseName + QString("mask_lvl%1_r.png").arg(level));

			saveImageData(lvlMaskPathL, maskL.cast<float>(), gamma);
			saveImageData(lvlMaskPathR, maskR.cast<float>(), gamma);

			if (l != nLevels-1) {

				std::array<int,3> nextShapeL = visualFeaturePyramidLeftDisp[level-1].shape();
				std::array<int,3> nextShapeR = visualFeaturePyramidRightDisp[level-1].shape();

				maskL = StereoVision::ImageProcessing::FgBgSegmentation::upscaleMask(maskL, {nextShapeL[0], nextShapeL[1]}).upscaled_mask;
				maskR = StereoVision::ImageProcessing::FgBgSegmentation::upscaleMask(maskR, {nextShapeR[0], nextShapeR[1]}).upscaled_mask;
			}
		}

		QString leftPath = outDir.absoluteFilePath(fileBaseName + "rectified_left.png");
		QString rightPath = outDir.absoluteFilePath(fileBaseName + "rectified_right.png");

		QString leftMaskPath = outDir.absoluteFilePath(fileBaseName + "rectified_left_mask.png");
		QString rightMaskPath = outDir.absoluteFilePath(fileBaseName + "rectified_right_mask.png");

		saveImageData(leftPath, transformedCamLeft, gamma);
		saveImageData(rightPath, transformedCamRight, gamma);

		ImageArray maskLImg(maskL.shape()[0], maskL.shape()[1], 1);
		ImageArray maskRImg(maskR.shape()[0], maskR.shape()[1], 1);

		for (int i = 0; i < maskL.shape()[0]; i++) {
			for (int j = 0; j < maskL.shape()[1]; j++) {
				maskLImg.atUnchecked(i,j,0) = (maskL.valueUnchecked(i,j) == Foreground) ? 1.0 : 0.0;
			}
		}

		for (int i = 0; i < maskR.shape()[0]; i++) {
			for (int j = 0; j < maskR.shape()[1]; j++) {
				maskRImg.atUnchecked(i,j,0) = (maskR.valueUnchecked(i,j) == Foreground) ? 1.0 : 0.0;
			}
		}

		saveImageData(leftMaskPath, maskLImg, gamma);
		saveImageData(rightMaskPath, maskRImg, gamma);

		for (int i = 0; i < nLevels; i++) {
			delete costs_policies_L[i];
			delete costs_policies_R[i];
		}

		//end row loop
	}
}

} //namespace StereoVisionApp
