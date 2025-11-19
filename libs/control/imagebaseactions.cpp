#include "imagebaseactions.h"

#include "utils_functions.h"

#include "application.h"

#include "datablocks/project.h"
#include "datablocks/image.h"
#include "datablocks/camera.h"
#include "datablocks/stereorig.h"
#include "datablocks/cameracalibration.h"
#include "datablocks/angleconstrain.h"
#include "datablocks/correspondencesset.h"
#include "datablocks/localcoordinatesystem.h"

#include <StereoVision/interpolation/interpolation.h>
#include <StereoVision/interpolation/lensdistortionsmap.h>
#include <StereoVision/geometry/stereorigrectifier.h>
#include <StereoVision/geometry/lensdistortion.h>
#include <StereoVision/geometry/imagecoordinates.h>
#include <StereoVision/geometry/alignement.h>

#include "sparsesolver/vertices/camerapose.h"

#include <StereoVision/imageProcessing/hexagonalRGBTargetsDetection.h>

#include <StereoVision/io/image_io.h>

#include "vision/imageio.h"
#include "vision/sparseMatchingPipeline.h"

#include "sparsesolver/helperfunctions.h"

#include "mainwindow.h"
#include "gui/rectifiedimageexportoptionsdialog.h"
#include "gui/hexagonaltargetdetectionoptionsdialog.h"

#include <QFileInfo>
#include <QDir>
#include <QPixmap>
#include <QRegularExpression>
#include <QFileDialog>
#include <QStandardPaths>

#include <QDebug>

#include <exiv2/exiv2.hpp>

namespace StereoVisionApp {

qint64 addPlaceholderImage(QString name, Project* p, Camera *cam) {

    QString cn = ImageFactory::imageClassName();

    if (cam == nullptr) {
        return -1;
    }

    if (cam->getProject() != p) {
        return -1;
    }

    qint64 id = p->createDataBlock(cn.toStdString().c_str());

    if (id >= 0) {
        Image* im = qobject_cast<Image*>(p->getById(id));

        im->setObjectName(name);

        im->assignCamera(cam->internalId());
    }

    return id;
}

qint64 addImage(QString f, Project* p, Camera* cam) {

	QString cn = ImageFactory::imageClassName();

	Exiv2::Image::AutoPtr image = Exiv2::ImageFactory::open(f.toStdString()); //TODO: check if Exiv2 can be updated.
	QPixmap pixmap(f);

	if(image.get() == 0 or pixmap.isNull()) {
		return -1;
	}

	qint64 id = p->createDataBlock(cn.toStdString().c_str());

	if (id >= 0) {
		Image* im = qobject_cast<Image*>(p->getById(id));
		im->setImageFile(f);

		QFileInfo inf(f);
		im->setObjectName(inf.baseName());

		Camera* imgCam = cam;

		bool needCam = imgCam == nullptr;

		if (imgCam != nullptr) {
			needCam = imgCam->getProject() == p;

			if ( imgCam->getProject() != p) {
				imgCam = nullptr;
			}
		}

		QString camName;

		image->readMetadata();
		Exiv2::ExifData &exifData = image->exifData();

		if (needCam) {

			auto camTag = exifData.findKey(Exiv2::ExifKey("Exif.Image.Model"));
			if (camTag != exifData.end()) {
				camName = QString::fromStdString(camTag->toString());
			}

			if (camName.isEmpty()) {
				camName = QString("default%1x%2").arg(pixmap.width()).arg(pixmap.height());
			}

			QVector<qint64> cam_ids = p->getIdsByClass(CameraFactory::cameraClassName());

			for (qint64 cam_id : cam_ids) {
				Camera* c = qobject_cast<Camera*>(p->getById(cam_id));

				if (c->objectName() == camName) {
					imgCam = c;
					break;
				}
			}
		}

		if (imgCam != nullptr) {
			im->assignCamera(imgCam->internalId());
		} else {

			qint64 cam_id = p->createDataBlock(CameraFactory::cameraClassName().toStdString().c_str());

			if (cam_id >= 0) {

				imgCam = qobject_cast<Camera*>(p->getById(cam_id));

				imgCam->setObjectName(camName);
				imgCam->setImSize(pixmap.size());
				imgCam->setOpticalCenterX(floatParameter(pixmap.width()/2.));
				imgCam->setOpticalCenterY(floatParameter(pixmap.height()/2.));

				im->assignCamera(cam_id);

				floatParameter fLen(pixmap.width(), false);
				floatParameter pixelAspectRatio(1.0, false);

				auto fLenTag = exifData.findKey(Exiv2::ExifKey("Exif.Image.FocalLength"));
				auto fLenPhotoTag = exifData.findKey(Exiv2::ExifKey("Exif.Photo.FocalLength"));
				auto fLenPhoto35mmTag = exifData.findKey(Exiv2::ExifKey("Exif.Photo.FocalLengthIn35mmFilm"));
				auto fPlanXRes = exifData.findKey(Exiv2::ExifKey("Exif.Image.FocalPlaneXResolution"));
				auto fPlanYRes = exifData.findKey(Exiv2::ExifKey("Exif.Image.FocalPlaneYResolution"));
				auto fPlanUnit = exifData.findKey(Exiv2::ExifKey("Exif.Image.FocalPlaneResolutionUnit"));

				if ((fLenTag != exifData.end() or fLenPhotoTag != exifData.end() or fLenPhoto35mmTag != exifData.end()) and
						(fLenPhoto35mmTag != exifData.end() or
						 ((fPlanXRes != exifData.end() or fPlanYRes != exifData.end()) and fPlanUnit != exifData.end()))) {

					Exiv2::Rational fPlanRes;
					Exiv2::Rational pixAspectRatio = {1, 1};

					if (fPlanXRes != exifData.end() or fPlanYRes != exifData.end()) {

						if (fPlanXRes != exifData.end()) {
							fPlanRes = fPlanXRes->toRational();
						} else {
							fPlanRes = fPlanYRes->toRational();
						}

						if (fPlanXRes != exifData.end() and fPlanYRes != exifData.end()) {
							Exiv2::Rational tmp = fPlanYRes->toRational();
							pixAspectRatio = {fPlanRes.first*tmp.second, fPlanRes.second*tmp.first};
						}

						if (pixAspectRatio.first == pixAspectRatio.second) {
							pixelAspectRatio.setIsSet(1.0);
						} else {
							pixelAspectRatio.setIsSet(static_cast<pFloatType>(pixAspectRatio.first)/static_cast<pFloatType>(pixAspectRatio.second));
						}

					}

					Exiv2::Rational fLenRatio;
					if (fLenPhoto35mmTag != exifData.end()) {
						fLenRatio = fLenPhoto35mmTag->toRational();
						fLen.setIsSet(static_cast<pFloatType>(fLenRatio.first)*static_cast<pFloatType>(pixmap.width())/
									  (static_cast<pFloatType>(fLenRatio.second)*35.));
					} else {

						if (fLenTag != exifData.end()) {
							fLenRatio = fLenTag->toRational();
						} else {
							fLenRatio = fLenPhotoTag->toRational();
						}

						bool hasConversionUnit = true;
						uint16_t unit = fPlanUnit->toLong();

						float fPlanUnit2mm = 1.0;

						if (unit == 2) {//inch
							fPlanUnit2mm = 25.4;
						} else if (unit == 3) {//cm
							fPlanUnit2mm = 10.;
						} else if (unit == 4) {//mm
							fPlanUnit2mm = 1.;
						} else {
							hasConversionUnit = false;
						}

						if (hasConversionUnit) {

							pFloatType c_flen = pFloatType(fLenRatio.first) / pFloatType(fLenRatio.second)
									* pFloatType(fPlanRes.first) / pFloatType(fPlanRes.second) / fPlanUnit2mm;

							fLen.setIsSet(c_flen);

						}

					}

				}

				imgCam->setFLen(fLen);
				imgCam->setPixelRatio(pixelAspectRatio);

			}

		}

	}

	return id;

}

QStringList addImages(QStringList images, Project* p) {

	QStringList failed;

	for (QString const& f : images) {

		qint64 id = addImage(f, p);

		if (id < 0) {
			failed << f;
		}

	}

	return failed;
}

int exportRectifiedImages(QList<qint64> imagesIds, Project* p, bool useOptimizedVals, QString outputDirectory, float gamma) {

	if (imagesIds.size() == 0) {
		return 0;
	}

	int treated = 0;

	for (qint64 id : imagesIds) {

		Image* img = p->getDataBlock<Image>(id);

		if (img != nullptr) {

			Camera* cam = p->getDataBlock<Camera>(img->assignedCamera());

			if (cam != nullptr) {

				int originalFormat;
				StereoVision::ImageArray array = getImageData(img->getImageFile(), gamma, &originalFormat);

				if (!array.empty()) {
					int height = array.shape()[0];
					int width = array.shape()[1];

					float f = (useOptimizedVals) ? cam->optimizedFLen().value() : cam->fLen().value();

					Eigen::Vector2f pp;
					Eigen::Vector3f k123;
					Eigen::Vector2f t12;
					Eigen::Vector2f B12;

					if (useOptimizedVals) {
						pp << cam->optimizedOpticalCenterX().value(), cam->optimizedOpticalCenterY().value();
						k123 << cam->optimizedK1().value(), cam->optimizedK2().value(), cam->optimizedK3().value();
						t12 << cam->optimizedP1().value(), cam->optimizedP2().value();
						B12 << cam->optimizedB1().value(), cam->optimizedB2().value();
					} else {
						pp << cam->opticalCenterX().value(), cam->opticalCenterY().value();
						k123 << cam->k1().value(), cam->k2().value(), cam->k3().value();
						t12 << cam->p1().value(), cam->p2().value();
						B12 << cam->B1().value(), cam->B2().value();
					}

					StereoVision::ImageArray distMap = StereoVision::Interpolation::computeLensDistortionMap(height,
																  width,
																  f,
																  pp,
																  k123,
																  t12,
																  B12);

					StereoVision::ImageArray transformed = StereoVision::Interpolation::interpolateImage(array, distMap);

					QFileInfo infos(img->getImageFile());
					QString outFile = (outputDirectory.isEmpty()) ? infos.dir().absolutePath() : outputDirectory;
					outFile = QDir::cleanPath(outFile);

					if (!outFile.endsWith('/')) {
						outFile += "/";
					}

					outFile += infos.baseName() + "_rectified." + infos.completeSuffix();

					if (saveImageData(outFile, transformed, gamma, originalFormat)) {
						treated++;
					}
				}

			}

		}

	}

	return treated;

}

int exportRectifiedImages(QList<qint64> imagesIds, Project* p, QWidget* w) {

	if (imagesIds.size() == 0) {
		return 0;
	}

	RectifiedImageExportOptionsDialog d(w);
	d.setModal(true);
	d.setWindowTitle(imagesIds.size() > 1 ? QObject::tr("Export rectified images") : QObject::tr("Export rectified image"));

	d.exec();

	if (d.result() == QDialog::Rejected) {
		return 0;
	}

	QString dir = d.selectedFolder();
	bool useOptimizedParametersSet = d.useOptimizedCameraParameters();
	float gamma = d.gamma();

	return exportRectifiedImages(imagesIds, p, useOptimizedParametersSet, dir, gamma);

}

int exportStereoRigRectifiedImages(QList<qint64> imagesIds, qint64 rigId, Project* p, QWidget* w) {

	if (imagesIds.size() != 2) {
		return 0;
	}

	StereoRig* rig = p->getDataBlock<StereoRig>(rigId);

	if (rig == nullptr) {
		return 0;
	}

	ImagePair* pair = nullptr;

	for (qint64 id : rig->listTypedSubDataBlocks(ImagePair::staticMetaObject.className())) {
		ImagePair* p = qobject_cast<ImagePair*>(rig->getById(id));

		if (p != nullptr) {

			if ((p->idImgCam1() == imagesIds[0] and p->idImgCam2() == imagesIds[1]) or
					(p->idImgCam1() == imagesIds[1] and p->idImgCam2() == imagesIds[0])) {
				pair = p;
				break;
			}
		}
	}

	if (pair == nullptr) {
		return 0;
	}

	RectifiedImageExportOptionsDialog d(w);
	d.setModal(true);
	d.setWindowTitle(imagesIds.size() > 1 ? QObject::tr("Export rectified images") : QObject::tr("Export rectified image"));

	d.exec();

	if (d.result() == QDialog::Rejected) {
		return 0;
	}

	QString dir = d.selectedFolder();
	bool useOptimizedParametersSet = d.useOptimizedCameraParameters();
	float gamma = d.gamma();

	Image* image1 = p->getDataBlock<Image>(pair->idImgCam1());
	Image* image2 = p->getDataBlock<Image>(pair->idImgCam2());

	auto rectifier = configureRectifierForStereoPair(pair, useOptimizedParametersSet);

	if (rectifier == nullptr) {
		return 0;
	}

	bool ok = rectifier->compute(StereoVision::Geometry::StereoRigRectifier::TargetRangeSetMethod::Same,
								 StereoVision::Geometry::StereoRigRectifier::TargetRangeSetMethod::Same);

	if (!ok) {
		return 0;
	}

	int originalFormat1;
	StereoVision::ImageArray arrayIm1 = getImageData(image1->getImageFile(), gamma, &originalFormat1);
	int originalFormat2;
	StereoVision::ImageArray arrayIm2 = getImageData(image2->getImageFile(), gamma, &originalFormat2);

	StereoVision::ImageArray transformedCam1 = StereoVision::Interpolation::interpolateImage(arrayIm1, rectifier->backWardMapCam1());
	StereoVision::ImageArray transformedCam2 = StereoVision::Interpolation::interpolateImage(arrayIm2, rectifier->backWardMapCam2());

	int treated = 0;

	QFileInfo infosCam1(image1->getImageFile());
	QString outFileCam1 = (dir.isEmpty()) ? infosCam1.dir().absolutePath() : dir;
	outFileCam1 = QDir::cleanPath(outFileCam1);

	if (!outFileCam1.endsWith('/')) {
		outFileCam1 += "/";
	}

	outFileCam1 += infosCam1.baseName() + "_stereorectified_rig" + QString("%1").arg(rig->internalId()) + "_." + infosCam1.completeSuffix();

	if (saveImageData(outFileCam1, transformedCam1, gamma, originalFormat1)) {
		treated++;
	}

	QFileInfo infosCam2(image2->getImageFile());
	QString outFileCam2 = (dir.isEmpty()) ? infosCam2.dir().absolutePath() : dir;
	outFileCam2 = QDir::cleanPath(outFileCam2);

	if (!outFileCam2.endsWith('/')) {
		outFileCam2 += "/";
	}

	outFileCam2 += infosCam2.baseName() + "_stereorectified_rig" + QString("%1").arg(rig->internalId()) + "_." + infosCam2.completeSuffix();

	if (saveImageData(outFileCam2, transformedCam2, gamma, originalFormat2)) {
		treated++;
	}

	if (treated > 0) {
		float dispDelta = rectifier->dispDelta();
		float normalizedBasline = rectifier->normalizedBasline();
		float rectifiedFLen = rectifier->reprojectionFLen();

		StereoVision::Geometry::AffineTransform c1_2_w = getImageToWorldTransform(image1).value().toAffineTransform();
		StereoVision::Geometry::AffineTransform c2_2_w = getImageToWorldTransform(image2).value().toAffineTransform();

		Eigen::Matrix3f RCam1 = c1_2_w.R*rectifier->CorrRCam1();
		Eigen::Matrix3f RCam2 = c2_2_w.R*rectifier->CorrRCam2();

		Eigen::Vector3f tCam1 = c1_2_w.t;
		Eigen::Vector3f tCam2 = c2_2_w.t;

		Eigen::Vector2f ppCam1 = rectifier->newPrincipalPointCam1();
		Eigen::Vector2f ppCam2 = rectifier->newPrincipalPointCam2();

		QString outFileInfos = (dir.isEmpty()) ? infosCam2.dir().absolutePath() : dir;
		outFileInfos = QDir::cleanPath(outFileInfos);

		if (!outFileInfos.endsWith('/')) {
			outFileInfos += "/";
		}

		outFileInfos += infosCam1.baseName() + "_" + infosCam2.baseName() +
				"_stereorectified_rig" + QString("%1").arg(rig->internalId()) + "_infos.txt";

		QFile infoFile(outFileInfos);

		if (infoFile.open(QFile::WriteOnly)) {

			QTextStream infostream(&infoFile);
			infostream.setRealNumberPrecision(12);

			infostream << "Disparity delta [px]: " << dispDelta << Qt::endl;
			infostream << "Normalized baseline [px / length unit]: " << normalizedBasline << '\n' << Qt::endl;

			infostream << "rectified focal length [px]: " << rectifiedFLen << Qt::endl;
			infostream << "principal point cam 1 [px]: [" << ppCam1.x() << ", " << ppCam1.y() << ']' << Qt::endl;
			infostream << "principal point cam 2 [px]: [" << ppCam2.x() << ", " << ppCam2.y() << ']' << '\n' << Qt::endl;

			std::stringstream ss;

			ss << RCam1.eulerAngles(0,1,2)/M_PI*180.;
			infostream << "Rotation cam 1 [deg]: \n" << QString::fromStdString(ss.str()) << Qt::endl;

			ss.str(std::string());
			ss << tCam1;
			infostream << "translation cam 1 [length unit]: \n" << QString::fromStdString(ss.str()) << '\n' << Qt::endl;

			ss.str(std::string());
			ss << RCam2.eulerAngles(0,1,2)/M_PI*180.;
			infostream << "Rotation cam 2 [deg]: \n" << QString::fromStdString(ss.str()) << Qt::endl;

			ss.str(std::string());
			ss << tCam2;
			infostream << "translation cam 2 [length unit]: \n" << QString::fromStdString(ss.str()) << '\n' << Qt::endl;

			infoFile.close();
		}
	}

	return treated;

}



int exportImageLandmarksPositionsToCSV(qint64 imageId, Project* p, QWidget* w) {

    if (p == nullptr) {
        return 0;
    }

    Image* img = p->getDataBlock<Image>(imageId);

    if (img == nullptr) {
        return 0;
    }

    QString saveFile = QFileDialog::getSaveFileName(w,
                                 QObject::tr("Export landmarks to csv"),
                                 QStandardPaths::standardLocations(QStandardPaths::PicturesLocation).first());

    if (saveFile.isEmpty()) {
        return 0;
    }

    QFile outFile(saveFile);

    bool ok = outFile.open(QFile::WriteOnly);

    if (!ok) {
        return 0;
    }

    QTextStream out(&outFile);

    out << "Landmark," << "u," << "v" << "\n";

    QVector<qint64> imLmId = img->listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName);

    for (qint64 subid : imLmId) {
        ImageLandmark* imLm = qobject_cast<ImageLandmark*>(img->getById(subid));

        if (imLm == nullptr) {
            continue;
        }

        Landmark* lm = imLm->attachedLandmark();

        if (lm == nullptr) {
            continue;
        }

        out << lm->objectName() << "," << imLm->x().value() << "," << imLm->y().value() << "\n";

    }

    outFile.close();

    return 1;

}

int addImagesToCalibration(QList<qint64> imagesIds, qint64 calibId, Project* p) {

	if (p == nullptr) {
		return 0;
	}

	CameraCalibration* calib = p->getDataBlock<CameraCalibration>(calibId);

	if (calib == nullptr) {
		return 0;
	}

	int treated = 0;

	for (qint64 id : imagesIds) {

		Image* img = p->getDataBlock<Image>(id);

		if (img == nullptr) {
			continue;
		}

		calib->addImg(id);
		treated++;

	}

	return treated;

}

int detectHexagonalTargets(QList<qint64> imagesIds, Project* p) {

	if (p == nullptr) {
		return 0; //no processing without projects
	}

	MainWindow* mw = MainWindow::getActiveMainWindow();

	double minThreshold = 80;
	double diffThreshold = 30;

	int minArea = 10;
	int maxArea = 800;

	double minToMaxAxisRatioThreshold = 0.6;
	double hexRelMaxDiameter = 0.2;

	double hexFirRelMaxRes = 0.1;

	double redGain = 1.1;
	double greenGain = 1.1;
	double blueGain = 1.0;

	bool clearPrevious = false;

	bool useHexScale = false;
	float hexEdge = 86.474;

	if (mw != nullptr) {

		HexagonalTargetDetectionOptionsDialog od(mw);
		od.setModal(true);

		od.setMinThreshold(minThreshold);
		od.setDiffThreshold(diffThreshold);

		od.setMinArea(minArea);
		od.setMaxArea(maxArea);

		od.setMinToMaxAxisRatioThreshold(minToMaxAxisRatioThreshold);

		od.setHexagonMaxRelDiameter(hexRelMaxDiameter);

		od.setHexagonFitMaxRelativeThreshold(hexFirRelMaxRes);

		od.setRedGain(redGain);
		od.setGreenGain(greenGain);
		od.setBlueGain(blueGain);

		od.setReplaceOld(clearPrevious);

		od.setUseHexagoneScale(useHexScale);
		od.setHexagoneSide(hexEdge);

		int code = od.exec();

		if (code != QDialog::Accepted) {
			return 0;
		}

		minThreshold = od.minThreshold();
		diffThreshold = od.diffThreshold();

		maxArea = od.maxArea();
		minArea = od.minArea();

		minToMaxAxisRatioThreshold = od.minToMaxAxisRatioThreshold();
		hexRelMaxDiameter = od.hexagonMaxRelDiameter();
		hexFirRelMaxRes = od.hexagonFitMaxRelativeThreshold();

		redGain = od.redGain();
		greenGain = od.greenGain();
		blueGain = od.blueGain();

		clearPrevious = od.replaceOld();

		useHexScale = od.useHexagoneScale();
		hexEdge = od.hexagoneSide();

	}

	int nImgsProcessed = 0;

	for (qint64 id : imagesIds) {

		Image* image = p->getDataBlock<Image>(id);

		bool sucess = detectHexagonalTargets(image,
											 minThreshold,
											 diffThreshold,
											 minArea,
											 maxArea,
											 minToMaxAxisRatioThreshold,
											 hexRelMaxDiameter,
											 hexFirRelMaxRes,
											 redGain,
											 greenGain,
											 blueGain,
											 clearPrevious,
											 useHexScale,
											 hexEdge);

		nImgsProcessed += (sucess) ? 1 : 0;

	}

	return nImgsProcessed;

}


bool detectHexagonalTargets(Image* image,
							double minThreshold,
							double diffThreshold,
							int minArea,
							int maxArea,
							double minToMaxAxisRatioThreshold,
							double hexRelMaxDiameter,
							double hexFirRelMaxRes,
							double redGain,
							double greenGain,
							double blueGain,
							bool clearPrevious,
							bool useHexScale,
							float hexEdge) {

	Project* p = image->getProject();

	if (image == nullptr or p == nullptr) {
		return false;
	}

	Multidim::Array<uint8_t, 3> img = StereoVision::IO::readImage<uint8_t>(image->getImageFile().toStdString());

	if (img.empty()) {
		return false;
	}

	constexpr StereoVision::Color::RedGreenBlue MC = StereoVision::Color::Blue;
	constexpr StereoVision::Color::RedGreenBlue PC = StereoVision::Color::Green;
	constexpr StereoVision::Color::RedGreenBlue NC = StereoVision::Color::Red;

	std::vector<StereoVision::ImageProcessing::HexRgbTarget::HexTargetPosition> targets =
		StereoVision::ImageProcessing::HexRgbTarget::detectHexTargets<uint8_t, MC, PC, NC>
			(img,
			 minThreshold,
			 diffThreshold,
			 minArea,
			 maxArea,
			 minToMaxAxisRatioThreshold,
			 hexRelMaxDiameter,
			 redGain,
			 greenGain,
			 blueGain,
			 hexFirRelMaxRes);

	for (StereoVision::ImageProcessing::HexRgbTarget::HexTargetPosition & target : targets) {

		QString colorCode = "RRRRR";


		for (int i = 0; i < 5; i++) {
			if (target.dotsPositives[i]) {
				colorCode[i] = 'G';
			}
		}

		QString lmNameBase = QString("HexaTarget_%1").arg(colorCode);

		std::array<Landmark*, 6> landmarks;

		for (int i = 0; i < 6; i++) {
			QString lmName = lmNameBase + QString("_%1").arg(i+1);

			Landmark* target_lm = p->getDataBlockByName<Landmark>(lmName);

			if (target_lm == nullptr) { //create the landmark if it does not exist.
				qint64 id = p->createDataBlock(LandmarkFactory::landmarkClassName().toStdString().c_str());

				target_lm = p->getDataBlock<Landmark>(id);
				target_lm->setObjectName(lmName);
			}

			landmarks[i] = target_lm;

			Eigen::Vector2f pos;

			if (i == 0) {
				pos = target.posRefDot;
			} else {
				pos = target.dotsPositions[i-1];
			}

			ImageLandmark* im_lm = image->getImageLandmarkByLandmarkId(target_lm->internalId());

			if (im_lm == nullptr) {

				qint64 imlm_id = image->addImageLandmark(QPointF(pos[1]+0.5, pos[0]+0.5), target_lm->internalId(), true, 3.0);
				im_lm = image->getImageLandmark(imlm_id);

			} else if (!clearPrevious) {
				continue;
			}

			im_lm->setX(pos[1]+0.5);
			im_lm->setY(pos[0]+0.5);
		}

		if (useHexScale) {
			/*const QString hexa_side_constraint_name("HexaTargets_HexagoneDistance");

			DistanceConstrain* hexa_dist = p->getDataBlockByName<DistanceConstrain>(hexa_side_constraint_name);

			if (hexa_dist == nullptr) {
				qint64 id = p->createDataBlock(DistanceConstrain::staticMetaObject.className());

				hexa_dist = p->getDataBlock<DistanceConstrain>(id);
				hexa_dist->setObjectName(hexa_side_constraint_name);

				hexa_dist->setDistanceValue(hexEdge);
			}

			for (int i = 0; i < 6; i++) {

				Landmark* lm0 = landmarks[i];
				Landmark* lm1 = landmarks[(i+1)%6];

				hexa_dist->insertLandmarksPair(lm0->internalId(), lm1->internalId()); //set constrain if not already existing

			}*/

			QString lcsName = QString("HexaTarget_%1_CoordinateSystem").arg(colorCode);

			LocalCoordinateSystem* lcs = p->getDataBlockByName<LocalCoordinateSystem>(lcsName);

			if (lcs == nullptr) {
				qint64 id = p->createDataBlock(LocalCoordinateSystem::staticMetaObject.className());

				lcs = p->getDataBlock<LocalCoordinateSystem>(id);
				lcs->setObjectName(lcsName);
			}

			for (int i = 0; i < 6; i++) {

				Landmark* lm = landmarks[i];

				float x = hexEdge*std::sin(i*60./180.*M_PI);
				float y = hexEdge*std::cos(i*60./180.*M_PI);
				float z = 0;

				lcs->addLandmarkLocalCoordinates(lm->internalId(), x, y, z); //set constrain if not already existing

			}

		} else {

			const QString hexa_angle_constraint_name("HexaTargets_HexagoneAngle");

			AngleConstrain* hexa_angle = p->getDataBlockByName<AngleConstrain>(hexa_angle_constraint_name);

			if (hexa_angle == nullptr) {
				qint64 id = p->createDataBlock(AngleConstrain::staticMetaObject.className());

				hexa_angle = p->getDataBlock<AngleConstrain>(id);
				hexa_angle->setObjectName(hexa_angle_constraint_name);

				hexa_angle->setAngleValue(120);
			}

			for (int i = 0; i < 6; i++) {

				Landmark* lm0 = landmarks[i];
				Landmark* lm1 = landmarks[(i+1)%6];
				Landmark* lm2 = landmarks[(i+2)%6];

				hexa_angle->insertLandmarksTriplet(lm0->internalId(), lm1->internalId(), lm2->internalId()); //set constrain if not already existing

			}

		}

	}

	return true;
}


int orientHexagonalTargetsRelativeToCamera(qint64 imgId, Project* p) {

	QRegularExpression hexNamePattern("^(HexaTarget_[RG]{5})_[1-6]$");

	if (p == nullptr) {
		qDebug() << "missing project";
		return 1;
	}

	Image* img = p->getDataBlock<Image>(imgId);

	if (img == nullptr) {
		qDebug() << "missing image";
		return 1;
	}

	Camera* cam = p->getDataBlock<Camera>(img->assignedCamera());

	if (cam == nullptr) {
		qDebug() << "missing camera";
		return 1;
	}

	Eigen::Matrix3f r = Eigen::Matrix3f::Identity();
	Eigen::Vector3f t = Eigen::Vector3f::Zero();

	if (img->isFixed()) {

		Eigen::Vector3f raxis;
		raxis.x() = img->xRot().value();
		raxis.y() = img->yRot().value();
		raxis.z() = img->zRot().value();

		r = StereoVision::Geometry::rodriguezFormula(raxis);

		t.x() = img->xCoord().value();
		t.y() = img->yCoord().value();
		t.z() = img->zCoord().value();

	} else if (img->optPos().isSet() and img->optRot().isSet()) {

		Eigen::Vector3f raxis;
		raxis.x() = img->optRot().value(0);
		raxis.y() = img->optRot().value(1);
		raxis.z() = img->optRot().value(2);

		r = StereoVision::Geometry::rodriguezFormula(raxis);

		t.x() = img->optPos().value(0);
		t.y() = img->optPos().value(1);
		t.z() = img->optPos().value(2);
	}

	StereoVision::Geometry::AffineTransform camToWorld(r, t);

	Eigen::Vector2f pp;
	pp[0] = cam->optimizedOpticalCenterX().value();
	pp[1] = cam->optimizedOpticalCenterY().value();


	bool hasRadialDist = cam->optimizedK1().isSet() and cam->optimizedK2().isSet() and cam->optimizedK3().isSet() and cam->useRadialDistortionModel();
	Eigen::Vector3f rp = Eigen::Vector3f::Zero();

	if (hasRadialDist) {
		rp << cam->optimizedK1().value(), cam->optimizedK2().value(), cam->optimizedK3().value();
	}

	bool hasTangentialDist = cam->optimizedP1().isSet() and cam->optimizedP2().isSet() and cam->useTangentialDistortionModel();
	Eigen::Vector2f tp = Eigen::Vector2f::Zero();

	if (hasTangentialDist) {
		tp << cam->optimizedP1().value(), cam->optimizedP2().value();
	}

	float fLen = cam->optimizedFLen().value();

	QVector<qint64> lmIds = img->getAttachedLandmarksIds();

	QSet<QString> treatedHexagones;

	for (qint64 lmId : lmIds) {

		Landmark* lm = p->getDataBlock<Landmark>(lmId);

		if (lm == nullptr) {
			continue;
		}

		QString objName = lm->objectName();

		QRegularExpressionMatch match = hexNamePattern.match(objName);

		if (!match.hasMatch()) {
			continue;
		}

		QString hexName = match.capturedView(1).toString();

		if (treatedHexagones.contains(hexName)) {
			continue; //hexagone already treated.
		}

		qDebug() << "treating hexagone: " << hexName;
		treatedHexagones.insert(hexName);

		std::array<Landmark*,6> hexLandmarks;
		std::array<ImageLandmark*,6> hexImageLandmarks;
		bool allFound = true;

		for (int i = 0; i < 6; i++) {
			QString name = QString("%1_%2").arg(hexName).arg(i+1);

			Landmark* lm = p->getDataBlockByName<Landmark>(name);

			if (lm == nullptr) {
				allFound = false;
				break;
			}

			ImageLandmark* imlm = img->getImageLandmarkByLandmarkId(lm->internalId());

			if (imlm == nullptr) {
				allFound = false;
				break;
			}

			hexLandmarks[i] = lm;
			hexImageLandmarks[i] = imlm;
		}

		if (!allFound) {
			continue;
		}

		Eigen::Array3Xf hexCoordinates;
		hexCoordinates.resize(3,6);

		Eigen::Array2Xf projCoordinates;
		projCoordinates.resize(2,6);

		for (int i = 0; i < 6; i++) {
			hexCoordinates(0,i) = 86.474*std::sin(i*60./180.*M_PI);
			hexCoordinates(1,i) = 86.474*std::cos(i*60./180.*M_PI);
			hexCoordinates(2,i) = 0.000;
		}

		for (int i = 0; i < 6; i++) {

			Eigen::Vector2f coord;
			coord << hexImageLandmarks[i]->x().value(), hexImageLandmarks[i]->y().value();

			coord -= pp;
			coord /= fLen;

			coord = StereoVision::Geometry::invertRadialTangentialDistorstion(coord, rp, tp, 15);

			projCoordinates.col(i) = coord;
		}

		if (!hexCoordinates.allFinite()) {
			qDebug() << "Problem with the hexagone coordinates";
			continue;
		}

		if (!projCoordinates.allFinite()) {
			qDebug() << "Problem with homogeneous coordinates";
			continue;
		}

		StereoVision::Geometry::AffineTransform<float> hexToCam = StereoVision::Geometry::pnp(projCoordinates, hexCoordinates);

		StereoVision::Geometry::AffineTransform<float> hexToWorld(camToWorld.R*hexToCam.R, camToWorld.t + camToWorld.R*hexToCam.t);

		Eigen::Array3Xf hexWorldCoordinates = hexToWorld*hexCoordinates;

		for (int i = 0; i < 6; i++) {

			floatParameterGroup<3> optPos;
			optPos.value(0) = hexWorldCoordinates.col(i)[0];
			optPos.value(1) = hexWorldCoordinates.col(i)[1];
			optPos.value(2) = hexWorldCoordinates.col(i)[2];

			Landmark* lm = hexLandmarks[i];
			lm->setOptPos(optPos);

		}

	}

	MainWindow* mw = MainWindow::getActiveMainWindow();

	if (mw != nullptr) {
		mw->openSparseViewer();
	}

	return 0;

}

int orientCamerasRelativeToObservedLandmarks(qint64 imgId, Project* p) {

	if (p == nullptr) {
		qDebug() << "missing project";
		return 1;
	}

	Image* img = p->getDataBlock<Image>(imgId);

	if (img == nullptr) {
		qDebug() << "missing image";
		return 1;
	}

	Camera* cam = p->getDataBlock<Camera>(img->assignedCamera());

	if (cam == nullptr) {
		qDebug() << "missing camera";
		return 1;
	}

	Eigen::Vector2f pp;

	if (cam->optimizedOpticalCenterX().isSet() and cam->optimizedOpticalCenterY().isSet()) {
		pp[0] = cam->optimizedOpticalCenterX().value();
		pp[1] = cam->optimizedOpticalCenterY().value();
	} else {
		pp[0] = cam->opticalCenterX().value();
		pp[1] = cam->opticalCenterY().value();
	}


	bool hasRadialDist = cam->optimizedK1().isSet() and cam->optimizedK2().isSet() and cam->optimizedK3().isSet() and cam->useRadialDistortionModel();
	Eigen::Vector3f rp = Eigen::Vector3f::Zero();

	if (hasRadialDist) {
		rp << cam->optimizedK1().value(), cam->optimizedK2().value(), cam->optimizedK3().value();
	}

	bool hasTangentialDist = cam->optimizedP1().isSet() and cam->optimizedP2().isSet() and cam->useTangentialDistortionModel();
	Eigen::Vector2f tp = Eigen::Vector2f::Zero();

	if (hasTangentialDist) {
		tp << cam->optimizedP1().value(), cam->optimizedP2().value();
	}

	float fLen;

	if (cam->optimizedFLen().isSet()) {
		fLen = cam->optimizedFLen().value();
	} else {
		fLen = cam->fLen().value();
	}

	QVector<qint64> lmIds = img->getAttachedLandmarksIds();

	QVector<Landmark*> observedLandmarks;
	observedLandmarks.reserve(lmIds.size());

	for (qint64 lmId : lmIds) {

		Landmark* lm = p->getDataBlock<Landmark>(lmId);

		if (lm == nullptr) {
			continue;
		}

		if (lm->optPos().isSet()) {
			observedLandmarks.push_back(lm);
        } else if (lm->xCoord().isSet() and lm->yCoord().isSet() and lm->zCoord().isSet()) {
            observedLandmarks.push_back(lm);
        }

	}

    if (observedLandmarks.size() < 4) {
		qDebug() << "Not enough landmarks available";
		return 1;
	}


	Eigen::Array3Xf pointsCoordinates;
	pointsCoordinates.resize(3,observedLandmarks.size());

	Eigen::Array2Xf projCoordinates;
	projCoordinates.resize(2,observedLandmarks.size());

	for (int i = 0; i < observedLandmarks.size(); i++) {

		Landmark* lm = observedLandmarks[i];

        constexpr bool applyLocalTransform = true;

        if (lm->optPos().isSet()) {
            constexpr bool optimized = true;
            pointsCoordinates.col(i) = lm->getOptimizableCoordinates(optimized, applyLocalTransform).value().cast<float>();
        } else {
            constexpr bool optimized = false;
            pointsCoordinates.col(i) = lm->getOptimizableCoordinates(optimized, applyLocalTransform).value().cast<float>();
        }

		ImageLandmark* imLm = img->getImageLandmarkByLandmarkId(lm->internalId());

		Eigen::Vector2f coord(imLm->x().value(), imLm->y().value());

		coord -= pp;
		coord /= fLen;

		coord = StereoVision::Geometry::invertRadialTangentialDistorstion(coord, rp, tp, 15);

		projCoordinates.col(i) = coord;

	}

	StereoVision::Geometry::AffineTransform world2Cam = StereoVision::Geometry::pnp(projCoordinates, pointsCoordinates);

	Eigen::Matrix3f Rcam2World = world2Cam.R.transpose();
	Eigen::Vector3f raxis = StereoVision::Geometry::inverseRodriguezFormula(Rcam2World);

	Eigen::Vector3f tCam2World = Rcam2World*(-world2Cam.t);

	floatParameterGroup<3> r;
	r.value(0) = raxis[0];
	r.value(1) = raxis[1];
	r.value(2) = raxis[2];

	floatParameterGroup<3> t;
	t.value(0) = tCam2World[0];
	t.value(1) = tCam2World[1];
	t.value(2) = tCam2World[2];

	img->setOptPos(t);
	img->setOptRot(r);

	MainWindow* mw = MainWindow::getActiveMainWindow();

	if (mw != nullptr) {
		mw->openSparseViewer();
	}

	return 0;
}

QTextStream& printImagesRelativePositions(QTextStream & stream, QVector<qint64> imagesIds, Project* p) {

	QMap<qint64, Image*> images;

	for (qint64 id : imagesIds) {
		Image* img = p->getDataBlock<Image>(id);

		if (img != nullptr) {
			images.insert(id, img);
		}
	}

	for (qint64 id1 : images.keys()) {
		Image* img1 = images[id1];

		auto img1ToWorld = getImageToWorldTransform(img1);

		if (! img1ToWorld.has_value()) {
			stream << "Image " << id1 << " has no optimized position, skipping !\n" << Qt::endl;
		}

		StereoVision::Geometry::ShapePreservingTransform worldToImg1 = img1ToWorld->inverse();

		for (qint64 id2 : images.keys()) {
			if (id2 == id1) {
				continue;
			}

			Image* img2 = images[id2];

			auto img2ToWorld = getImageToWorldTransform(img2);

			if (! img2ToWorld.has_value()) {
				continue;
			}

			StereoVision::Geometry::ShapePreservingTransform img2ToImg1 = worldToImg1*img2ToWorld.value();
			StereoVision::Geometry::AffineTransform aff = img2ToImg1.toAffineTransform();

			stream << "Image " << id2 << " to Image " << id1 << " transform:\n" << Qt::endl;

			stream << "R = " << Qt::endl;
			stream << '[';
			for (int i = 0; i < 3; i++) {
				if (i != 0) {
					stream << ' ';
				}
				stream << '[';
				for (int j = 0; j < 3; j++) {
					if (j != 0) {
						stream << ", ";
					}
					stream << aff.R(i,j);
				}
				stream << "]\n";
			}
			stream << ']' << Qt::endl;

			stream << "t = " << Qt::endl;
			stream << '[';
			for (int i = 0; i < 3; i++) {
				if (i != 0) {
					stream << ", \n ";
				}
				stream << aff.t[i];
			}
			stream << "]\n" << Qt::endl;

		}

	}

	return stream;

}



StatusOptionalReturn<void> autoDetectImagesTiePointsHeadless(QMap<QString,QString> const& kwargs, QStringList const& argv) {

    StereoVisionApplication* app = StereoVisionApplication::GetAppInstance();

    if (app == nullptr) {
        return StatusOptionalReturn<void>::error("could not load app instance!");
    }

    Project* p = app->getCurrentProject();

    if (p == nullptr) {
        return StatusOptionalReturn<void>::error("could not access any active project!");
    }

    //images

    if (argv.size() < 2) {
        return StatusOptionalReturn<void>::error("Invalid number of arguments, expected at least two images");
    }

    QString const& image1UrlStr = argv[0];
    QString const& image2UrlStr = argv[1];

    QVector<qint64> image1Url = DataBlock::decodeUrl(image1UrlStr);
    QVector<qint64> image2Url = DataBlock::decodeUrl(image2UrlStr);

    Image* img1 = qobject_cast<Image*>(p->getByUrl(image1Url));
    Image* img2 = qobject_cast<Image*>(p->getByUrl(image2Url));

    if (img1 == nullptr or img2 == nullptr) {
        return StatusOptionalReturn<void>::error("Could not load images, invalid images references! Are the datablock present in the project?");
    }

    //matching pipeline configuration

    using ComputeType = float;

    using CornerDetecor = HarrisCornerDetectorModule<ComputeType>;
    using MatchModule = HungarianCornerMatchModule<ComputeType>;
    using InlierModule = GenericInlinerSelectionModule<ComputeType>;

    using MatchingPipeline = ModularSparseMatchingPipeline<ComputeType,
                                                           CornerDetecor,
                                                           MatchModule,
                                                           GenericTiePointsRenfinementModule<ComputeType>,
                                                           GenericInlinerSelectionModule<ComputeType>>;

    int lowPassRadius = 3;
    int nonMaxSupprRadius = 3;
    int maxNCorners = 1000;

    if (kwargs.contains("cornerLowPassRadius")) {
        bool ok = true;
        lowPassRadius = kwargs["cornerLowPassRadius"].toInt(&ok);

        if (!ok) {
            return StatusOptionalReturn<void>::error("argument cornerLowPassRadius has invalid value (must be convertible to int)");
        }
    }

    if (kwargs.contains("cornerNonMaxSupprRadius")) {
        bool ok = true;
        nonMaxSupprRadius = kwargs["cornerNonMaxSupprRadius"].toInt(&ok);

        if (!ok) {
            return StatusOptionalReturn<void>::error("argument cornerNonMaxSupprRadius has invalid value (must be convertible to int)");
        }
    }

    if (kwargs.contains("cornerMaxNCorners")) {
        bool ok = true;
        maxNCorners = kwargs["cornerMaxNCorners"].toInt(&ok);

        if (!ok) {
            return StatusOptionalReturn<void>::error("argument cornerMaxNCorners has invalid value (must be convertible to int)");
        }
    }

    // not used at the moment
    constexpr int patchRadius = 100;
    constexpr int nSamples = 100;

    int nRansacIterations = 20000;

    if (kwargs.contains("ransacIterations")) {
        bool ok = true;
        nRansacIterations = kwargs["ransacIterations"].toInt(&ok);

        if (!ok) {
            return StatusOptionalReturn<void>::error("argument ransacIterations has invalid value (must be convertible to int)");
        }
    }

    float epipolarRansacThreshold = 0.02;
    float perspectiveRansacThreshold = 20;

    if (kwargs.contains("ransacThreshold")) {
        bool ok = true;
        float threshold = kwargs["ransacThreshold"].toFloat(&ok);

        if (!ok) {
            return StatusOptionalReturn<void>::error("argument ransacThreshold has invalid value (must be convertible to float)");
        }
        epipolarRansacThreshold = threshold;
        perspectiveRansacThreshold = threshold;
    }

    CornerDetecor* baseCornerDetectionModule = new CornerDetecor(lowPassRadius, nonMaxSupprRadius, maxNCorners);
    MatchModule* basePointsMatchingModule = new HungarianCornerMatchModule<float>(patchRadius, nSamples);
    InlierModule* baseInlierModule = nullptr;
    if (kwargs.contains("inlierModule")) {
        if (kwargs["inlierModule"] == "perspective") {
            baseInlierModule = new RansacPerspectiveInlinerSelectionModule<ComputeType>(nRansacIterations, perspectiveRansacThreshold);
        } else {
            baseInlierModule = new RansacEpipolarInlinerSelectionModule<ComputeType>(nRansacIterations, epipolarRansacThreshold);
        }
    } else {
        baseInlierModule = new RansacEpipolarInlinerSelectionModule<ComputeType>(nRansacIterations, epipolarRansacThreshold);
    }

    MatchingPipeline matchingPipeline(baseCornerDetectionModule, basePointsMatchingModule, nullptr, baseInlierModule);

    //run the pipeline

    Multidim::Array<float,3> img1_data = StereoVision::IO::readImage<float>(img1->getImageFile().toStdString());

    if (img1_data.empty()) {
        return StatusOptionalReturn<void>::error("Could not load raster data from img1!");
    }

    Multidim::Array<float,3> img2_data = StereoVision::IO::readImage<float>(img2->getImageFile().toStdString());

    if (img2_data.empty()) {
        return StatusOptionalReturn<void>::error("Could not load raster data from img2!");
    }

    matchingPipeline.setupPipeline(img1_data, img2_data);
    matchingPipeline.runAllSteps();

    if (!matchingPipeline.lastErrorMessage().isEmpty()) {
        return StatusOptionalReturn<void>::error(qPrintable(QString("Error during matching process: %1").arg(matchingPipeline.lastErrorMessage())));
    }

    Image::ImageMatchBuilder builder1(img1);
    Image::ImageMatchBuilder builder2(img2);

    auto[uvs1, uvs2] = matchingPipeline.filteredCorners();

    auto status = CorrespondencesSet::writeUVCorrespondancesToSet(&builder1, uvs1, &builder2, uvs2, p);

    if (!status.isValid()) {
        return StatusOptionalReturn<void>::error(status.errorMessage());
    }

    return StatusOptionalReturn<void>();
}

} //namespace StereoVisionApp
