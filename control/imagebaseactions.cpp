#include "imagebaseactions.h"

#include "datablocks/project.h"
#include "datablocks/image.h"
#include "datablocks/camera.h"

#include "interpolation/interpolation.h"
#include "interpolation/lensdistortionsmap.h"

#include "vision/imageio.h"

#include "mainwindow.h"
#include "gui/rectifiedimageexportoptionsdialog.h"

#include <QFileInfo>
#include <QDir>
#include <QPixmap>

#include <exiv2/exiv2.hpp>

namespace StereoVisionApp {

QStringList addImages(QStringList images, Project* p) {

	QString cn = ImageFactory::imageClassName();

	QStringList failed;

	for (QString const& f : images) {

		Exiv2::Image::AutoPtr image = Exiv2::ImageFactory::open(f.toStdString());
		QPixmap pixmap(f);

		if(image.get() == 0 or pixmap.isNull()) {
			failed << f;
			continue;
		}

		qint64 id = p->createDataBlock(cn.toStdString().c_str());

		if (id >= 0) {
			Image* im = qobject_cast<Image*>(p->getById(id));
			im->setImageFile(f);

			QFileInfo inf(f);
			im->setObjectName(inf.baseName());

			Camera* imgCam = nullptr;

			QString camName;

			image->readMetadata();
			Exiv2::ExifData &exifData = image->exifData();
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

		} else {
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

				StereoVision::ImageArray array = getImageData(img->getImageFile(), gamma);

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

					if (saveImageData(outFile, transformed, gamma)) {
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

} // namespace StereoVisionApp
