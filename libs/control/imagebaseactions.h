#ifndef IMAGEBASEACTIONS_H
#define IMAGEBASEACTIONS_H

#include <QStringList>
#include <QTextStream>

#include "../utils/statusoptionalreturn.h"

class QWidget;

namespace StereoVisionApp {

class Project;
class Image;
class Camera;

/*!
 * \brief addImage add a single placeholder image (image without a file) to the project.
 * \param name the name of the new image
 * \param p the project to add the image to.
 * \param cam the camera to use (cannot be nullptr, as no camera can be infered from the image metadata).
 * \return the id of the image, or -1 on failure.
 *
 * An image without an attached file can still be used for photogrametry, as long as matches are imported from external sources
 */
qint64 addPlaceholderImage(QString name, Project* p, Camera *cam);

/*!
 * \brief addImage add a single image to the project.
 * \param filename the filename of the image
 * \param p the project to add the image to.
 * \param cam the camera to use (if nullptr, camera will be infered, and possibly created, from the image metadata).
 * \return the id of the image, or -1 on failure.
 */
qint64 addImage(QString filename, Project* p, Camera *cam = nullptr);

/*!
 * \brief addImages add a series of images to a project
 * \param images the list of paths for the images to add
 * \param p the project to add the images into
 * \return The list of images that could NOT be added.
 */
QStringList addImages(QStringList images, Project* p);

/*!
 * \brief exportRectifiedImages undistort images and export the undistorted version.
 * \param imagesIds The ids of the images to export in the project
 * \param p the project where the images are located
 * \param outputDirectory the directory where the undistorted images are to be saved
 * \return the number of images successfully treated.
 */
int exportRectifiedImages(QList<qint64> imagesIds, Project* p, bool useOptimizedVals = true, QString outputDirectory = "", float gamma = 2.2);

int exportRectifiedImages(QList<qint64> imagesIds, Project* p, QWidget* w);

int exportStereoRigRectifiedImages(QList<qint64> imagesIds, qint64 rigId, Project* p, QWidget* w);

int exportImageLandmarksPositionsToCSV(qint64 imageId, Project* p, QWidget* w);

int addImagesToCalibration(QList<qint64> imagesIds, qint64 calibId, Project* p);

int detectHexagonalTargets(QList<qint64> imagesIds, Project* p);

bool detectHexagonalTargets(Image* img,
							double minThreshold = 80,
							double diffThreshold = 30,
							int minArea = 10,
							int maxArea = 800,
							double minToMaxAxisRatioThreshold = 0.6,
							double hexRelMaxDiameter = 0.2,
							double hexFirRelMaxRes = 0.1,
							double redGain = 1.1,
							double greenGain = 1.1,
							double blueGain = 1.0,
							bool clearPrevious = false,
							bool useHexScale = false,
							float hexEdge = 90.016);

int orientHexagonalTargetsRelativeToCamera(qint64 imgId, Project* p);

int orientCamerasRelativeToObservedLandmarks(qint64 imgId, Project* p);


QTextStream& printImagesRelativePositions(QTextStream & stream, QVector<qint64> imagesIds, Project* p);

StatusOptionalReturn<void> autoDetectImagesTiePointsHeadless(QMap<QString,QString> const& kwargs, QStringList const& argv);

} // namespace StereoVisionApp

#endif // IMAGEBASEACTIONS_H
