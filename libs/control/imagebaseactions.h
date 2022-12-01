#ifndef IMAGEBASEACTIONS_H
#define IMAGEBASEACTIONS_H

#include <QStringList>
#include <QTextStream>

class QWidget;

namespace StereoVisionApp {

class Project;
class Image;


/*!
 * \brief addImage add a single image to the project.
 * \param filename the filename of the image
 * \param p the project to add the image to.
 * \return the id of the image, or -1 on failure.
 */
qint64 addImage(QString filename, Project* p);

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

} // namespace StereoVisionApp

#endif // IMAGEBASEACTIONS_H
