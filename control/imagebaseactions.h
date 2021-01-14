#ifndef IMAGEBASEACTIONS_H
#define IMAGEBASEACTIONS_H

#include <QStringList>

class QWidget;

namespace StereoVisionApp {

class Project;

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

} // namespace StereoVisionApp

#endif // IMAGEBASEACTIONS_H
