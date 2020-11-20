#ifndef IMAGEBASEACTIONS_H
#define IMAGEBASEACTIONS_H

#include <QStringList>

namespace StereoVisionApp {

class Project;

/*!
 * \brief addImages add a series of images to a project
 * \param images the list of paths for the images to add
 * \param p the project to add the images into
 * \return The list of images that could NOT be added.
 */
QStringList addImages(QStringList images, Project* p);

} // namespace StereoVisionApp

#endif // IMAGEBASEACTIONS_H
