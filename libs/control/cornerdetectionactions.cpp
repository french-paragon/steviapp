#include "cornerdetectionactions.h"

#include "mainwindow.h"

#include "gui/cornerdetectortesteditor.h"

#include "datablocks/project.h"
#include "datablocks/image.h"

#include "LibStevi/io/image_io.h"

#include <QTextStream>

namespace StereoVisionApp {

void detectCornerInImage(Project* proj, qint64 imgId) {

    QTextStream out(stdout);

    if (proj == nullptr) {
        return;
    }

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (mw == nullptr) {
        out << "No main windows available" << Qt::endl;
        return; //no headless at the moment
    }

    Image* img_block = proj->getDataBlock<Image>(imgId);

    if (img_block == nullptr) {
        out << "No image block available" << Qt::endl;
        return;
    }

    QString img_path = img_block->getImageFile();

    Multidim::Array<float,3> img_data = StereoVision::IO::readImage<float>(img_path.toStdString());

    if (img_data.empty()) {
        out << "Empty image data" << Qt::endl;
        return;
    }

    Editor* editor = mw->openEditor(CornerDetectorTestEditor::staticMetaObject.className());

    if (editor == nullptr) {
        out << "No editor available" << Qt::endl;
        return;
    }

    CornerDetectorTestEditor* cdte = qobject_cast<CornerDetectorTestEditor*>(editor);

    if (cdte == nullptr) {
        out << "Failed to cast editor" << Qt::endl;
        return;
    }

    cdte->setImageData(img_data);

}

} // namespace StereoVisionApp
