#include "cornerdetectionactions.h"

#include "mainwindow.h"

#include "gui/cornerdetectortesteditor.h"
#include "gui/cornermatchingtesteditor.h"

#include "datablocks/project.h"
#include "datablocks/image.h"

#include <StereoVision/io/image_io.h>

#include <MultidimArrays/MultidimArrays.h>

#include <QTextStream>

namespace StereoVisionApp {

Multidim::Array<float, 3> generateTestImage(std::array<int, 2> const& shape,
                                            std::array<float, 2> const& ptR,
                                            std::array<float, 2> const& ptG,
                                            std::array<float, 2> const& ptB) {

    Multidim::Array<float, 3> ret(shape[0], shape[1], 3);

    for (int i = 0; i < shape[0]; i++) {

        for (int j = 0; j < shape[1]; j++) {

            std::array<float, 3> color = {0,0,0};
            std::array<float, 2> dPix = {-0.5, 0.5};

            for (float di : dPix) {
                for (float dj : dPix) {

                    Eigen::Vector2f pos;
                    pos << i + di - ptB[0], j + dj - ptB[1];

                    Eigen::Matrix2f A;
                    A(0,0) = ptR[0] - ptB[0];
                    A(1,0) = ptR[1] - ptB[1];
                    A(0,1) = ptG[0] - ptB[0];
                    A(1,1) = ptG[1] - ptB[1];

                    Eigen::Vector2f coord = A.inverse()*pos;

                    bool inside = coord[0] >= 0 and coord[1] >= 0 and 1-coord[0]-coord[1] >= 0;

                    if (inside) {
                        color[0] += 0.25*255*coord[0];
                        color[1] += 0.25*255*coord[1];
                        color[2] += 0.25*255*(1-coord[0]-coord[1]);
                    }

                }
            }

            ret.atUnchecked(i,j,0) = color[0];
            ret.atUnchecked(i,j,1) = color[1];
            ret.atUnchecked(i,j,2) = color[2];
        }

    }

    return ret;

}

void detectCornerInTestImage() {

    QTextStream out(stdout);

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (mw == nullptr) {
        out << "No main windows available" << Qt::endl;
        return; //no headless at the moment
    }

    float dx = std::cos(30/180*M_PI)*100;
    float dy = std::sin(30/180*M_PI)*100;

    Multidim::Array<float,3> img_data = generateTestImage({600, 1000},
                                                          {300-100,500},
                                                          {300+dy,500+dx},
                                                          {300+dy,500-dx});

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

void matchCornersInTestImagePair() {

    QTextStream out(stdout);

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (mw == nullptr) {
        out << "No main windows available" << Qt::endl;
        return; //no headless at the moment
    }

    float dx = std::cos(30/180*M_PI)*100;
    float dy = std::sin(30/180*M_PI)*100;

    float dx1 = 100;
    float dy1 = -50;

    Multidim::Array<float,3> img_data1 = generateTestImage({600, 1000},
                                                           {300+dy1-100,500+dx1},
                                                           {300+dy1+dy,500+dx1+dx},
                                                           {300+dy1+dy,500+dx1-dx});

    float dx2 = -75;
    float dy2 = -100;

    Multidim::Array<float,3> img_data2 = generateTestImage({600, 1000},
                                                           {300+dy2-100,500+dx2},
                                                           {300+dy2+dy,500+dx2+dx},
                                                           {300+dy2+dy,500+dx2-dx});


    Editor* editor = mw->openEditor(CornerMatchingTestEditor::staticMetaObject.className());

    if (editor == nullptr) {
        out << "No editor available" << Qt::endl;
        return;
    }

    CornerMatchingTestEditor* cmte = qobject_cast<CornerMatchingTestEditor*>(editor);

    if (cmte == nullptr) {
        out << "Failed to cast editor" << Qt::endl;
        return;
    }

    cmte->setImageData(img_data1, img_data2);
}

void matchCornersInImagePair(Project* proj, qint64 imgId1, qint64 imgId2) {

    QTextStream out(stdout);

    if (proj == nullptr) {
        return;
    }

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (mw == nullptr) {
        out << "No main windows available" << Qt::endl;
        return; //no headless at the moment
    }

    Image* img_block1 = proj->getDataBlock<Image>(imgId1);
    Image* img_block2 = proj->getDataBlock<Image>(imgId2);

    if (img_block1 == nullptr) {
        out << "No image block available" << Qt::endl;
        return;
    }

    if (img_block2 == nullptr) {
        out << "No image block available" << Qt::endl;
        return;
    }

    QString img_path1 = img_block1->getImageFile();
    QString img_path2 = img_block2->getImageFile();

    Multidim::Array<float,3> img_data1 = StereoVision::IO::readImage<float>(img_path1.toStdString());

    if (img_data1.empty()) {
        out << "Empty image 1 data" << Qt::endl;
        return;
    }

    Multidim::Array<float,3> img_data2 = StereoVision::IO::readImage<float>(img_path2.toStdString());

    if (img_data2.empty()) {
        out << "Empty image 2 data" << Qt::endl;
        return;
    }

    Editor* editor = mw->openEditor(CornerMatchingTestEditor::staticMetaObject.className());

    if (editor == nullptr) {
        out << "No editor available" << Qt::endl;
        return;
    }

    CornerMatchingTestEditor* cmte = qobject_cast<CornerMatchingTestEditor*>(editor);

    if (cmte == nullptr) {
        out << "Failed to cast editor" << Qt::endl;
        return;
    }

    cmte->setImageData(img_data1, img_data2);

    class ImageMatchBuilder : public CornerMatchingTestEditor::MatchBuilder {
    public:
        ImageMatchBuilder(Image* img) : _img(img) {

        }
        virtual Correspondences::Generic correspondanceFromUV(float u, float v) const {
            Correspondences::Typed<Correspondences::UV> ret;
            ret.blockId = _img->internalId();
            ret.u = u;
            ret.v = v;
            return ret;
        }
        virtual QString targetTitle() const {
            return _img->objectName();
        }
    protected:
        Image* _img;
    };

    cmte->setMatchBuilders(new ImageMatchBuilder(img_block1), new ImageMatchBuilder(img_block2));

}

} // namespace StereoVisionApp
