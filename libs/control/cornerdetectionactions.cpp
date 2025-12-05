#include "cornerdetectionactions.h"

#include "mainwindow.h"

#include "gui/cornerdetectortesteditor.h"
#include "gui/cornermatchingeditor.h"

#include "datablocks/project.h"
#include "datablocks/image.h"

#include <StereoVision/io/image_io.h>

#include <MultidimArrays/MultidimArrays.h>

#include <QTextStream>
#include <QPdfWriter>
#include <QPainter>
#include <QFile>

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


    Editor* editor = mw->openEditor(CornerMatchingEditor::staticMetaObject.className());

    if (editor == nullptr) {
        out << "No editor available" << Qt::endl;
        return;
    }

    CornerMatchingEditor* cmte = qobject_cast<CornerMatchingEditor*>(editor);

    if (cmte == nullptr) {
        out << "Failed to cast editor" << Qt::endl;
        return;
    }

    cmte->addImageData("simulatedImage1", img_data1, nullptr);
    cmte->addImageData("simulatedImage2", img_data2, nullptr);
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

    Editor* editor = mw->openEditor(CornerMatchingEditor::staticMetaObject.className());

    if (editor == nullptr) {
        out << "No editor available" << Qt::endl;
        return;
    }

    CornerMatchingEditor* cmte = qobject_cast<CornerMatchingEditor*>(editor);

    if (cmte == nullptr) {
        out << "Failed to cast editor" << Qt::endl;
        return;
    }

    cmte->addImageData(img_block1->objectName(), img_data1, new Image::ImageMatchBuilder(img_block1));
    cmte->addImageData(img_block2->objectName(), img_data2, new Image::ImageMatchBuilder(img_block2));

}

StatusOptionalReturn<void> exportCorrespondancesFromImages(QString const& outFile, QString const& img1Label, QString const& img2Label,
                                     QPixmap const& img1, QPixmap const& img2,
                                     QVector<QPointF> const& coords1, QVector<QPointF> const& coords2,
                                     QVector<QString> const& labels, QVector<QColor> const& colors) {

    if (img1.isNull() and img2.isNull()) {
        return StatusOptionalReturn<void>::error("Null images 1 & 2");
    } else if (img1.isNull()) {
        return StatusOptionalReturn<void>::error("Null image 1");
    } else if (img2.isNull()) {
        return StatusOptionalReturn<void>::error("Null image 2");
    }

    if (coords1.size() != coords2.size()) {
        return StatusOptionalReturn<void>::error("Incompatible number of points given!");
    }

    int nPoints = coords1.size();

    QFile out(outFile);

    if (!out.open(QFile::WriteOnly)) {
        return StatusOptionalReturn<void>::error(qPrintable(QString("Impossible to open file %1 in write mode!").arg(outFile)));
    }

    QPdfWriter writer(&out);
    writer.setResolution(72);

    QSize paperSize = QPageSize(QPageSize::A4).sizePoints();

    if (paperSize.width() < paperSize.height()) {
        paperSize.transpose(); //set horizontal
    }

    writer.setPageSize(QPageSize(paperSize));
    writer.setPageMargins(QMarginsF(0,0,0,0));

    QSize img1Size = img1.size();
    QSize img2Size = img2.size();

    constexpr float marginInPoints = 5;
    constexpr float spacingInPoints = 50;

    float availableHeight = paperSize.height() - 2*marginInPoints;
    float availableWidth = (paperSize.width() - 2*marginInPoints - spacingInPoints)/2;

    float img1HeightScale = availableHeight / img1Size.height();
    float img2HeightScale = availableHeight / img2Size.height();


    float img1WidthScale = availableWidth / img1Size.width();
    float img2WidthScale = availableWidth / img2Size.width();

    float img1Scale = std::min(img1HeightScale, img1WidthScale);
    float img2Scale = std::min(img2HeightScale, img2WidthScale);

    QSizeF img1PrintSize = QSizeF(img1Size)*img1Scale;
    QSizeF img2PrintSize = QSizeF(img2Size)*img2Scale;

    float leftoverHorizontalSpace1 = availableWidth - img1PrintSize.width();
    float leftoverVerticalSpace1 = availableHeight - img1PrintSize.height();

    float leftoverHorizontalSpace2 = availableWidth - img2PrintSize.width();
    float leftoverVerticalSpace2 = availableHeight - img2PrintSize.height();

    QPointF img1TopLeftCorner(marginInPoints + leftoverHorizontalSpace1/2,
                              marginInPoints + leftoverVerticalSpace1/2);
    QPointF img2TopLeftCorner(marginInPoints + availableWidth + spacingInPoints + leftoverHorizontalSpace2/2,
                              marginInPoints + leftoverVerticalSpace2/2);

    QPainter painter(&writer);

    painter.drawPixmap(QRectF(img1TopLeftCorner, img1PrintSize).toRect(), img1);
    painter.drawPixmap(QRectF(img2TopLeftCorner, img2PrintSize).toRect(), img2);

    QVector<QColor> linkColors;

    if (!colors.isEmpty()) {
        linkColors = colors;
    } else {
        linkColors = QVector<QColor>(coords1.size());
        std::default_random_engine re(img1Size.width() + img1Size.height() + img2Size.width() + img2Size.height()); //deterministic seeding
        std::uniform_int_distribution<uint8_t> dist(0,230);

        for (int i = 0; i < linkColors.size(); i++) {
            linkColors[i] = QColor(dist(re),dist(re),dist(re));
        }
    }

    QVector<QString> pointsLabels;

    if (!labels.isEmpty()) {
        pointsLabels = labels;
    } else {
        pointsLabels = QVector<QString>(coords1.size());

        for (int i = 0; i < pointsLabels.size(); i++) {
            pointsLabels[i] = "";
        }
    }

    for (int i = 0; i < nPoints; i++) {
        QPointF ptImg1 = img1Scale*coords1[i] + img1TopLeftCorner;
        QPointF ptImg2 = img2Scale*coords2[i] + img2TopLeftCorner;

        QPen pen(linkColors[i]);
        pen.setWidthF(0.3);
        pen.setStyle(Qt::PenStyle::SolidLine);

        painter.setPen(pen);
        painter.drawLine(ptImg1, ptImg2);
    }

    return StatusOptionalReturn<void>();
}

void addImages2MatchCornersEditor(Project* proj, QVector<qint64> const& imgs) {

    QTextStream out(stdout);

    if (proj == nullptr) {
        return;
    }

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (mw == nullptr) {
        out << "No main windows available" << Qt::endl;
        return; //no headless at the moment
    }

    Editor* editor = mw->openEditor(CornerMatchingEditor::staticMetaObject.className());

    if (editor == nullptr) {
        out << "No editor available" << Qt::endl;
        return;
    }

    CornerMatchingEditor* cmte = qobject_cast<CornerMatchingEditor*>(editor);

    if (cmte == nullptr) {
        out << "Failed to cast editor" << Qt::endl;
        return;
    }

    for (qint64 id : imgs) {

        Image* img_block = proj->getDataBlock<Image>(id);

        if (img_block == nullptr) {
            out << "No image block available for id " << id << Qt::endl;
            continue;
        }

        QString img_path = img_block->getImageFile();

        Multidim::Array<float,3> img_data = StereoVision::IO::readImage<float>(img_path.toStdString());

        if (img_data.empty()) {
            out << "Empty image data for id " << id << Qt::endl;
            continue;
        }

        cmte->addImageData(img_block->objectName(), img_data, new Image::ImageMatchBuilder(img_block));

    }

}

} // namespace StereoVisionApp
