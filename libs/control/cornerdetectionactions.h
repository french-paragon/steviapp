#ifndef STEREOVISIONAPP_CORNERDETECTIONACTIONS_H
#define STEREOVISIONAPP_CORNERDETECTIONACTIONS_H

#include <QtGlobal>

#include <QPixmap>

#include "../utils/statusoptionalreturn.h"

namespace StereoVisionApp {

class Project;

void detectCornerInTestImage();
void detectCornerInImage(Project* proj, qint64 imgId);

void matchCornersInTestImagePair();
void matchCornersInImagePair(Project* proj, qint64 imgId1, qint64 imgId2);

StatusOptionalReturn<void> exportCorrespondancesFromImages(QString const& outFile, QString const& img1Label, QString const& img2Label,
                                     QPixmap const& img1, QPixmap const& img2,
                                     QVector<QPointF> const& coords1, QVector<QPointF> const& coords2,
                                     QVector<QString> const& labels = QVector<QString>(), QVector<QColor> const& colors = QVector<QColor>());

inline StatusOptionalReturn<void> exportCorrespondancesFromImages(QString const& outFile, QString const& img1Label, QString const& img2Label,
                                            QPixmap const& img1, QPixmap const& img2,
                                            QVector<QPointF> const& coords1, QVector<QPointF> const& coords2,
                                            QVector<QColor> const& colors) {
    return exportCorrespondancesFromImages(outFile,img1Label, img2Label, img1, img2, coords1, coords2, QVector<QString>(), colors);
}

void addImages2MatchCornersEditor(Project* proj, QVector<qint64> const& imgs);

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CORNERDETECTIONACTIONS_H
