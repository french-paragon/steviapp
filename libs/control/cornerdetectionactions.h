#ifndef STEREOVISIONAPP_CORNERDETECTIONACTIONS_H
#define STEREOVISIONAPP_CORNERDETECTIONACTIONS_H

#include <QtGlobal>

#include <QPixmap>

#include "../utils/statusoptionalreturn.h"

namespace StereoVisionApp {

class Project;
class HeadlessSparseMatchingPipelineInterface;

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

HeadlessSparseMatchingPipelineInterface* getAppHeadlessSparseMatchingInterface();

StatusOptionalReturn<void> configureModularHeadlessSparseMatchingPipeline();

StatusOptionalReturn<void> setupHeadlessHarrisCornerDetectorModule(int lowPassRadius,
                                                                          int nonMaximumSuppressionRadius,
                                                                          int maxNCorners);

StatusOptionalReturn<void> setupHeadlessHungarianCornerMatchModule(int patchRadius, int nSamples);

StatusOptionalReturn<void> setupHeadlessBestNCornerMatchModule(int nMatches, float maxRatio2Best);

StatusOptionalReturn<void> setupHeadlessRansacEpipolarInlinerSelectionModule(int nRansacIterations, float threshold);
StatusOptionalReturn<void> setupHeadlessRansacPerspectiveInlinerSelectionModule(int nRansacIterations,
                                                                                       bool enableMultiThresholding,
                                                                                       float threshold,
                                                                                       float subthreshold);

StatusOptionalReturn<void>  addImageToHeadlessSparseMatching(Project* proj, qint64 imgId);

StatusOptionalReturn<void> runHeadlessMatching();

StatusOptionalReturn<void> printHeadlessSparseMatchingResults(QString const& filePath);
StatusOptionalReturn<std::string> getHeadlessSparseMatchingResults();

StatusOptionalReturn<void> exportHeadlessSparseMatchingResultsView(QString const& outFilePath);

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CORNERDETECTIONACTIONS_H
