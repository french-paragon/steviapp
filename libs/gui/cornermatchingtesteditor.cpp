#include "cornermatchingtesteditor.h"

#include <StereoVision/sparseMatching/cornerDetectors.h>
#include <StereoVision/sparseMatching/nonLocalMaximumPointSelection.h>
#include <StereoVision/sparseMatching/pointsDescriptors.h>
#include <StereoVision/sparseMatching/pointsOrientation.h>

#include <StereoVision/correlation/matching_costs.h>

#include <StereoVision/optimization/assignement_problems.h>

#include "imagewidget.h"

#include "gui/imagedisplayoverlays/labelledpointsoverlay.h"

#include "vision/sparseMatchingPipeline.h"

#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QSpinBox>
#include <QPushButton>
#include <QSet>

namespace SpaMat = StereoVision::SparseMatching;
namespace Corr = StereoVision::Correlation;
namespace Optm = StereoVision::Optimization;

namespace StereoVisionApp {

CornerMatchingTestEditor::CornerMatchingTestEditor(QWidget *parent)
{

    QVBoxLayout* verticalLayout = new QVBoxLayout();

    QHBoxLayout* imagesAlignementLayout = new QHBoxLayout();

    _imageDisplay1 = new ImageWidget(this);
    _imageDisplay2 = new ImageWidget(this);

    _imageViewAdapter1 = new StereoVision::Gui::ArrayDisplayAdapter<float>(&_imgData1, 0, 255, 1, 0, 2, {0,1,2}, this);
    _imageViewAdapter2 = new StereoVision::Gui::ArrayDisplayAdapter<float>(&_imgData2, 0, 255, 1, 0, 2, {0,1,2}, this);

    _pointsOverlay1 = new LabelledPointsOverlay(this);
    _pointsOverlay2 = new LabelledPointsOverlay(this);

    connect(_pointsOverlay1, &LabelledPointsOverlay::activePointChanged, _pointsOverlay2, &LabelledPointsOverlay::setActivePoint);
    connect(_pointsOverlay2, &LabelledPointsOverlay::activePointChanged, _pointsOverlay1, &LabelledPointsOverlay::setActivePoint);

    _imageDisplay1->setImage(_imageViewAdapter1);
    _imageDisplay1->addOverlay(_pointsOverlay1);
    _imageDisplay1->installEventFilter(_pointsOverlay1);

    _imageDisplay2->setImage(_imageViewAdapter2);
    _imageDisplay2->addOverlay(_pointsOverlay2);
    _imageDisplay2->installEventFilter(_pointsOverlay2);

    _imageDisplay1->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
    _imageDisplay2->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));

    imagesAlignementLayout->addWidget(_imageDisplay1);
    imagesAlignementLayout->addWidget(_imageDisplay2);

    verticalLayout->addLayout(imagesAlignementLayout);

    QHBoxLayout* optionsLayout = new QHBoxLayout();

    QGroupBox* cornerOptionsBox = new QGroupBox(this);
    cornerOptionsBox->setTitle(tr("Corner detection options"));

    QFormLayout* cornerFormLayout = new QFormLayout();

    _lowPassRadius = new QSpinBox(this);
    _lowPassRadius->setMinimum(1);
    _lowPassRadius->setMaximum(10);
    _lowPassRadius->setSuffix("px");

    _lowPassRadius->setValue(3);

    _nonMaximumMaxSuppressionRadius = new QSpinBox(this);
    _nonMaximumMaxSuppressionRadius->setMinimum(1);
    _nonMaximumMaxSuppressionRadius->setMaximum(25);
    _nonMaximumMaxSuppressionRadius->setSuffix("px");

    _nonMaximumMaxSuppressionRadius->setValue(3);

    _nSelected = new QSpinBox(this);
    _nSelected->setMinimum(-1);
    _nSelected->setMaximum(999999);

    _nSelected->setValue(-1);

    cornerFormLayout->addRow(tr("Low pass radius"), _lowPassRadius);
    cornerFormLayout->addRow(tr("Non maximum suppression radius"), _nonMaximumMaxSuppressionRadius);
    cornerFormLayout->addRow(tr("Max # selected points"), _nSelected);

    cornerOptionsBox->setLayout(cornerFormLayout);

    optionsLayout->addWidget(cornerOptionsBox);

    QGroupBox* featureOptionsBox = new QGroupBox(this);
    featureOptionsBox->setTitle(tr("Features matching options"));

    QFormLayout* featureFormLayout = new QFormLayout();

    _featurePatchRadius = new QSpinBox(this);
    _featurePatchRadius->setMinimum(1);
    _featurePatchRadius->setMaximum(99);
    _featurePatchRadius->setSuffix("px");

    _featurePatchRadius->setValue(10);

    _nFeatures = new QSpinBox(this);
    _nFeatures->setMinimum(1);
    _nFeatures->setMaximum(1024);

    _nFeatures->setValue(100);

    featureFormLayout->addRow(tr("Features patch radius"), _featurePatchRadius);
    featureFormLayout->addRow(tr("Features vector size"), _nFeatures);

    featureOptionsBox->setLayout(featureFormLayout);

    optionsLayout->addWidget(featureOptionsBox);

    verticalLayout->addLayout(optionsLayout);

    QHBoxLayout* buttonLayout = new QHBoxLayout();

    QPushButton* computeButton = new QPushButton(this);
    computeButton->setText(tr("Compute"));

    connect(computeButton, &QPushButton::clicked, this, &CornerMatchingTestEditor::compute);

    buttonLayout->addStretch();
    buttonLayout->addWidget(computeButton);

    verticalLayout->addLayout(buttonLayout);

    setLayout(verticalLayout);

}

void CornerMatchingTestEditor::setImageData(Multidim::Array<float, 3> const& imgData1,
                                            Multidim::Array<float, 3> const& imgData2) {
    _imgData1 = imgData1;
    _imgData2 = imgData2;

    _pointsOverlay1->setPointSet({});
    _pointsOverlay2->setPointSet({});

    _imageViewAdapter1->imageDataUpdated();
    _imageViewAdapter2->imageDataUpdated();

    _imageDisplay1->update();
    _imageDisplay2->update();
}

void CornerMatchingTestEditor::setImageData(Multidim::Array<float, 3> && imgData1,
                                            Multidim::Array<float, 3> && imgData2) {
    _imgData1 = imgData1;
    _imgData2 = imgData2;

    _pointsOverlay1->setPointSet({});
    _pointsOverlay2->setPointSet({});

    _imageViewAdapter1->imageDataUpdated();
    _imageViewAdapter2->imageDataUpdated();

    _imageDisplay1->update();
    _imageDisplay2->update();

}

void CornerMatchingTestEditor::clearImageData() {

    _imgData1 = Multidim::Array<float, 3>();
    _imgData2 = Multidim::Array<float, 3>();

    _pointsOverlay1->setPointSet({});
    _pointsOverlay2->setPointSet({});

    _imageDisplay1->update();
    _imageDisplay2->update();
}

void CornerMatchingTestEditor::compute() {

    constexpr int batchDim = 2;
    constexpr int nDim = 3;

    if (_imgData1.empty()) {

        _imageDisplay1->update();
        _imageDisplay2->update();

        return;
    }

    if (_imgData2.empty()) {

        _imageDisplay1->update();
        _imageDisplay2->update();

        return;
    }

    int lpRadius = _lowPassRadius->value();

    int nomMaxSupprRadius = _nonMaximumMaxSuppressionRadius->value();
    int nItems = _nSelected->value();

    int patchRadius = _featurePatchRadius->value();
    int nSamples = _nFeatures->value();

    using CornerDetectorModule = HarrisCornerDetectorModule<float>;
    using MatchingModule = HungarianCornerMatchModule<float>;
    using RefinementModule = IdentityRefineInlierModule<float>;
    using FilteringModule = RansacEpipolarInlinerSelectionModule<float>;

    using MatchingPipeline = ModularSparseMatchingPipeline<float,
                                                           CornerDetectorModule,
                                                           MatchingModule,
                                                           RefinementModule,
                                                           FilteringModule>;

    MatchingPipeline matchingPipeline(new CornerDetectorModule(lpRadius, nomMaxSupprRadius, nItems),
                                      new MatchingModule(patchRadius, nSamples),
                                      nullptr,
                                      new FilteringModule(200));

    matchingPipeline.setupPipeline(_imgData1, _imgData2);

    bool verbose = true;
    matchingPipeline.runAllSteps(verbose);

    std::vector<std::array<float, 2>> const& corners1 = matchingPipeline.corners1();
    std::vector<std::array<float, 2>> const& corners2 = matchingPipeline.corners2();

    std::vector<std::array<int,2>> assignements = matchingPipeline.assignements();
    QSet<int> inliers(matchingPipeline.inliers().begin(), matchingPipeline.inliers().end());

    QVector<QPointF> points1;
    QVector<QPointF> points2;

    QVector<QColor> points1Colors;
    QVector<QColor> points2Colors;

    points1.reserve(assignements.size());
    points2.reserve(assignements.size());

    points1Colors.reserve(assignements.size());
    points2Colors.reserve(assignements.size());

    QColor inlierColor(50,255,120);
    QColor outlierColor(255,50,50);

    for (int i = 0; i < assignements.size(); i++) {
        std::array<int,2> const& assignement = assignements[i];

        int f1_idx = assignement[0];
        int f2_idx = assignement[1];

        if (f1_idx >= (int) corners1.size()) {
            continue;
        }

        if (f2_idx >= (int) corners2.size()) {
            continue;
        }

        auto& f1 = corners1[f1_idx];
        auto& f2 = corners2[f2_idx];

        points1.push_back(QPointF(f1[1], f1[0]));
        points2.push_back(QPointF(f2[1], f2[0]));

        if (inliers.contains(i)) {
            points1Colors.push_back(inlierColor);
            points2Colors.push_back(inlierColor);
        } else {
            points1Colors.push_back(outlierColor);
            points2Colors.push_back(outlierColor);
        }
    }

    _pointsOverlay1->setPointSet(points1);
    _pointsOverlay2->setPointSet(points2);

    _pointsOverlay1->setColorMap(points1Colors);
    _pointsOverlay2->setColorMap(points2Colors);

}

CornerMatchingTestEditorFactory::CornerMatchingTestEditorFactory(QObject* parent) :
    EditorFactory(parent)
{

}

QString CornerMatchingTestEditorFactory::TypeDescrName() const {
    return tr("Corner matching test editor");
}
QString CornerMatchingTestEditorFactory::itemClassName() const {
    return CornerMatchingTestEditor::staticMetaObject.className();
}
Editor* CornerMatchingTestEditorFactory::factorizeEditor(QWidget* parent) const {
    return new CornerMatchingTestEditor(parent);
}

} // namespace StereoVisionApp
