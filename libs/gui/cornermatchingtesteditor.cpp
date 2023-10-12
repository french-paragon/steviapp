#include "cornermatchingtesteditor.h"

#include <LibStevi/sparseMatching/cornerDetectors.h>
#include <LibStevi/sparseMatching/nonLocalMaximumPointSelection.h>
#include <LibStevi/sparseMatching/pointsDescriptors.h>
#include <LibStevi/sparseMatching/pointsOrientation.h>

#include <LibStevi/correlation/matching_costs.h>

#include <LibStevi/optimization/assignement_problems.h>

#include "imagewidget.h"

#include "gui/imagedisplayoverlays/labelledpointsoverlay.h"

#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QSpinBox>
#include <QPushButton>

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

    _imageViewAdapter1->update();
    _imageViewAdapter2->update();

    _imageDisplay1->update();
    _imageDisplay2->update();
}

void CornerMatchingTestEditor::setImageData(Multidim::Array<float, 3> && imgData1,
                                            Multidim::Array<float, 3> && imgData2) {
    _imgData1 = imgData1;
    _imgData2 = imgData2;

    _pointsOverlay1->setPointSet({});
    _pointsOverlay2->setPointSet({});

    _imageViewAdapter1->update();
    _imageViewAdapter2->update();

    _imageDisplay1->update();
    _imageDisplay2->update();

}

void CornerMatchingTestEditor::clearImageData() {

    _imgData1 = Multidim::Array<float, 3>();
    _imgData2 = Multidim::Array<float, 3>();

    _pointsOverlay1->setPointSet({});
    _pointsOverlay2->setPointSet({});

    _imageViewAdapter1->update();
    _imageViewAdapter2->update();

    _imageDisplay1->update();
    _imageDisplay2->update();
}

void CornerMatchingTestEditor::compute() {

    constexpr int batchDim = 2;
    constexpr int nDim = 3;

    if (_imgData1.empty()) {

        _imageViewAdapter1->update();
        _imageViewAdapter2->update();

        _imageDisplay1->update();
        _imageDisplay2->update();

        return;
    }

    if (_imgData2.empty()) {

        _imageViewAdapter1->update();
        _imageViewAdapter2->update();

        _imageDisplay1->update();
        _imageDisplay2->update();

        return;
    }

    int lpRadius = _lowPassRadius->value();

    Multidim::Array<float, 2> scoreData1;
    Multidim::Array<float, 2> scoreData2;

    scoreData1 = SpaMat::windowedHarrisCornerScore(Multidim::Array<float, 3, Multidim::ConstView>(_imgData1), lpRadius, 0, batchDim);
    scoreData2 = SpaMat::windowedHarrisCornerScore(Multidim::Array<float, 3, Multidim::ConstView>(_imgData2), lpRadius, 0, batchDim);

    int nomMaxSupprRadius = _nonMaximumMaxSuppressionRadius->value();
    int nItems = _nSelected->value();

    float threshold = 0;

    std::vector<std::array<float, 2>> corners1 =
            SpaMat::nonLocalMaximumPointSelection(Multidim::Array<float, 2, Multidim::ConstView>(scoreData1),
                                                  nomMaxSupprRadius,
                                                  threshold,
                                                  nItems);

    std::vector<std::array<float, 2>> corners2 =
            SpaMat::nonLocalMaximumPointSelection(Multidim::Array<float, 2, Multidim::ConstView>(scoreData2),
                                                  nomMaxSupprRadius,
                                                  threshold,
                                                  nItems);


    std::vector<std::array<int, 2>> integral_corners1(corners1.size());
    std::vector<std::array<int, 2>> integral_corners2(corners2.size());

    for (int i = 0; i < corners1.size(); i++) {
        std::array<float, 2> pt = corners1[i];
        integral_corners1[i] = std::array<int, 2>{static_cast<int>(std::round(pt[0])), static_cast<int>(std::round(pt[1]))};
    }

    for (int i = 0; i < corners2.size(); i++) {
        std::array<float, 2> pt = corners2[i];
        integral_corners2[i] = std::array<int, 2>{static_cast<int>(std::round(pt[0])), static_cast<int>(std::round(pt[1]))};
    }

    int patchRadius = _featurePatchRadius->value();

    std::vector<SpaMat::orientedCoordinate<2>> oriented1 = SpaMat::intensityOrientedCoordinates<true>(integral_corners1, _imgData1, patchRadius, batchDim);
    std::vector<SpaMat::orientedCoordinate<2>> oriented2 = SpaMat::intensityOrientedCoordinates<true>(integral_corners2, _imgData1, patchRadius, batchDim);

    int nSamples = _nFeatures->value();

    std::array<int,nDim> shape;
    std::fill(shape.begin(), shape.end(), 2*patchRadius+1);
    shape[batchDim] = 3;

    std::vector<std::array<int,nDim>> patchCoords = SpaMat::generateDensePatchCoordinates<nDim>(shape);


    auto features1 = SpaMat::WhitenedPixelsDescriptor(oriented1, _imgData1, patchCoords, batchDim);
    auto features2 = SpaMat::WhitenedPixelsDescriptor(oriented2, _imgData2, patchCoords, batchDim);

    int n = features1.size();
    int m = features2.size();

    Eigen::MatrixXf costs;
    costs.resize(n, m);

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            costs(i,j) = Corr::SumAbsDiff(features1[i].features, features2[j].features);
        }
    }

    std::vector<std::array<int,2>> assignements = Optm::optimalAssignement(costs);

    QVector<QPointF> points1;
    QVector<QPointF> points2;

    points1.reserve(assignements.size());
    points2.reserve(assignements.size());

    for (std::array<int,2> const& assignement : assignements) {
        int f1_idx = assignement[0];
        int f2_idx = assignement[0];

        auto& f1 = features1[f1_idx];
        auto& f2 = features2[f2_idx];

        points1.push_back(QPointF(f1.coord[1], f1.coord[0]));
        points2.push_back(QPointF(f2.coord[1], f2.coord[0]));
    }

    _pointsOverlay1->setPointSet(points1);
    _pointsOverlay2->setPointSet(points2);

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
