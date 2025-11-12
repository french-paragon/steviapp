#include "cornermatchingeditor.h"

#include <StereoVision/sparseMatching/cornerDetectors.h>
#include <StereoVision/sparseMatching/nonLocalMaximumPointSelection.h>
#include <StereoVision/sparseMatching/pointsDescriptors.h>
#include <StereoVision/sparseMatching/pointsOrientation.h>

#include <StereoVision/correlation/matching_costs.h>

#include <StereoVision/optimization/assignement_problems.h>

#include "imagewidget.h"

#include "gui/imagedisplayoverlays/labelledpointsoverlay.h"

#include "datablocks/correspondencesset.h"

#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QSet>
#include <QMessageBox>
#include <QCoreApplication>
#include <QComboBox>

namespace SpaMat = StereoVision::SparseMatching;
namespace Corr = StereoVision::Correlation;
namespace Optm = StereoVision::Optimization;

namespace StereoVisionApp {


CornerMatchingEditor::MatchBuilder::~MatchBuilder() {

}

using HarrisDetectorBase = CornerMatchingEditor::ConfigurableCornerDetectionModule<HarrisCornerDetectorModule<CornerMatchingEditor::ComputeType>>;
class ConfigurableHarrisCornerDetectorModule : public HarrisDetectorBase
{
    Q_DECLARE_TR_FUNCTIONS(ConfigurableHarrisCornerDetectorModule)
public:
    ConfigurableHarrisCornerDetectorModule(int lowPassRadius = 3, int nonMaximumSuppressionRadius = 3, int maxNCorners = 100) :
        HarrisDetectorBase(lowPassRadius, nonMaximumSuppressionRadius, maxNCorners)
    {

    }
    virtual QWidget* getConfigurationWidget(QWidget* parent) {

        QWidget* configWidget = new QWidget(parent);

        QFormLayout* cornerFormLayout = new QFormLayout(configWidget);

        QSpinBox* lowPassRadius = new QSpinBox(configWidget);
        lowPassRadius->setMinimum(1);
        lowPassRadius->setMaximum(10);
        lowPassRadius->setSuffix("px");

        lowPassRadius->setValue(HarrisDetectorBase::_lpRadius);

        QSpinBox* nonMaximumMaxSuppressionRadius = new QSpinBox(configWidget);
        nonMaximumMaxSuppressionRadius->setMinimum(1);
        nonMaximumMaxSuppressionRadius->setMaximum(25);
        nonMaximumMaxSuppressionRadius->setSuffix("px");

        nonMaximumMaxSuppressionRadius->setValue(HarrisDetectorBase::_nonMaxSupprRadius);

        QSpinBox* nSelected = new QSpinBox(configWidget);
        nSelected->setMinimum(-1);
        nSelected->setMaximum(999999);

        nSelected->setValue(HarrisDetectorBase::_maxNCorners);

        cornerFormLayout->addRow(tr("Low pass radius"), lowPassRadius);
        cornerFormLayout->addRow(tr("Non maximum suppression radius"), nonMaximumMaxSuppressionRadius);
        cornerFormLayout->addRow(tr("Max # selected points"), nSelected);

        QObject::connect(lowPassRadius, qOverload<int>(&QSpinBox::valueChanged), [this] (int radius) {
            setLowPassRadius(radius);
        });

        QObject::connect(nonMaximumMaxSuppressionRadius, qOverload<int>(&QSpinBox::valueChanged), [this] (int radius) {
            setNonMaxSupprRadius(radius);
        });

        QObject::connect(nSelected, qOverload<int>(&QSpinBox::valueChanged), [this] (int nCorners) {
            setMaxNCorners(nCorners);
        });

        return configWidget;

    }

protected:
};

using RansacEpipolarSelectionBase = CornerMatchingEditor::ConfigurableInlierSelectionModule<RansacEpipolarInlinerSelectionModule<CornerMatchingEditor::ComputeType>>;
class ConfigurableRansacEpipolarSelectionModule : public RansacEpipolarSelectionBase {
    Q_DECLARE_TR_FUNCTIONS(ConfigurableRansacEpipolarSelectionModule)
public:
    ConfigurableRansacEpipolarSelectionModule(int nRansacIterations = 200, float threshold = 0.02) :
        RansacEpipolarSelectionBase(nRansacIterations, threshold)
    {

    }
    virtual QWidget* getConfigurationWidget(QWidget* parent) override {

        QWidget* ret = new QWidget(parent);

        QFormLayout* inlierFormLayout = new QFormLayout(ret);

        QSpinBox* nRansacIterationInput = new QSpinBox(ret);
        nRansacIterationInput->setMinimum(1);
        nRansacIterationInput->setMaximum(999999);

        nRansacIterationInput->setValue(RansacEpipolarSelectionBase::_nIterations);

        QDoubleSpinBox* ransacThreshold = new QDoubleSpinBox(ret);
        ransacThreshold->setMinimum(0.0);
        ransacThreshold->setMaximum(999999);

        ransacThreshold->setValue(RansacEpipolarSelectionBase::_threshold);
        ransacThreshold->setSingleStep(0.01);

        inlierFormLayout->addRow(tr("Ransac iterations"), nRansacIterationInput);
        inlierFormLayout->addRow(tr("Ransac threshold"), ransacThreshold);

        QObject::connect(nRansacIterationInput, qOverload<int>(&QSpinBox::valueChanged), [this] (int n) {
            setNIterations(n);
        });

        QObject::connect(ransacThreshold, qOverload<double>(&QDoubleSpinBox::valueChanged), [this] (double threshold) {
            setThreshold(threshold);
        });

        return ret;

    }
};

using RansacPerspectiveSelectionBase = CornerMatchingEditor::ConfigurableInlierSelectionModule<RansacPerspectiveInlinerSelectionModule<CornerMatchingEditor::ComputeType>>;
class ConfigurableRansacPerspectiveSelectionModule : public RansacPerspectiveSelectionBase {
    Q_DECLARE_TR_FUNCTIONS(ConfigurableRansacEpipolarSelectionModule)
public:
    ConfigurableRansacPerspectiveSelectionModule(int nRansacIterations = 200, float threshold = 5) :
        RansacPerspectiveSelectionBase(nRansacIterations, threshold)
    {

    }
    virtual QWidget* getConfigurationWidget(QWidget* parent) override {

        QWidget* ret = new QWidget(parent);

        QFormLayout* inlierFormLayout = new QFormLayout(ret);

        QSpinBox* nRansacIterationInput = new QSpinBox(ret);
        nRansacIterationInput->setMinimum(1);
        nRansacIterationInput->setMaximum(999999);

        nRansacIterationInput->setValue(RansacPerspectiveSelectionBase::_nIterations);

        QDoubleSpinBox* ransacThreshold = new QDoubleSpinBox(ret);
        ransacThreshold->setMinimum(0.0);
        ransacThreshold->setMaximum(999999);

        ransacThreshold->setValue(RansacPerspectiveSelectionBase::_threshold);
        ransacThreshold->setSingleStep(0.5);
        ransacThreshold->setSuffix("px");

        inlierFormLayout->addRow(tr("Ransac iterations"), nRansacIterationInput);
        inlierFormLayout->addRow(tr("Ransac threshold"), ransacThreshold);

        QObject::connect(nRansacIterationInput, qOverload<int>(&QSpinBox::valueChanged), [this] (int n) {
            setNIterations(n);
        });

        QObject::connect(ransacThreshold, qOverload<double>(&QDoubleSpinBox::valueChanged), [this] (double threshold) {
            setThreshold(threshold);
        });

        return ret;

    }
};

CornerMatchingEditor::CornerMatchingEditor(QWidget *parent) :
    Editor(parent),
    _nextSideToAddToIsRight(false),
    _matchBuilder1(nullptr),
    _matchBuilder2(nullptr)
{

    QVBoxLayout* verticalLayout = new QVBoxLayout();

    _leftImageSelector = new QComboBox(this);
    _rightImageSelector = new QComboBox(this);

    QHBoxLayout* comboBoxesLayout = new QHBoxLayout();

    connect(_leftImageSelector, qOverload<int>(&QComboBox::currentIndexChanged),
            this, &CornerMatchingEditor::commitImageSelection);
    connect(_rightImageSelector, qOverload<int>(&QComboBox::currentIndexChanged),
            this, &CornerMatchingEditor::commitImageSelection);

    comboBoxesLayout->addWidget(_leftImageSelector);
    comboBoxesLayout->addWidget(_rightImageSelector);

    verticalLayout->addLayout(comboBoxesLayout);

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

    _cornerOptionsLayout = new QVBoxLayout();

    cornerOptionsBox->setLayout(_cornerOptionsLayout);

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

    QGroupBox* inliersOptionsBox = new QGroupBox(this);
    inliersOptionsBox->setTitle(tr("Inliers filtering options"));

    _inlierOptionsLayout = new QVBoxLayout(inliersOptionsBox);

    QComboBox* inlierSelectionMethodBox = new QComboBox(this);

    inlierSelectionMethodBox->addItem(tr("Epipolar Ransac"), static_cast<int>(epipolarRansac));
    inlierSelectionMethodBox->addItem(tr("Perspective Ransac"), static_cast<int>(perspectiveRansac));

    connect(inlierSelectionMethodBox, qOverload<int>(&QComboBox::currentIndexChanged), this, [this, inlierSelectionMethodBox] (int idx) {
        int data = inlierSelectionMethodBox->itemData(idx).toInt();

        switch (data) {
        case epipolarRansac:{
            ConfigurableRansacEpipolarSelectionModule* baseInlierModule = new ConfigurableRansacEpipolarSelectionModule();
            _matchingPipeline->setInlierModule(baseInlierModule);
        }
        break;
        case perspectiveRansac:{
            ConfigurableRansacPerspectiveSelectionModule* baseInlierModule = new ConfigurableRansacPerspectiveSelectionModule();
            _matchingPipeline->setInlierModule(baseInlierModule);
        }
        break;
        }
        reconfigureInlierSelectorOptions();
    });

    _inlierOptionsLayout->addWidget(inlierSelectionMethodBox);

    optionsLayout->addWidget(inliersOptionsBox);

    verticalLayout->addLayout(optionsLayout);

    QHBoxLayout* buttonLayout = new QHBoxLayout();

    QPushButton* computeButton = new QPushButton(this);
    computeButton->setText(tr("Compute"));

    _saveButton = new QPushButton(this);
    _saveButton->setText(tr("Write matches"));

    _saveButton->setEnabled(false);

    connect(computeButton, &QPushButton::clicked, this, &CornerMatchingEditor::compute);
    connect(_saveButton, &QPushButton::clicked, this, &CornerMatchingEditor::writeCorrespondanceSet);

    buttonLayout->addStretch();
    buttonLayout->addWidget(_saveButton);
    buttonLayout->addWidget(computeButton);

    verticalLayout->addLayout(buttonLayout);

    setLayout(verticalLayout);

    //special function to setup matching pipeline, clearner.
    setupMatchingPipeline();

}
CornerMatchingEditor::~CornerMatchingEditor() {

    if (_matchingPipeline != nullptr) {
        delete _matchingPipeline;
    }

    for (ImageDataHolder* imageDataHolder : _availableImgsData) {
        if (imageDataHolder != nullptr) {
            delete imageDataHolder;
        }
    }
}
CornerMatchingEditor::ImageDataHolder::~ImageDataHolder() {
    if (matchBuilder != nullptr) {
        delete matchBuilder;
    }
}

void CornerMatchingEditor::commitImageSelection() {

    int leftIdx = _leftImageSelector->currentIndex();
    int rightIdx = _rightImageSelector->currentIndex();

    if (leftIdx == rightIdx) {
        return;
    }

    if (leftIdx < 0) {
        return;
    }

    if (rightIdx < 0) {
        return;
    }

    setImageData(_availableImgsData[leftIdx]->imgData, _availableImgsData[rightIdx]->imgData);
    setMatchBuilders(_availableImgsData[leftIdx]->matchBuilder, _availableImgsData[rightIdx]->matchBuilder);

}

void CornerMatchingEditor::setImageData(Multidim::Array<float, 3> const& imgData1,
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

void CornerMatchingEditor::setImageData(Multidim::Array<float, 3> && imgData1,
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
void CornerMatchingEditor::setMatchBuilders(MatchBuilder* matchBuilder1,
                                                MatchBuilder* matchBuilder2) {

    _matchBuilder1 = matchBuilder1;
    _matchBuilder2 = matchBuilder2;
}

void CornerMatchingEditor::clearImageData() {

    _imgData1 = Multidim::Array<float, 3>();
    _imgData2 = Multidim::Array<float, 3>();

    _pointsOverlay1->setPointSet({});
    _pointsOverlay2->setPointSet({});

    _imageDisplay1->update();
    _imageDisplay2->update();
}

void CornerMatchingEditor::setupMatchingPipeline() {

    int lpRadius = 3;
    int nomMaxSupprRadius = 3;
    int nItems = 100;

    int patchRadius = _featurePatchRadius->value();
    int nSamples = _nFeatures->value();

    int nRansacIteration = 200;
    float threshold = 0.1;

    GenericConfigurableCornerDetectionModule* baseCornerDetectionModule = new ConfigurableHarrisCornerDetectorModule(lpRadius, nomMaxSupprRadius, nItems);
    HungarianCornerMatchModule<float>* basePointsMatchingModule = new HungarianCornerMatchModule<float>(patchRadius, nSamples);
    ConfigurableRansacEpipolarSelectionModule* baseInlierModule = new ConfigurableRansacEpipolarSelectionModule(nRansacIteration, threshold);

    _matchingPipeline = new MatchingPipeline(baseCornerDetectionModule, basePointsMatchingModule, nullptr, baseInlierModule);

    //edit modules values


    connect(_featurePatchRadius, qOverload<int>(&QSpinBox::valueChanged), this, [basePointsMatchingModule] (int val) {
        basePointsMatchingModule->setPatchRadius(val);
    });

    connect(_nFeatures, qOverload<int>(&QSpinBox::valueChanged), this, [basePointsMatchingModule] (int val) {
        basePointsMatchingModule->setNSamples(val);
    });

    reconfigureCornerDetectorOptions();
    reconfigureInlierSelectorOptions();

}


void CornerMatchingEditor::reconfigureCornerDetectorOptions() {
    if (_cornerOptionsLayout == nullptr) {
        return;
    }

    //clear the element, if an element is present
    QLayoutItem *item = _cornerOptionsLayout->takeAt(0);

    if (item != nullptr) {
        if (item->widget()) {
            delete item->widget();
        }
        delete item;
    }

    QWidget* configWidget = _matchingPipeline->cornerModule()->getConfigurationWidget(this);

    _cornerOptionsLayout->addWidget(configWidget);
}

void CornerMatchingEditor::reconfigureInlierSelectorOptions() {
    if (_inlierOptionsLayout == nullptr) {
        return;
    }

    //clear the element, if an element is present
    if (_inlierOptionsLayout->count() > 1) { //configurartion widget has been added
        QLayoutItem *item = _inlierOptionsLayout->takeAt(1);

        if (item != nullptr) {
            if (item->widget()) {
                delete item->widget();
            }
            delete item;
        }
    }

    QWidget* configWidget = _matchingPipeline->inlierModule()->getConfigurationWidget(this);

    _inlierOptionsLayout->addWidget(configWidget);

}

void CornerMatchingEditor::compute() {

    _saveButton->setEnabled(false);

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

    _matchingPipeline->setupPipeline(_imgData1, _imgData2);

    bool verbose = true;
    _matchingPipeline->runAllSteps(verbose);

    std::vector<std::array<float, 2>> const& corners1 = _matchingPipeline->corners1();
    std::vector<std::array<float, 2>> const& corners2 = _matchingPipeline->corners2();

    std::vector<std::array<int,2>> assignements = _matchingPipeline->assignements();
    QSet<int> inliers(_matchingPipeline->inliers().begin(), _matchingPipeline->inliers().end());

    QVector<QPointF> points1;
    QVector<QPointF> points2;

    QVector<QColor> points1Colors;
    QVector<QColor> points2Colors;

    points1.reserve(assignements.size());
    points2.reserve(assignements.size());

    points1Colors.reserve(assignements.size());
    points2Colors.reserve(assignements.size());

    QColor inlierColor(10,150,30);
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

    if (_matchBuilder1 != nullptr and _matchBuilder2 != nullptr) {
        _saveButton->setEnabled(true);
    }

}

void CornerMatchingEditor::writeCorrespondanceSet() {

    if (_matchBuilder1 == nullptr or _matchBuilder2 == nullptr) {
        return;
    }

    Project* project = activeProject();

    if (project == nullptr) {
        QMessageBox::warning(this,
                             QObject::tr("Could not import correspondences"),
                             QObject::tr("Could not get project to create datablock!"));
        return;
    }

    CorrespondencesSet* correspSet = nullptr;

    if (correspSet == nullptr) {
        qint64 id = project->createDataBlock(CorrespondencesSet::staticMetaObject.className());

        if (id < 0) {
            QMessageBox::warning(this,
                                 QObject::tr("Could not import correspondences"),
                                 QObject::tr("Could not create datablock in project!"));
            return ;
        }

        correspSet = project->getDataBlock<CorrespondencesSet>(id);

        if (correspSet == nullptr) {
            QMessageBox::warning(this,
                                 QObject::tr("Could not import correspondences"),
                                 QObject::tr("Could not load created datablock from project!"));
            return;
        }

        correspSet->setObjectName(QString("%1 - %2 correspondence set").arg(_matchBuilder1->targetTitle(), _matchBuilder2->targetTitle()));
    }

    std::vector<std::array<float, 2>> const& corners1 = _matchingPipeline->corners1();
    std::vector<std::array<float, 2>> const& corners2 = _matchingPipeline->corners2();

    std::vector<std::array<int,2>> assignements = _matchingPipeline->assignements();
    QSet<int> inliers(_matchingPipeline->inliers().begin(), _matchingPipeline->inliers().end());


    for (int i = 0; i < assignements.size(); i++) {
        if (!inliers.contains(i)) {
            continue;
        }

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

        float u1 = f1[0];
        float v1 = f1[1];

        float u2 = f2[0];
        float v2 = f2[1];

        correspSet->addCorrespondence(Correspondences::GenericPair{_matchBuilder1->correspondanceFromUV(u1,v1),
                                                                   _matchBuilder2->correspondanceFromUV(u2,v2)});
    }

}

CornerMatchingTestEditorFactory::CornerMatchingTestEditorFactory(QObject* parent) :
    EditorFactory(parent)
{

}

QString CornerMatchingTestEditorFactory::TypeDescrName() const {
    return tr("Corner matching test editor");
}
QString CornerMatchingTestEditorFactory::itemClassName() const {
    return CornerMatchingEditor::staticMetaObject.className();
}
Editor* CornerMatchingTestEditorFactory::factorizeEditor(QWidget* parent) const {
    return new CornerMatchingEditor(parent);
}

} // namespace StereoVisionApp
