#include "cornerdetectortesteditor.h"

#include <LibStevi/sparseMatching/cornerDetectors.h>
#include <LibStevi/sparseMatching/nonLocalMaximumPointSelection.h>

#include "imagewidget.h"
#include "imagedisplayoverlays/pointssetoverlay.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>

#include <QGroupBox>
#include <QSpinBox>
#include <QComboBox>
#include <QSizePolicy>
#include <QLabel>
#include <QCheckBox>

#include <QTimer>

#include <QEvent>
#include <QMouseEvent>

namespace StereoVisionApp {

class ImageMouseMoveEventFilter : public QObject {
    Q_OBJECT
public:
    ImageMouseMoveEventFilter(QObject* parent) :
        QObject(parent)
    {

    }

Q_SIGNALS:
    void clicked(QPoint widgetPos);

protected:

    bool eventFilter(QObject *watched, QEvent *event) override {

        if (event->type() != QEvent::MouseButtonPress) {
            return false;
        }

        QMouseEvent* mEvent = dynamic_cast<QMouseEvent*>(event);

        if (mEvent->buttons() != Qt::LeftButton) {
            return false;
        }

        QPoint wpos = mEvent->pos();
        Q_EMIT clicked(wpos);

        return false;

    }


};


CornerDetectorTestEditor::CornerDetectorTestEditor(QWidget* parent):
    Editor(parent)
{

    _scheduler = new QTimer(this);
    _scheduler->setSingleShot(true);

    connect(_scheduler, &QTimer::timeout, this, &CornerDetectorTestEditor::compute);

    QVBoxLayout* verticalLayout = new QVBoxLayout();

    _imageDisplay = new ImageWidget(this);

    _imageViewAdapter = new StereoVision::Gui::ArrayDisplayAdapter<float>(&_imgData, 0, 255, 1, 0, 2, {0,1,2}, this);
    _scoreViewAdapter = new StereoVision::Gui::GrayscaleArrayDisplayAdapter<float>(&_scoreData, 0, 1, 1, 0, this);
    _scoreViewAdapter->configureOriginalChannelDisplay(tr("Harris score"));

    _pointsOverlay = new PointsSetOverlay(this);
    _pointsOverlay->setColor(QColor(255, 0, 0));
    _pointsOverlay->setRadius(3);

    _imageDisplay->setImage(_imageViewAdapter);
    _imageDisplay->addOverlay(_pointsOverlay);

    ImageMouseMoveEventFilter* eFilter = new ImageMouseMoveEventFilter(_imageDisplay);
    _imageDisplay->installEventFilter(eFilter);

    connect(eFilter, &ImageMouseMoveEventFilter::clicked, this, &CornerDetectorTestEditor::mouseClicked);

    _imageDisplay->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));

    verticalLayout->addWidget(_imageDisplay);

    _infosLabel = new QLabel(this);
    _infosLabel->setSizePolicy(QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum));

    verticalLayout->addWidget(_infosLabel);

    QHBoxLayout* optionsLayout = new QHBoxLayout();

    QGroupBox* optionsBox = new QGroupBox(this);
    optionsBox->setTitle(tr("Options"));

    QFormLayout* formLayout = new QFormLayout();

    _lowPassRadius = new QSpinBox(this);
    _lowPassRadius->setMinimum(1);
    _lowPassRadius->setMaximum(10);
    _lowPassRadius->setSuffix("px");

    _lowPassRadius->setValue(3);

    connect(_lowPassRadius, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &CornerDetectorTestEditor::scheduduleRecomputation);

    _nonMaximumMaxSuppressionRadius = new QSpinBox(this);
    _nonMaximumMaxSuppressionRadius->setMinimum(1);
    _nonMaximumMaxSuppressionRadius->setMaximum(25);
    _nonMaximumMaxSuppressionRadius->setSuffix("px");

    _nonMaximumMaxSuppressionRadius->setValue(3);

    connect(_nonMaximumMaxSuppressionRadius, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &CornerDetectorTestEditor::scheduduleRecomputation);

    _nSelected = new QSpinBox(this);
    _nSelected->setMinimum(-1);
    _nSelected->setMaximum(999999);

    _nSelected->setValue(-1);

    connect(_nSelected, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &CornerDetectorTestEditor::scheduduleRecomputation);

    formLayout->addRow(tr("Low pass radius"), _lowPassRadius);
    formLayout->addRow(tr("Non maximum suppression radius"), _nonMaximumMaxSuppressionRadius);
    formLayout->addRow(tr("Max # selected points"), _nSelected);

    optionsBox->setLayout(formLayout);

    QGroupBox* displayBox = new QGroupBox(this);
    displayBox->setTitle(tr("Display"));

    QFormLayout* formLayout2 = new QFormLayout();

    _displayComboBox = new QComboBox(this);
    _displayComboBox->addItem(tr("Image"), QVariant("image"));
    _displayComboBox->addItem(tr("Score"), QVariant("score"));

    connect(_displayComboBox, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &CornerDetectorTestEditor::setDisplayAdapter);

    formLayout2->addRow(tr("Show"), _displayComboBox);

    QCheckBox* displayPointsBox = new QCheckBox(this);
    displayPointsBox->setCheckState(Qt::Checked);
    displayPointsBox->setTristate(false);

    connect(displayPointsBox, &QCheckBox::stateChanged, this, [this] (int state) {
       if (state == Qt::Checked) {
           _pointsOverlay->show();
       } else {
           _pointsOverlay->hide();
       }
    });

    formLayout2->addRow(tr("DisplayPoints"), displayPointsBox);

    displayBox->setLayout(formLayout2);

    optionsBox->setSizePolicy(QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum));
    displayBox->setSizePolicy(QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum));

    optionsLayout->addWidget(optionsBox);
    optionsLayout->addWidget(displayBox);

    verticalLayout->addLayout(optionsLayout);

    setLayout(verticalLayout);

}

void CornerDetectorTestEditor::setImageData(Multidim::Array<float, 3> const& imgData) {
    _imgData = imgData;
    _scoreData = Multidim::Array<float, 2>();
    _pointsOverlay->setPointSet({});
    scheduduleRecomputation();
}
//Move variant, that might be usefull
void CornerDetectorTestEditor::setImageData(Multidim::Array<float, 3> && imgData) {
    _imgData = imgData;
    _scoreData = Multidim::Array<float, 2>();
    _pointsOverlay->setPointSet({});
    scheduduleRecomputation();
}

void CornerDetectorTestEditor::clearImageData() {
    _imgData = Multidim::Array<float, 3>();
    _scoreData = Multidim::Array<float, 2>();
    _pointsOverlay->setPointSet({});
    scheduduleRecomputation();
}

void CornerDetectorTestEditor::setDisplayAdapter() {
    QString type = _displayComboBox->currentData().toString();

    if (type == "score") {
        _imageDisplay->setImage(_scoreViewAdapter);
    } else {
        _imageDisplay->setImage(_imageViewAdapter);
    }
}

void CornerDetectorTestEditor::scheduduleRecomputation() {
    _scheduler->start(3000);
}
void CornerDetectorTestEditor::compute() {

    constexpr int batchDim = 2;

    if (_imgData.empty()) {
        _scoreData = Multidim::Array<float, 2>();
        _imageDisplay->update();
        return;
    }

    int lpRadius = _lowPassRadius->value();

    _scoreData = StereoVision::SparseMatching::windowedHarrisCornerScore(Multidim::Array<float, 3, Multidim::ConstView>(_imgData), lpRadius, 0, batchDim);

    float min = 0;
    float max = 0;

    if (!_scoreData.empty()) {
        min = _scoreData.atUnchecked(0,0);
        max = _scoreData.atUnchecked(0,0);
    }

    for (int i = 0; i < _scoreData.shape()[0]; i++) {
        for (int j = 0; j < _scoreData.shape()[1]; j++) {
            float val = _scoreData.atUnchecked(i,j);

            if (min > val) {
                min = val;
            }

            if (max < val) {
                max = val;
            }
        }
    }

    _scoreViewAdapter->setBWLevel(min, max);

    int nomMaxSupprRadius = _nonMaximumMaxSuppressionRadius->value();
    int nItems = _nSelected->value();

    float threshold = 0;

    std::vector<std::array<float, 2>> items =
            StereoVision::SparseMatching::nonLocalMaximumPointSelection(Multidim::Array<float, 2, Multidim::ConstView>(_scoreData), nomMaxSupprRadius, threshold, nItems);

    QVector<QPointF> points;
    points.reserve(items.size());

    for (std::array<float, 2> & item : items) {
        points.push_back(QPointF(item[1], item[0]));
    }

    _pointsOverlay->setPointSet(points);

    _imageDisplay->update();

}

void CornerDetectorTestEditor::mouseClicked(QPoint widgetPos) {

    QPointF imageCoord = _imageDisplay->widgetToImageCoordinates(widgetPos);
    QPoint pos = imageCoord.toPoint();

    if (pos.x() < 0 and pos.x() >= _imgData.shape()[1]) {
        _infosLabel->clear();
        return;
    }

    if (pos.y() < 0 and pos.y() >= _imgData.shape()[0]) {
        _infosLabel->clear();
        return;
    }

    QString tmpl = (tr("Position x:%1 y:%2 | %3 | %4"));

    auto descr = _imageViewAdapter->getOriginalChannelsInfos(pos);
    QString colorStr = "";

    if (descr.empty()) {
        QColor col = _imageDisplay->currentImage()->getColorAtPoint(pos);
        colorStr = QString("r:%1 g:%2 b:%3").arg(col.red()).arg(col.green()).arg(col.blue());
    } else {
        for (auto const& info : descr) {
            colorStr += QString("%1:%2").arg(info.channelName).arg(info.channelValue);
        }
    }

    descr = _scoreViewAdapter->getOriginalChannelsInfos(pos);
    QString scoreStr = "";

    for (auto const& info : descr) {
        scoreStr += QString("%1:%2").arg(info.channelName).arg(info.channelValue);
    }

    QString message = tmpl.arg(pos.x()).arg(pos.y()).arg(colorStr).arg(scoreStr);
    _infosLabel->setText(message);
}

CornerDetectorTestEditorFactory::CornerDetectorTestEditorFactory(QObject* parent) :
    EditorFactory(parent)
{

}

QString CornerDetectorTestEditorFactory::TypeDescrName() const {
    return tr("Corner detector test editor");
}
QString CornerDetectorTestEditorFactory::itemClassName() const {
    return CornerDetectorTestEditor::staticMetaObject.className();
}
Editor* CornerDetectorTestEditorFactory::factorizeEditor(QWidget* parent) const {
    return new CornerDetectorTestEditor(parent);
}

} // namespace StereoVisionApp

#include "cornerdetectortesteditor.moc"
