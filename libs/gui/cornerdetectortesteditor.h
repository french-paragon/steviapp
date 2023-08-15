#ifndef STEREOVISIONAPP_CORNERDETECTORTESTEDITOR_H
#define STEREOVISIONAPP_CORNERDETECTORTESTEDITOR_H

#include "./editor.h"

#include <MultidimArrays/MultidimArrays.h>
#include <LibStevi/gui/arraydisplayadapter.h>

class QSpinBox;
class QComboBox;
class QTimer;
class QLabel;

namespace StereoVisionApp {

class ImageWidget;
class PointsSetOverlay;

/*!
 * \brief The CornerDetectorTestEditor class is not an usefull editor class, rather, it is a class to test corner detections during devellopement.
 */
class CornerDetectorTestEditor : public Editor
{
    Q_OBJECT
public:
    CornerDetectorTestEditor(QWidget* parent = nullptr);

    void setImageData(Multidim::Array<float, 3> const& imgData);
    //Move variant, that might be usefull
    void setImageData(Multidim::Array<float, 3> && imgData);

    void clearImageData();

protected:

    void scheduduleRecomputation();
    void compute();

    void mouseClicked(QPoint widgetPos);

    void setDisplayAdapter();

    QTimer* _scheduler;

    ImageWidget* _imageDisplay;
    QLabel* _infosLabel;

    PointsSetOverlay* _pointsOverlay;

    QSpinBox* _lowPassRadius;
    QSpinBox* _nonMaximumMaxSuppressionRadius;
    QSpinBox* _nSelected;

    QComboBox* _displayComboBox;

    Multidim::Array<float, 3> _imgData;
    Multidim::Array<float, 2> _scoreData;

    StereoVision::Gui::ArrayDisplayAdapter<float>* _imageViewAdapter;
    StereoVision::Gui::GrayscaleArrayDisplayAdapter<float>* _scoreViewAdapter;
};

class CornerDetectorTestEditorFactory : public EditorFactory
{
    Q_OBJECT
public:

    explicit CornerDetectorTestEditorFactory(QObject* parent = nullptr);

    virtual QString TypeDescrName() const;
    virtual QString itemClassName() const;
    virtual Editor* factorizeEditor(QWidget* parent) const;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CORNERDETECTORTESTEDITOR_H
