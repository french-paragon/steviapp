#ifndef STEREOVISIONAPP_CORNERMATCHINGTESTEDITOR_H
#define STEREOVISIONAPP_CORNERMATCHINGTESTEDITOR_H

#include "./editor.h"

#include <MultidimArrays/MultidimArrays.h>
#include <StereoVision/gui/arraydisplayadapter.h>

class QSpinBox;

namespace StereoVisionApp {

class ImageWidget;
class LabelledPointsOverlay;

/*!
 * \brief The CornerMatchingTestEditor class is meant to test features matching during devellopement.
 */
class CornerMatchingTestEditor : public Editor
{
    Q_OBJECT
public:
    CornerMatchingTestEditor(QWidget* parent = nullptr);

    void setImageData(Multidim::Array<float, 3> const& imgData1,
                      Multidim::Array<float, 3> const& imgData2);
    //Move variant, that might be usefull
    void setImageData(Multidim::Array<float, 3> && imgData1,
                      Multidim::Array<float, 3> && imgData2);

    void clearImageData();

protected:

    void compute();

    void setDisplayAdapter();

    ImageWidget* _imageDisplay1;
    ImageWidget* _imageDisplay2;

    QSpinBox* _lowPassRadius;
    QSpinBox* _nonMaximumMaxSuppressionRadius;
    QSpinBox* _nSelected;

    QSpinBox* _featurePatchRadius;
    QSpinBox* _nFeatures;

    LabelledPointsOverlay* _pointsOverlay1;
    LabelledPointsOverlay* _pointsOverlay2;

    Multidim::Array<float, 3> _imgData1;
    Multidim::Array<float, 3> _imgData2;

    StereoVision::Gui::ArrayDisplayAdapter<float>* _imageViewAdapter1;
    StereoVision::Gui::ArrayDisplayAdapter<float>* _imageViewAdapter2;

    StereoVision::Gui::GrayscaleArrayDisplayAdapter<float>* _scoreViewAdapter1;
    StereoVision::Gui::GrayscaleArrayDisplayAdapter<float>* _scoreViewAdapter2;
};

class CornerMatchingTestEditorFactory : public EditorFactory
{
    Q_OBJECT
public:

    explicit CornerMatchingTestEditorFactory(QObject* parent = nullptr);

    virtual QString TypeDescrName() const;
    virtual QString itemClassName() const;
    virtual Editor* factorizeEditor(QWidget* parent) const;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CORNERMATCHINGTESTEDITOR_H
