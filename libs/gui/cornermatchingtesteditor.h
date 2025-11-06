#ifndef STEREOVISIONAPP_CORNERMATCHINGTESTEDITOR_H
#define STEREOVISIONAPP_CORNERMATCHINGTESTEDITOR_H

#include "./editor.h"

#include "../datablocks/genericcorrespondences.h"

#include "../vision/sparseMatchingPipeline.h"

#include <MultidimArrays/MultidimArrays.h>
#include <StereoVision/gui/arraydisplayadapter.h>

class QSpinBox;
class QDoubleSpinBox;
class QPushButton;

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

    /*!
     * \brief The MatchBuilder class is a strategy for the CornerMatchingTestEditor to build matches from its correspondances
     */
    class MatchBuilder {
    public:
        virtual ~MatchBuilder();
        virtual Correspondences::Generic correspondanceFromUV(float u, float v) const = 0;
        virtual QString targetTitle() const = 0;
    };

    using MatchingPipeline = ModularSparseMatchingPipeline<float,
                                                           GenericCornerDetectorModule<float>,
                                                           GenericCornerMatchModule<float>,
                                                           GenericTiePointsRenfinementModule<float>,
                                                           GenericInlinerSelectionModule<float>>;

    CornerMatchingTestEditor(QWidget* parent = nullptr);
    ~CornerMatchingTestEditor();

    void setImageData(Multidim::Array<float, 3> const& imgData1,
                      Multidim::Array<float, 3> const& imgData2);
    //Move variant, that might be usefull
    void setImageData(Multidim::Array<float, 3> && imgData1,
                      Multidim::Array<float, 3> && imgData2);

    /*!
     * \brief setMatchBuilders set the match builders for the editor
     * \param matchBuilder1 the match builder for the first image
     * \param matchBuilder2 the match builder for the second image
     */
    void setMatchBuilders(MatchBuilder* matchBuilder1, MatchBuilder* matchBuilder2);

    void clearImageData();

protected:

    void setupMatchingPipeline();

    void compute();
    void writeCorrespondanceSet();

    void setDisplayAdapter();

    ImageWidget* _imageDisplay1;
    ImageWidget* _imageDisplay2;

    QSpinBox* _lowPassRadius;
    QSpinBox* _nonMaximumMaxSuppressionRadius;
    QSpinBox* _nSelected;

    QSpinBox* _featurePatchRadius;
    QSpinBox* _nFeatures;

    QSpinBox* _nRansacIterationInput;
    QDoubleSpinBox* _ransacThreshold;

    LabelledPointsOverlay* _pointsOverlay1;
    LabelledPointsOverlay* _pointsOverlay2;

    Multidim::Array<float, 3> _imgData1;
    Multidim::Array<float, 3> _imgData2;

    MatchBuilder* _matchBuilder1;
    MatchBuilder* _matchBuilder2;

    MatchingPipeline* _matchingPipeline;

    StereoVision::Gui::ArrayDisplayAdapter<float>* _imageViewAdapter1;
    StereoVision::Gui::ArrayDisplayAdapter<float>* _imageViewAdapter2;

    StereoVision::Gui::GrayscaleArrayDisplayAdapter<float>* _scoreViewAdapter1;
    StereoVision::Gui::GrayscaleArrayDisplayAdapter<float>* _scoreViewAdapter2;

    QPushButton* _saveButton;
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
