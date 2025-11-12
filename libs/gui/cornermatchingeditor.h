#ifndef STEREOVISIONAPP_CORNERMATCHINGTESTEDITOR_H
#define STEREOVISIONAPP_CORNERMATCHINGTESTEDITOR_H

#include "./editor.h"

#include "../datablocks/genericcorrespondences.h"

#include "../vision/sparseMatchingPipeline.h"

#include <MultidimArrays/MultidimArrays.h>
#include <StereoVision/gui/arraydisplayadapter.h>

#include <QComboBox>

class QSpinBox;
class QDoubleSpinBox;
class QPushButton;
class QVBoxLayout;

namespace StereoVisionApp {

class ImageWidget;
class LabelledPointsOverlay;

/*!
 * \brief The CornerMatchingEditor class is meant to do sparse matching of features between rasters.
 */
class CornerMatchingEditor : public Editor
{
    Q_OBJECT
public:

    enum InliersFilteringMethods {
        epipolarRansac = 0,
        perspectiveRansac = 1
    };

    /*!
     * \brief The MatchBuilder class is a strategy for the CornerMatchingEditor to build matches from its correspondances
     */
    class MatchBuilder {
    public:
        virtual ~MatchBuilder();
        virtual Correspondences::Generic correspondanceFromUV(float u, float v) const = 0;
        virtual QString targetTitle() const = 0;
    };

    using ComputeType = float;

    class GenericConfigurableCornerDetectionModule {
    public:
        virtual ~GenericConfigurableCornerDetectionModule() {

        }

        virtual std::vector<std::array<float, 2>> detectCorners(Multidim::Array<ComputeType,3, Multidim::ConstView> const& img) = 0;
        virtual QWidget* getConfigurationWidget(QWidget* parent) = 0;
    };

    template <typename BaseDetector>
    class ConfigurableCornerDetectionModule : public GenericConfigurableCornerDetectionModule, public BaseDetector {
    public:
        template<typename ... T>
        ConfigurableCornerDetectionModule(T ... args) :
            BaseDetector(args...)
        {

        }
        virtual std::vector<std::array<float, 2>> detectCorners(Multidim::Array<ComputeType,3, Multidim::ConstView> const& img) override {
            return BaseDetector::detectCorners(img);
        }
    };

    class GenericConfigurableInlierSelectionModule {
    public:
        virtual ~GenericConfigurableInlierSelectionModule() {

        }

        virtual std::vector<int> getInliers(std::vector<std::array<int,2>> const& assignement,
                                            Multidim::Array<ComputeType,3, Multidim::ConstView> const& imgData1,
                                            std::vector<std::array<float, 2>> & corners1,
                                            Multidim::Array<ComputeType,3, Multidim::ConstView> const& imgData2,
                                            std::vector<std::array<float, 2>> & corners2) = 0;
        virtual QWidget* getConfigurationWidget(QWidget* parent) = 0;
    };

    template <typename BaseSelector>
    class ConfigurableInlierSelectionModule : public GenericConfigurableInlierSelectionModule, public BaseSelector {
    public:
        template<typename ... T>
        ConfigurableInlierSelectionModule(T ... args) :
            BaseSelector(args...)
        {

        }
        virtual std::vector<int> getInliers(std::vector<std::array<int,2>> const& assignement,
                                            Multidim::Array<ComputeType,3, Multidim::ConstView> const& imgData1,
                                            std::vector<std::array<float, 2>> & corners1,
                                            Multidim::Array<ComputeType,3, Multidim::ConstView> const& imgData2,
                                            std::vector<std::array<float, 2>> & corners2) {
            return BaseSelector::getInliers(assignement, imgData1, corners1, imgData2, corners2);
        }
    };

    using MatchingPipeline = ModularSparseMatchingPipeline<ComputeType,
                                                           GenericConfigurableCornerDetectionModule,
                                                           GenericCornerMatchModule<float>,
                                                           GenericTiePointsRenfinementModule<float>,
                                                           GenericConfigurableInlierSelectionModule>;

    CornerMatchingEditor(QWidget* parent = nullptr);
    ~CornerMatchingEditor();

    template<typename T>
    void addImageData(QString const& name,
                      T && imgData,
                      MatchBuilder* matchBuilder) {
        static_assert(std::is_same_v<std::remove_cv_t<std::remove_reference_t<T>>,Multidim::Array<float, 3>>, "Image data must be a raster array");

        ImageDataHolder* dataHolder = new ImageDataHolder();
        dataHolder->name = name;
        dataHolder->imgData = std::forward<T>(imgData); //perfect forwarding
        dataHolder->matchBuilder = matchBuilder;

        _availableImgsData.push_back(dataHolder);

        _leftImageSelector->addItem(name);
        _rightImageSelector->addItem(name);

        if (_nextSideToAddToIsRight) {
            _rightImageSelector->setCurrentIndex(_rightImageSelector->count()-1);
        } else {
            _leftImageSelector->setCurrentIndex(_leftImageSelector->count()-1);
        }

        _nextSideToAddToIsRight = !_nextSideToAddToIsRight;

        commitImageSelection();
    }

protected:

    void commitImageSelection();

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

    struct ImageDataHolder {
        ~ImageDataHolder();
        QString name;
        Multidim::Array<float, 3> imgData;
        MatchBuilder* matchBuilder;
    };

    void setupMatchingPipeline();

    void reconfigureCornerDetectorOptions();
    void reconfigureInlierSelectorOptions();

    void compute();
    void writeCorrespondanceSet();

    void setDisplayAdapter();

    ImageWidget* _imageDisplay1;
    ImageWidget* _imageDisplay2;

    QVBoxLayout* _cornerOptionsLayout;
    QVBoxLayout* _inlierOptionsLayout;

    QSpinBox* _featurePatchRadius;
    QSpinBox* _nFeatures;

    LabelledPointsOverlay* _pointsOverlay1;
    LabelledPointsOverlay* _pointsOverlay2;

    QComboBox* _leftImageSelector;
    QComboBox* _rightImageSelector;

    bool _nextSideToAddToIsRight;
    QVector<ImageDataHolder*> _availableImgsData;

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
