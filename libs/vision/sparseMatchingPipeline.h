#ifndef SPARSEMATCHINGPIPELINE_H
#define SPARSEMATCHINGPIPELINE_H

#include <MultidimArrays/MultidimArrays.h>

#include <QString>
#include <QTextStream>
#include <vector>

#include <StereoVision/sparseMatching/cornerDetectors.h>
#include <StereoVision/sparseMatching/nonLocalMaximumPointSelection.h>
#include <StereoVision/sparseMatching/pointsDescriptors.h>
#include <StereoVision/sparseMatching/pointsOrientation.h>

#include <StereoVision/correlation/matching_costs.h>

#include <StereoVision/optimization/assignement_problems.h>
#include <StereoVision/optimization/generic_ransac.h>

#include <StereoVision/geometry/alignement.h>

#include "../datablocks/genericcorrespondences.h"

namespace StereoVisionApp {

template<typename T>
class SparseMatchingPipeline {
public:

    enum Steps {
        Initialized = 0, //ready to run, nothing has been processed yet
        CornersDetected = 1,
        CornersMatched = 2,
        CornersRefined = 3,
        InliersFiltered = 4
    };

    virtual ~SparseMatchingPipeline() {

    }

    void setupPipeline(Multidim::Array<T,2> const& img1Data, Multidim::Array<T,2> const& img2Data) {
        _lastErrorMessage.clear();
        _currentStep = Initialized;

        _corners1.clear();
        _corners2.clear();

        _assignements.clear();
        _inliers.clear();

        typename Multidim::Array<T,3, Multidim::ConstView>::ShapeBlock img1Shape{img1Data.shape()[0], img1Data.shape()[1], 1};
        typename Multidim::Array<T,3, Multidim::ConstView>::ShapeBlock img1Strides{img1Data.strides()[0], img1Data.strides()[1], 1};
        _img1View = img1Data.template buildReshapedView<3>(img1Shape, img1Strides);

        typename Multidim::Array<T,3, Multidim::ConstView>::ShapeBlock img2Shape{img2Data.shape()[0], img2Data.shape()[1], 1};
        typename Multidim::Array<T,3, Multidim::ConstView>::ShapeBlock img2Strides{img2Data.strides()[0], img2Data.strides()[1], 1};
        _img2View = img2Data.template buildReshapedView<3>(img2Shape, img2Strides);
    }
    void setupPipeline(Multidim::Array<T,3> const& img1Data, Multidim::Array<T,3> const& img2Data) {
        _lastErrorMessage.clear();
        _currentStep = Initialized;

        _img1View = img1Data.template buildReshapedView<3>(img1Data.shape(), img1Data.strides());

        _img2View = img2Data.template buildReshapedView<3>(img2Data.shape(), img2Data.strides());

    }

    bool detectCorners() {
        _lastErrorMessage = "unknown error in detect corners step";

        if (_currentStep != Initialized) {
            if (_currentStep >= CornersDetected) {
                return true; //corners already detected;
            } else { //should not occur
                _lastErrorMessage = "";
                return false;
            }
        }

        bool ok = detectCornersImpl();

        if (!ok) {
            return false;
        }
        _lastErrorMessage.clear();

        _currentStep = CornersDetected;
        return true;
    }
    bool matchCorners() {

        _lastErrorMessage = "unknown error in match corners step";

        if (_currentStep != CornersDetected) {
            if (_currentStep >= CornersMatched) {
                return true; //corners already matched;
            } else {
                _lastErrorMessage = "matchCorners called at wrong step in sparse matching pipeline";
                return false;
            }
        }

        bool ok = matchCornersImpl();

        if (!ok) {
            return false;
        }
        _lastErrorMessage.clear();

        _currentStep = CornersMatched;
        return true;

    }
    bool refineMatchedCorners() {

        _lastErrorMessage = "unknown error in refine matched corners step";

        if (_currentStep != CornersMatched) {
            if (_currentStep >= CornersRefined) {
                return true; //corners already matched;
            } else {
                _lastErrorMessage = "refineMatchedCorners called at wrong step in sparse matching pipeline";
                return false;
            }
        }

        bool ok = refineMatchedCornersImpl();

        if (!ok) {
            return false;
        }
        _lastErrorMessage.clear();

        _currentStep = CornersRefined;
        return true;
    }
    bool filterInliers() {

        _lastErrorMessage = "unknown error in filter inliers step";

        if (_currentStep != CornersRefined) {
            if (_currentStep >= InliersFiltered) {
                return true; //should not happed;
            } else {
                _lastErrorMessage = "filterInliers called at wrong step in sparse matching pipeline";
                return false;
            }
        }

        bool ok = filterInliersImpl();

        if (!ok) {
            return false;
        }
        _lastErrorMessage.clear();

        _currentStep = InliersFiltered;
        return true;

    }

    bool runNextSteps() {
        bool ok = true;

        if (_currentStep < CornersDetected) {
            ok = detectCorners();
            if (!ok) {
                return false;
            }
            return true;
        }

        if (_currentStep < CornersMatched) {
            ok = matchCorners();
            if (!ok) {
                return false;
            }
            return true;
        }

        if (_currentStep < CornersRefined) {
            ok = refineMatchedCorners();
            if (!ok) {
                return false;
            }
            return true;
        }

        if (_currentStep < InliersFiltered) {
            ok = filterInliers();
            if (!ok) {
                return false;
            }
            return true;
        }

        return true;
    }

    bool runAllSteps(bool verbose = false) {
        bool ok = true;

        QTextStream out(stdout);

        if (verbose) {
            out << "Start sparse matching pipeline! " << Qt::endl;
        }

        if (_currentStep < CornersDetected) {
            ok = detectCorners();
            if (!ok) {
                return false;
            }
        }

        if (verbose) {
            out << "Finised corner detection step !" << "\n" <<
                "\tDetected " << _corners1.size() << " corners in image 1 and " << _corners2.size() << " corners in image 2!" << Qt::endl;
        }

        if (_currentStep < CornersMatched) {
            ok = matchCorners();
            if (!ok) {
                return false;
            }
        }

        if (verbose) {
            out << "Finised corner matching step !" << "\n" <<
                "\tMatched " << _assignements.size() << " corners pairs!" << Qt::endl;
        }

        if (_currentStep < CornersRefined) {
            ok = refineMatchedCorners();
            if (!ok) {
                return false;
            }
        }

        if (verbose) {
            out << "Finised matching refinement step !" << Qt::endl;
        }

        if (_currentStep < InliersFiltered) {
            ok = filterInliers();
            if (!ok) {
                return false;
            }
        }

        if (verbose) {
            out << "Finised inlier detection step !" << "\n" <<
                "\tSelected " << _inliers.size() << " corners pairs!" << Qt::endl;
        }


        return true;
    }

    inline QString lastErrorMessage() const {
        return _lastErrorMessage;
    }

    std::vector<std::array<float, 2>> const& corners1() const { return _corners1; }
    std::vector<std::array<float, 2>> const& corners2() const { return _corners2; }

    std::vector<std::array<int,2>> const& assignements() const { return _assignements; }
    std::vector<int> const& inliers() const { return _inliers; }

    std::pair<std::vector<std::array<float, 2>>,std::vector<std::array<float, 2>>> filteredCorners() const {
        std::vector<std::array<float, 2>> uvs1(_inliers.size());
        std::vector<std::array<float, 2>> uvs2(_inliers.size());

        for (int i = 0; i < _inliers.size(); i++) {

            std::array<int,2> const& assignement = _assignements[i];

            int f1_idx = assignement[0];
            int f2_idx = assignement[1];

            if (f1_idx >= (int) _corners1.size()) {
                continue;
            }

            if (f2_idx >= (int) _corners2.size()) {
                continue;
            }

            auto& f1 = _corners1[f1_idx];
            auto& f2 = _corners2[f2_idx];

            float u1 = f1[0];
            float v1 = f1[1];

            float u2 = f2[0];
            float v2 = f2[1];

            uvs1[i] = {u1,v1};
            uvs2[i] = {u2,v2};

        }

        return std::make_pair(uvs1, uvs2);
    }

protected:

    virtual bool detectCornersImpl() = 0;
    virtual bool matchCornersImpl() = 0;
    virtual bool refineMatchedCornersImpl() = 0;
    virtual bool filterInliersImpl() = 0;

    std::vector<std::array<float, 2>> _corners1;
    std::vector<std::array<float, 2>> _corners2;

    std::vector<std::array<int,2>> _assignements;
    std::vector<int> _inliers;

    Steps _currentStep;

    QString _lastErrorMessage;

    Multidim::Array<T,3, Multidim::ConstView> _img1View;
    Multidim::Array<T,3, Multidim::ConstView> _img2View;

};

/*!
 * \brief The ModularSparseMatchingPipeline class represent a full sparse matching pipeline made up of modules (strategy pattern).
 *
 * The class is mostly an holder for different expected modules.
 *
 * The first module, CornerModuleT, will provide a set of detected corner/points of interest. This one is mandatory.
 * The second module, MatcherModuleT, will match the points between both images. It is mandatory.
 * The third module, RefineModuleT, will try to refine the coordinated of joint corners, trying to improve their alignment.
 * The last module, InlierModuleT, will filter out outlier matches, e.g. using some form of ransac
 *
 * The pipeline is constructed using the different modules and then setup with images. It only keeps reference to the images
 * data so the original image data should be kept in memory for the whole processing pipeline.
 */
template<typename T,
         typename CornerModuleT,
         typename MatcherModuleT,
         typename RefineModuleT,
         typename InlierModuleT>
class ModularSparseMatchingPipeline : public SparseMatchingPipeline<T> {

public:

    ModularSparseMatchingPipeline(CornerModuleT* cornerModule,
                                  MatcherModuleT* matcherModule,
                                  RefineModuleT* refineModule,
                                  InlierModuleT* inlierModule) :
        _cornerModule(cornerModule),
        _matcherModule(matcherModule),
        _refineModule(refineModule),
        _inlierModule(inlierModule)
    {

    }

    virtual ~ModularSparseMatchingPipeline() {

        if (_cornerModule != nullptr) {
            delete _cornerModule;
        }

        if (_matcherModule != nullptr) {
            delete _matcherModule;
        }

        if (_refineModule != nullptr) {
            delete _refineModule;
        }

        if (_inlierModule != nullptr) {
            delete _inlierModule;
        }

    }

    inline CornerModuleT* cornerModule() {
        return _cornerModule;
    }

    inline MatcherModuleT* matcherModule() {
        return _matcherModule;
    }

    inline RefineModuleT* refineModule() {
        return _refineModule;
    }

    inline InlierModuleT* inlierModule() {
        return _inlierModule;
    }

    inline CornerModuleT* takeCornerModule() {
        CornerModuleT* ret = _cornerModule;
        _cornerModule = nullptr;
        return ret;
    }

    inline MatcherModuleT* takeMatcherModule() {
        InlierModuleT* ret = _matcherModule;
        _matcherModule = nullptr;
        return ret;
    }

    inline RefineModuleT* takeRefineModule() {
        RefineModuleT* ret = _refineModule;
        _refineModule = nullptr;
        return ret;
    }

    inline InlierModuleT* takeInlierModule() {
        InlierModuleT* ret = _inlierModule;
        _inlierModule = nullptr;
        return ret;
    }

    void setCornerModule(CornerModuleT* newCornerModule) {
        if (_cornerModule != nullptr) {
            delete _cornerModule;
        }
        _cornerModule = newCornerModule;
    }

    void setMatcherModule(MatcherModuleT* newMatcherModule) {
        if (_matcherModule != nullptr) {
            delete _matcherModule;
        }
        _matcherModule = newMatcherModule;
    }

    void setRefineModule(RefineModuleT* newRefineModule) {
        if (_refineModule != nullptr) {
            delete _refineModule;
        }
        _refineModule = newRefineModule;
    }

    void setInlierModule(InlierModuleT* newInlierModule) {
        if (_inlierModule != nullptr) {
            delete _inlierModule;
        }
        _inlierModule = newInlierModule;
    }

protected:

    bool detectCornersImpl() override {
        SparseMatchingPipeline<T>::_corners1 = _cornerModule->detectCorners(SparseMatchingPipeline<T>::_img1View);
        SparseMatchingPipeline<T>::_corners2 = _cornerModule->detectCorners(SparseMatchingPipeline<T>::_img2View);
        return true;
    }
    bool matchCornersImpl() override {
        SparseMatchingPipeline<T>::_assignements =
            _matcherModule->matchCorners(
                SparseMatchingPipeline<T>::_img1View,
                SparseMatchingPipeline<T>::_corners1,
                SparseMatchingPipeline<T>::_img2View,
                SparseMatchingPipeline<T>::_corners2);
        return true;
    }
    bool refineMatchedCornersImpl() override {
        if (_refineModule == nullptr) {
            return true; //refine module is not mandatory
        }
        bool ok =
            _refineModule->refineMatchedPositions(
                SparseMatchingPipeline<T>::_assignements,
                SparseMatchingPipeline<T>::_img1View,
                SparseMatchingPipeline<T>::_corners1,
                SparseMatchingPipeline<T>::_img2View,
                SparseMatchingPipeline<T>::_corners2);

        return ok;
    }
    bool filterInliersImpl() override {
        SparseMatchingPipeline<T>::_inliers =
            _inlierModule->getInliers(
                SparseMatchingPipeline<T>::_assignements,
                SparseMatchingPipeline<T>::_img1View,
                SparseMatchingPipeline<T>::_corners1,
                SparseMatchingPipeline<T>::_img2View,
                SparseMatchingPipeline<T>::_corners2);
        return true;
    }

    CornerModuleT* _cornerModule;
    MatcherModuleT* _matcherModule;
    RefineModuleT* _refineModule;
    InlierModuleT* _inlierModule;
};

template<typename T>
class IdentityRefineInlierModule {
public:
    bool refineMatchedPositions(std::vector<std::array<int,2>> const& assignement,
                                Multidim::Array<T,3, Multidim::ConstView> const& imgData1,
                                std::vector<std::array<float, 2>> & corners1,
                                Multidim::Array<T,3, Multidim::ConstView> const& imgData2,
                                std::vector<std::array<float, 2>> & corners2) {
        Q_UNUSED(assignement);
        Q_UNUSED(imgData1);
        Q_UNUSED(corners1);
        Q_UNUSED(imgData2);
        Q_UNUSED(corners2);
        return true;
    }
    std::vector<int> getInliers(std::vector<std::array<int,2>> const& assignement,
                                Multidim::Array<T,3, Multidim::ConstView> const& imgData1,
                                std::vector<std::array<float, 2>> & corners1,
                                Multidim::Array<T,3, Multidim::ConstView> const& imgData2,
                                std::vector<std::array<float, 2>> & corners2) {
        Q_UNUSED(imgData1);
        Q_UNUSED(corners1);
        Q_UNUSED(imgData2);
        Q_UNUSED(corners2);

        std::vector<int> ret(assignement.size());
        for (int i = 0; i < assignement.size(); i++) {
            ret[i] = i;
        }

        return ret;
    }
};

template<typename T>
class GenericCornerDetectorModule { //class used for polymorphic modules
public:

    virtual ~GenericCornerDetectorModule() {

    }

    virtual std::vector<std::array<float, 2>> detectCorners(Multidim::Array<T,3, Multidim::ConstView> const& img) = 0;
};

template<typename T>
class HarrisCornerDetectorModule : public GenericCornerDetectorModule<T> {
public:

    static constexpr int batchDim = 2;

    HarrisCornerDetectorModule(int lowPassRadius, int nonMaximumSuppressionRadius, int maxNCorners) :
        _lpRadius(lowPassRadius),
        _nonMaxSupprRadius(nonMaximumSuppressionRadius),
        _maxNCorners(maxNCorners)
    {

    }

    std::vector<std::array<float, 2>> detectCorners(Multidim::Array<T,3, Multidim::ConstView> const& img) override {

        Multidim::Array<float, 2> scoreData = StereoVision::SparseMatching::windowedHarrisCornerScore(img, _lpRadius, 0, batchDim);

        float threshold = 0;

        std::vector<std::array<float, 2>> corners =
            StereoVision::SparseMatching::nonLocalMaximumPointSelection(Multidim::Array<float, 2, Multidim::ConstView>(scoreData),
                                                  _nonMaxSupprRadius,
                                                  threshold,
                                                  _maxNCorners);

        return corners;
    }

    inline void setLowPassRadius(int lpRadius) {
        _lpRadius = lpRadius;
    }

    inline void setNonMaxSupprRadius(int radius) {
        _nonMaxSupprRadius = radius;
    }

    inline void setMaxNCorners(int nCorners) {
        _maxNCorners = nCorners;
    }

protected:

    int _lpRadius;
    int _nonMaxSupprRadius;
    int _maxNCorners;
};

template<typename T>
class GenericCornerMatchModule {
public:

    virtual ~GenericCornerMatchModule() {

    }

    virtual std::vector<std::array<int,2>> matchCorners(
            Multidim::Array<T,3, Multidim::ConstView> const& img1,
            std::vector<std::array<float, 2>> const& corners1,
            Multidim::Array<T,3, Multidim::ConstView> const& img2,
            std::vector<std::array<float, 2>> const& corners2) = 0;
};

template<typename T>
class HungarianCornerMatchModule : public GenericCornerMatchModule<T> {
public:

    static constexpr int batchDim = 2;

    HungarianCornerMatchModule(int patchRadius, int nSamples) :
        _patchRadius(patchRadius),
        _nSamples(nSamples)
    {

    }

    static float computeMatchingCost(Multidim::Array<T,3, Multidim::ConstView> const& img1,
                                     std::array<float, 2> const& corner1,
                                     Multidim::Array<T,3, Multidim::ConstView> const& img2,
                                     std::array<float, 2> const& corner2) {

        std::array<int, 2> integralCorner1{static_cast<int>(std::round(corner1[0])),static_cast<int>(std::round(corner1[1]))};
        std::array<int, 2> integralCorner2{static_cast<int>(std::round(corner2[0])),static_cast<int>(std::round(corner2[1]))};

        std::array<float, 2> dummy{0,0};

        std::vector<StereoVision::SparseMatching::orientedCoordinate<2>> oriented1 =
            {StereoVision::SparseMatching::orientedCoordinate<2>{integralCorner1,dummy}};
        std::vector<StereoVision::SparseMatching::orientedCoordinate<2>> oriented2 =
            {StereoVision::SparseMatching::orientedCoordinate<2>{integralCorner2,dummy}};

        using FFTDescr = StereoVision::SparseMatching::CircularFFTFeatureInfos<8,16,32>;

        std::vector<float> radiuses= {2,4,8};

        auto features1 = StereoVision::SparseMatching::CircularFFTAmplitudeDescriptors<FFTDescr>(oriented1, img1, radiuses, batchDim);
        auto features2 = StereoVision::SparseMatching::CircularFFTAmplitudeDescriptors<FFTDescr>(oriented2, img2, radiuses, batchDim);

        return StereoVision::Correlation::SumAbsDiff(features1[0].features, features2[0].features);
    }

    virtual std::vector<std::array<int,2>> matchCorners(
        Multidim::Array<T,3, Multidim::ConstView> const& img1,
        std::vector<std::array<float, 2>> const& corners1,
        Multidim::Array<T,3, Multidim::ConstView> const& img2,
        std::vector<std::array<float, 2>> const& corners2) override {



        std::vector<std::array<int, 2>> integral_corners1(corners1.size());
        std::vector<std::array<int, 2>> integral_corners2(corners2.size());

        for (size_t i = 0; i < corners1.size(); i++) {
            std::array<float, 2> pt = corners1[i];
            integral_corners1[i] = std::array<int, 2>{static_cast<int>(std::round(pt[0])), static_cast<int>(std::round(pt[1]))};
        }

        for (size_t i = 0; i < corners2.size(); i++) {
            std::array<float, 2> pt = corners2[i];
            integral_corners2[i] = std::array<int, 2>{static_cast<int>(std::round(pt[0])), static_cast<int>(std::round(pt[1]))};
        }

        std::vector<StereoVision::SparseMatching::orientedCoordinate<2>> oriented1 =
            StereoVision::SparseMatching::intensityOrientedCoordinates<true>(integral_corners1, img1, _patchRadius, batchDim);
        std::vector<StereoVision::SparseMatching::orientedCoordinate<2>> oriented2 =
            StereoVision::SparseMatching::intensityOrientedCoordinates<true>(integral_corners2, img2, _patchRadius, batchDim);

        constexpr int nDim = 3;

        std::array<int,nDim> shape = img1.shape();

        /*auto patchCoords = StereoVision::SparseMatching::generateUniformRadialSampleCoordinatesWithFeaturesDim<nDim,batchDim>(_patchRadius, _nSamples, shape[batchDim]);

        auto features1 = StereoVision::SparseMatching::OrientedWhitenedPixelsDescriptor(oriented1, img1, patchCoords, batchDim);
        auto features2 = StereoVision::SparseMatching::OrientedWhitenedPixelsDescriptor(oriented2, img2, patchCoords, batchDim);*/

        using FFTDescr = StereoVision::SparseMatching::CircularFFTFeatureInfos<8,16,32>;

        std::vector<float> radiuses= {2,4,8};

        auto features1 = StereoVision::SparseMatching::CircularFFTAmplitudeDescriptors<FFTDescr>(oriented1, img1, radiuses, batchDim);
        auto features2 = StereoVision::SparseMatching::CircularFFTAmplitudeDescriptors<FFTDescr>(oriented2, img2, radiuses, batchDim);

        int n = features1.size();
        int m = features2.size();

        Eigen::MatrixXf costs;
        costs.resize(n, m);

        std::vector<float> bestCosts(n);

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                costs(i,j) = StereoVision::Correlation::SumAbsDiff(features1[i].features, features2[j].features);
                if (j == 0) {
                    bestCosts[i] = costs(i,j);
                } else {
                    bestCosts[i] = std::min(bestCosts[i],costs(i,j));
                }
            }
        }

        auto targetIt = bestCosts.begin();
        std::advance(targetIt, bestCosts.size()/2);
        std::nth_element(bestCosts.begin(), targetIt, bestCosts.end());

        float maxNonAssignementCost = *targetIt;
        maxNonAssignementCost *= 1.5;

        Eigen::MatrixXf extendedCosts = StereoVision::Optimization::extendCostForNBestCosts(costs,3);

        extendedCosts = StereoVision::Optimization::setConstantMaxNonAssignementCost(extendedCosts, maxNonAssignementCost);

        std::vector<std::array<int,2>> assignements = StereoVision::Optimization::optimalAssignement(extendedCosts);

        std::vector<std::array<int,2>> ret;
        ret.reserve(assignements.size());

        for (std::array<int,2> const& proposal : assignements) {

            if (proposal[0] < 0 or proposal[0] >= corners1.size()) {
                continue;
            }

            if (proposal[1] < 0 or proposal[1] >= corners2.size()) {
                continue;
            }

            ret.push_back(proposal);
        }

        std::sort(ret.begin(), ret.end(), [&costs] (std::array<int,2> const& match1, std::array<int,2> const& match2) {
            return costs(match1[0],match1[1]) < costs(match2[0],match2[1]);
        });

        return ret;
    }

    void setPatchRadius(int radius) {
        _patchRadius = radius;
    }

    void setNSamples(int n) {
        _nSamples = n;
    }

protected:

    int _patchRadius;
    int _nSamples;
};

template<typename T>
class GenericTiePointsRenfinementModule {
public:

    virtual ~GenericTiePointsRenfinementModule() {

    }
    virtual bool refineMatchedPositions(std::vector<std::array<int,2>> const& assignement,
                                Multidim::Array<T,3, Multidim::ConstView> const& imgData1,
                                std::vector<std::array<float, 2>> & corners1,
                                Multidim::Array<T,3, Multidim::ConstView> const& imgData2,
                                std::vector<std::array<float, 2>> & corners2) = 0;

};

template<typename T>
class GenericInlinerSelectionModule {
public:

    virtual ~GenericInlinerSelectionModule() {

    }

    virtual std::vector<int> getInliers(std::vector<std::array<int,2>> const& assignement,
                                Multidim::Array<T,3, Multidim::ConstView> const& imgData1,
                                std::vector<std::array<float, 2>> & corners1,
                                Multidim::Array<T,3, Multidim::ConstView> const& imgData2,
                                std::vector<std::array<float, 2>> & corners2) = 0;
};

template<typename T>
class RansacEpipolarInlinerSelectionModule : public GenericInlinerSelectionModule<T> {
public:

    RansacEpipolarInlinerSelectionModule(int nRansacIterations, float threshold = 0.02) :
        _nIterations(nRansacIterations),
        _threshold(threshold)
    {

    }

    std::vector<int> getInliers(std::vector<std::array<int,2>> const& assignement,
                                Multidim::Array<T,3, Multidim::ConstView> const& imgData1,
                                std::vector<std::array<float, 2>> & corners1,
                                Multidim::Array<T,3, Multidim::ConstView> const& imgData2,
                                std::vector<std::array<float, 2>> & corners2) {

        int s1 = std::min(imgData1.shape()[0], imgData1.shape()[1]);
        int s2 = std::min(imgData2.shape()[0], imgData2.shape()[1]);

        if (assignement.size() < 8) { //too small to run ransac
            std::vector<int> ret(assignement.size());
            for (int i = 0; i < ret.size(); i++) {
                ret[i] = i;
            }

            return ret;
        }

        using Measure = std::array<float,4>; //points pair [x1, y1, x2, y2];

        struct Model {
            Model() {
                FundamentalMatrix = Eigen::Matrix3f::Identity();
            }
            Model(std::vector<Measure> const& observations) {

                using PointsArrayT = Eigen::Array<float, 2, Eigen::Dynamic>;

                PointsArrayT pts1;
                PointsArrayT pts2;

                pts1.resize(2, observations.size());
                pts2.resize(2, observations.size());

                for (int i = 0; i < observations.size(); i++) {

                    Measure const& observation = observations[i];

                    float const& x1 = observation[0];
                    float const& y1 = observation[1];

                    float const& x2 = observation[2];
                    float const& y2 = observation[3];

                    pts1(0,i) = x1;
                    pts1(1,i) = y1;

                    pts2(0,i) = x2;
                    pts2(1,i) = y2;

                }

                FundamentalMatrix = StereoVision::Geometry::estimateEssentialMatrix(pts1, pts2);

            }

            float error(Measure const& observation) {

                float const& x1 = observation[0];
                float const& y1 = observation[1];

                float const& x2 = observation[2];
                float const& y2 = observation[3];

                Eigen::Vector3f line = FundamentalMatrix*Eigen::Vector3f(x2,y2,1);
                float res = std::fabs(Eigen::Vector3f(x1,y1,1).dot(line))/std::sqrt(line.x()*line.x() + line.y()*line.y()); //distance to the epipolar line
                return std::fabs(res);
            }

            Eigen::Matrix3f FundamentalMatrix;
        };

        std::vector<Measure> observations(assignement.size());

        for (int i = 0; i < assignement.size(); i++) {

            int corner1Id = assignement[i][0];
            int corner2Id = assignement[i][1];

            observations[i][0] = corners1[corner1Id][0]/s1;
            observations[i][1] = corners1[corner1Id][1]/s1;

            observations[i][2] = corners2[corner2Id][0]/s2;
            observations[i][3] = corners2[corner2Id][1]/s2;
        }

        constexpr int minObs = 8;

        StereoVision::Optimization::GenericRansac<Measure, Model> ransac(observations, minObs, _threshold);

        ransac.ransacIterations(_nIterations);

        std::vector<int> ret;
        ret.reserve(corners1.size());

        for (int i = 0; i < corners1.size(); i++) {
            if (ransac.currentInliers()[i]) {
                ret.push_back(i);
            }
        }

        return ret;

    }

    void setNIterations(int nIterations) {
        _nIterations = nIterations;
    }

    void setThreshold(float threshold) {
        _threshold = threshold;
    }

protected:

    int _nIterations;
    float _threshold;
};

template<typename T>
class RansacPerspectiveInlinerSelectionModule : public GenericInlinerSelectionModule<T> {
public:

    RansacPerspectiveInlinerSelectionModule(int nRansacIterations, float threshold = 10) :
        _nIterations(nRansacIterations),
        _threshold(threshold)
    {

    }

    std::vector<int> getInliers(std::vector<std::array<int,2>> const& assignement,
                                Multidim::Array<T,3, Multidim::ConstView> const& imgData1,
                                std::vector<std::array<float, 2>> & corners1,
                                Multidim::Array<T,3, Multidim::ConstView> const& imgData2,
                                std::vector<std::array<float, 2>> & corners2) {

        if (assignement.size() < 4) { //too small to run ransac
            std::vector<int> ret(assignement.size());
            for (int i = 0; i < ret.size(); i++) {
                ret[i] = i;
            }

            return ret;
        }

        using Measure = std::array<float,4>; //points pair [x1, y1, x2, y2];

        struct Model {
            Model() {
                PerspectiveWarpMatrix = Eigen::Matrix3f::Identity();
            }
            Model(std::vector<Measure> const& observations) {

                using PointsArrayT = Eigen::Array<float, 2, Eigen::Dynamic>;

                PointsArrayT pts1;
                PointsArrayT pts2;

                pts1.resize(2, observations.size());
                pts2.resize(2, observations.size());

                for (int i = 0; i < observations.size(); i++) {

                    Measure const& observation = observations[i];

                    float const& x1 = observation[0];
                    float const& y1 = observation[1];

                    float const& x2 = observation[2];
                    float const& y2 = observation[3];

                    pts1(0,i) = x1;
                    pts1(1,i) = y1;

                    pts2(0,i) = x2;
                    pts2(1,i) = y2;

                }

                PerspectiveWarpMatrix = StereoVision::Geometry::estimatePerspectiveTransformMatrix(pts1, pts2);

            }

            float error(Measure const& observation) {

                float const& x1 = observation[0];
                float const& y1 = observation[1];

                float const& x2 = observation[2];
                float const& y2 = observation[3];

                Eigen::Vector3f reprojected = PerspectiveWarpMatrix*Eigen::Vector3f(x1,y1,1);
                float dx = reprojected.x()/reprojected.z() - x2;
                float dy = reprojected.y()/reprojected.z() - y2;
                float res = std::sqrt(dx*dx + dy*dy); //distance to the original point
                return res;
            }

            Eigen::Matrix3f PerspectiveWarpMatrix;
        };

        std::vector<Measure> observations(assignement.size());

        for (int i = 0; i < assignement.size(); i++) {

            int corner1Id = assignement[i][0];
            int corner2Id = assignement[i][1];

            observations[i][0] = corners1[corner1Id][0];
            observations[i][1] = corners1[corner1Id][1];

            observations[i][2] = corners2[corner2Id][0];
            observations[i][3] = corners2[corner2Id][1];
        }

        using Sampler = StereoVision::Optimization::FullyPrioritizedRansacSamplingStrategy;

        constexpr int minObs = 4;

        std::vector<int> order(observations.size());

        for (int i = 0; i < observations.size(); i++) {
            order[i] = i; //samples are assumed to be sorted from smallest cost to highest cost by the matching module
        }

        StereoVision::Optimization::GenericRansac<Measure, Model, Sampler> ransac(observations, Sampler(order), minObs, _threshold);

        ransac.ransacIterations(_nIterations);

        std::vector<int> ret;
        ret.reserve(assignement.size());

        for (int i = 0; i < assignement.size(); i++) {
            if (ransac.currentInliers()[i]) {
                ret.push_back(i);
            }
        }

        return ret;

    }

    void setNIterations(int nIterations) {
        _nIterations = nIterations;
    }

    void setThreshold(float threshold) {
        _threshold = threshold;
    }

protected:

    int _nIterations;
    float _threshold;
};

} // namespace StereoVisionApp

#endif // SPARSEMATCHINGPIPELINE_H
