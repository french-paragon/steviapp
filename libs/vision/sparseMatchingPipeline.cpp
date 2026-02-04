#include "sparseMatchingPipeline.h"

namespace StereoVisionApp {

const char* HeadlessSparseMatchingPipelineInterface::INTERFACE_NAME = "StereoVisionAppHeadlessSparseMatchingPipelineInterface";

HeadlessSparseMatchingPipelineInterface::HeadlessSparseMatchingPipelineInterface(QObject* parent) :
    QObject(parent)
{
    _current_pipeline = static_cast<SparseMatchingPipeline<float>*>(nullptr);

    _img_1_holder = ImageDataHolder{"",Multidim::Array<float, 3>(),nullptr};
    _img_2_holder = ImageDataHolder{"",Multidim::Array<float, 3>(),nullptr};

    _nextFrame = &_img_1_holder;
}
HeadlessSparseMatchingPipelineInterface::~HeadlessSparseMatchingPipelineInterface() {

    std::visit([] (auto* ptr) {
        if (ptr != nullptr) {
            delete ptr;
        }
    }, _current_pipeline);
}

HeadlessSparseMatchingPipelineInterface::ImageDataHolder::~ImageDataHolder() {
    if (matchBuilder != nullptr) {
        delete matchBuilder;
    }
}

void HeadlessSparseMatchingPipelineInterface::configureImageData(QString const& name, Multidim::Array<float, 3> && data, Correspondences::UVMatchBuilder* matchBuilder) {

    ImageDataHolder& nextFrame = *_nextFrame;

    if (nextFrame.matchBuilder != nullptr) {
        delete nextFrame.matchBuilder;
    }

    nextFrame.name = name;
    nextFrame.imgData = data;
    nextFrame.matchBuilder = matchBuilder;

    if (_nextFrame == &_img_1_holder) {
        _nextFrame = &_img_2_holder;
    } else {
        _nextFrame = &_img_1_holder;
    }
}

StatusOptionalReturn<void> HeadlessSparseMatchingPipelineInterface::runAll(bool verbose) {
    return std::visit([verbose, this] (auto* ptr) {
        if (ptr == nullptr) {
            return StatusOptionalReturn<void>::error("Matching pipeline not set");
        }

        using ComputeT = typename std::remove_pointer_t<decltype(ptr)>::ComputeT;

        Multidim::Array<ComputeT,3> img1;
        Multidim::Array<ComputeT,3> img2;

        if constexpr (std::is_same_v<ComputeT, float>) {
            ptr->setupPipeline(_img_1_holder.imgData, _img_2_holder.imgData);
        } else {
            img1 = _img_1_holder.imgData.cast<ComputeT>();
            img2 = _img_2_holder.imgData.cast<ComputeT>();
            ptr->setupPipeline(img1, img2);
        }
        ptr->runAllSteps(verbose);

        return StatusOptionalReturn<void>();
    }, _current_pipeline);
}

} // namespace StereoVisionApp
