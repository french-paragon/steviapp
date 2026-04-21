#ifndef TYPES_INFOS_H
#define TYPES_INFOS_H

#include <array>
#include <type_traits>

#include <type_traits>
namespace StereoVisionApp {

template<typename Indexable>
struct IndexableInfos {

private:
    struct InternalFunctor {
        auto operator()(Indexable & indexable, int idx) {
            return indexable[idx];
        }
    };

public:
    using ScalarT = std::remove_reference_t<typename std::invoke_result<InternalFunctor, Indexable &, int>::type>;
};

static_assert(std::is_same_v<IndexableInfos<std::array<double,3>>::ScalarT, double>);

} //namespace StereoVisionApp

#endif // TYPES_INFOS_H
