#ifndef FUNCTIONAL_H
#define FUNCTIONAL_H

#include <tuple>
#include <cmath>
#include <vector>
#include <ceres/autodiff_cost_function.h>
#include <ceres/dynamic_autodiff_cost_function.h>

namespace ceres {
class CostFunction;
}

namespace StereoVisionApp {

template<bool cond>
struct ConditionalRef {
    template<typename Ttrue, typename Tfalse>
    static Ttrue& val(Ttrue & t, Tfalse & f) { (void) f; return t; }
};

template<>
struct ConditionalRef<false> {
    template<typename Ttrue, typename Tfalse>
    static Tfalse& val(Ttrue & t, Tfalse & f) { (void) t; return f; }
};

/*!
 * \brief The CallFromTuple class contains a static call method that allows to call a functor from a tuple of arguments
 *
 * Using this class, it is possible to modify arguments parameters packs to create function decorators on the fly very easily.
 */
class CallFromTuple {

public:
    template <typename FunctorT, typename TupleT>
    static inline auto call (FunctorT const& functor, TupleT& args) {

        constexpr int nParams = std::tuple_size_v<TupleT>;

        using SubCallerT = std::conditional_t<nParams == 0, DirectCaller, RecursiveCaller>;

        return SubCallerT::callImpl(functor, args);

    }

    template <int idx, typename TupleT>
    static inline auto removeArgFromTuple(TupleT const& tuple) {
        constexpr int tupleLen = std::tuple_size_v<TupleT>;
        static_assert (tupleLen > 0, "Cannot process empty tuple type");
        static_assert (idx >= 0 and idx < tupleLen, "invalid Idx");

        std::tuple<> emptyTuple;
        auto beforeTuple = TupleSlice<0,std::max(0,idx-1),TupleT>::val(tuple);
        auto afterTuple = TupleSlice<std::min(idx+1,tupleLen-1),tupleLen-1,TupleT>::val(tuple);

        constexpr bool beforeShouldBeEmpty = idx-1 < 0;
        constexpr bool afterShouldBeEmpty = idx+1 > tupleLen-1;

        return std::tuple_cat(ConditionalRef<beforeShouldBeEmpty>::val(emptyTuple,beforeTuple),ConditionalRef<afterShouldBeEmpty>::val(emptyTuple,afterTuple));
    }

    template <int idx, typename ArgT, typename TupleT>
    static inline auto insertArgInTuple(TupleT const& tuple, ArgT const& val) {

        constexpr int tupleLen = std::tuple_size_v<TupleT>;
        static_assert (idx >= 0 and idx <= tupleLen, "invalid Idx");

        std::tuple<> emptyTuple;
        auto beforeTuple = TupleSlice<0,std::max(0,idx-1),TupleT>::val(tuple);
        auto afterTuple = TupleSlice<std::min(idx,tupleLen-1),tupleLen-1,TupleT>::val(tuple);

        constexpr bool beforeShouldBeEmpty = idx-1 < 0;
        constexpr bool afterShouldBeEmpty = idx > tupleLen-1;

        return std::tuple_cat(ConditionalRef<beforeShouldBeEmpty>::val(emptyTuple,beforeTuple),
                              std::tuple<ArgT>(val),
                              ConditionalRef<afterShouldBeEmpty>::val(emptyTuple,afterTuple));
    }

protected:
    template <int start, int end, typename TupleT>
    struct TupleSlice {
        static inline auto val(TupleT const& initial) {
            return std::tuple_cat(std::tuple<std::tuple_element_t<start, TupleT>>(std::get<start>(initial)),
                                  TupleSlice<start+1,std::max(start+1,end),TupleT>::val(initial)); //using std::max(start+1,end) ensure to kill unwanted recursion
        }
    };
    template <int id, typename TupleT>
    struct TupleSlice<id,id,TupleT> {
        static inline auto val(TupleT const& initial) {
            return std::tuple<std::tuple_element_t<id, TupleT>>(std::get<id>(initial));
        }
    };

    struct DirectCaller {
        template <typename FunctorT, typename TupleT, typename ... ArgsT>
        static inline auto callImpl (FunctorT const& functor, TupleT& unusedTuple, ArgsT& ... args) {
            (void) unusedTuple;
            return functor(args...);
        }
    };

    struct RecursiveCaller {
        template <typename FunctorT, typename TupleT, typename ... ArgsT>
        static inline auto callImpl (FunctorT const& functor, TupleT& args, ArgsT& ... expandedArgs) {

            constexpr int tupleSize = std::tuple_size_v<TupleT>;
            constexpr int paramPackSize = sizeof... (expandedArgs);

            constexpr int paramId = tupleSize - paramPackSize - 1;

            using SubCallerT = std::conditional_t<tupleSize <= paramPackSize+1, DirectCaller, RecursiveCaller>;

            return SubCallerT::template callImpl<FunctorT,TupleT, typename std::tuple_element<paramId, TupleT>::type&>(functor,
                                                                  args,
                                                                  std::get<paramId>(args),
                                                                  expandedArgs...);

        }
    };

};

template<typename T, int n>
class ArrayToTuple {
    static_assert(n>=1);
public:

    static auto convert(std::array<T,n> const& array) {
        std::array<T,n-1> subArray;
        for (int i = 0; i < n-1; i++) {
            subArray[i] = array[i+1];
        }
        return std::tuple_cat(std::tuple<T>(array[0]), ArrayToTuple<T,n-1>::convert(subArray));
    }
};
template<typename T>
class ArrayToTuple<T,1> {
public:

    static auto convert(std::array<T,1> const& array) {
        return std::tuple<T>(array[0]);
    }
};

template <typename T>
using IdentityDecorator = T;

/*!
 * \brief The BaseDynamicDecorator class is used as a base to decorate functors for DynamicAutoDiffCostFunction
 */
class BaseDynamicDecorator {

public:

    inline BaseDynamicDecorator() :
        _nParams(0)
    {

    }

    inline int nParams() const {
        return _nParams;
    }

    inline void setNParams(int nParams) {
        _nParams = nParams;
    }

protected:

    inline void setNParams(int nParams) const { //allow to set the number of parameters in functors in const methods
        _nParams = nParams;
    }

private:

    mutable int _nParams;

};

/*!
 * \brief The CollapseArgs class use one input argument to fill two inputs from an underlying functor
 *
 * Ceres require that each input argument appear only once in the cost function,
 * but sometimes the same argument can be used for multiple inputs of a functor.
 *
 * This decorator allow to build a functor taking as input a single argument that is then split between the functors
 */
template <typename FunctorT, std::size_t arg0id, std::size_t arg1id>
class CollapseArgs : private FunctorT
{
    static_assert(arg0id < arg1id, "invalid arguments positions provided");
public:

    template <typename ... P>
    CollapseArgs(P... args) :
        FunctorT(args...)
    {

    }

    template <typename ... P>
    bool operator()(P ... params) const {

        std::tuple<P...> args(params...);

        using T = std::remove_const_t<
            std::remove_pointer_t<
                std::remove_reference_t<decltype (std::get<0>(args))>
                >>;

        // insert new argument, copying the other one.
        auto processedArgs =
            CallFromTuple::insertArgInTuple<arg1id, const T*>(
                args, std::get<arg0id>(args));

        auto variadic_lambda = [this] (auto... params) {
            (void) this; //remove useless warning.
            return FunctorT::operator()(params...);
        };

        return CallFromTuple::call(variadic_lambda, processedArgs);
    }
};

struct CostFunctionData {
    ceres::CostFunction* costFunction;
    std::vector<double*> params;
};

template<typename DynamicFunctorT>
class CollapseArgsDynamic : private DynamicFunctorT, public virtual BaseDynamicDecorator {

public:
    template <typename ... P>
    CollapseArgsDynamic(std::vector<int> const& paramMapping, P... args) :
        DynamicFunctorT(args...),
        _mapping(paramMapping)
    {
        int nParam = 0;

        for (int paramId : paramMapping) {
            nParam = std::max(paramId+1, nParam);
        }

        setNParams(nParam);
    }

    template <typename T>
    bool operator()(T const* const* parameters, T* residuals) const {

        std::vector<T const*> mapped(_mapping.size());

        for (int i = 0; i < _mapping.size(); i++) {
            mapped[i] = parameters[_mapping[i]];
        }

        int oldNParams = nParams();
        setNParams(mapped.size()); //ensure the nParams is configured to the number expected by the subfunctor
        bool status = DynamicFunctorT::operator()(mapped.data(), residuals);
        setNParams(oldNParams);
        return status;

    }

protected:
    std::vector<int> _mapping;
};

template<typename FunctorT, int... argsSize>
class CollapseArgsStatic2Dynamic : private FunctorT, public virtual BaseDynamicDecorator {

protected:



public:
    template <typename ... P>
    CollapseArgsStatic2Dynamic(std::array<int,sizeof...(argsSize)> const& paramMapping, P... args) :
        FunctorT(args...),
        _mapping(paramMapping)
    {
        int nParam = 0;

        for (int paramId : paramMapping) {
            nParam = std::max(paramId+1, nParam);
        }

        setNParams(nParam);
    }

    template <typename T>
    bool operator()(T const* const* parameters, T* residuals) const {

        std::array<T const*,sizeof...(argsSize)> mapped;

        for (int i = 0; i < _mapping.size(); i++) {
            mapped[i] = parameters[_mapping[i]];
        }

        auto processedArgs = std::tuple_cat(ArrayToTuple<T const*,sizeof...(argsSize)>::convert(mapped),
                                            std::tuple<T*>(residuals));

        auto variadic_lambda = [this] (auto... params) {
            (void) this;
            return FunctorT::operator()(params...);
        };

        return CallFromTuple::call(variadic_lambda, processedArgs);
    }

protected:
    std::array<int,sizeof...(argsSize)> _mapping;
};

template<typename FunctorT, int nRes, int ... argsSizes>
class ParamsCollapseHelper {
public:

protected:

    static constexpr int nArgs = 1 + sizeof...(argsSizes);

    template<typename F_T, int ... argsSizesHead>
    struct CollapserHelperHead {
        template<int currentArgId, int targetArgId, int targetSize, int ... argsSizesTail>
        struct CollapserHelperTail {

            template <typename ... T>
            static CostFunctionData buildCollapsedArgsCostFunctionImpl(std::vector<double*> & parameters, T ... constructorArgs) {

                static_assert((sizeof...(argsSizesHead) <= targetArgId));

                if constexpr (sizeof...(argsSizesHead) < targetArgId) {
                    //re-iterate until after currentArgId
                    using NextCollapserHead = CollapserHelperHead<F_T, argsSizesHead..., targetSize>;
                    using NextCollapserTail = typename NextCollapserHead::template CollapserHelperTail<currentArgId, targetArgId, argsSizesTail...>;

                    return NextCollapserTail::buildCollapsedArgsCostFunctionImpl(parameters, constructorArgs...);

                } else {

                    if (parameters[currentArgId] == parameters[targetArgId]) {

                        using DecoratedFunctor = CollapseArgs<F_T, currentArgId, targetArgId>;
                        parameters.erase(parameters.begin() + targetArgId); //remove argument

                        if constexpr (currentArgId == targetArgId-1 and sizeof...(argsSizesTail) == 0) { //end of chain
                            using CostFunc = ceres::AutoDiffCostFunction<DecoratedFunctor,nRes,argsSizesHead...>;
                            return CostFunctionData{new CostFunc(new DecoratedFunctor(constructorArgs...)), parameters};
                        } else if constexpr (sizeof...(argsSizesTail) == 0) { //still need to check next argument for duplicates

                            if constexpr (currentArgId+2 >= sizeof...(argsSizesHead)) {//after removing the argument, no need to iterate more
                                using CostFunc = ceres::AutoDiffCostFunction<DecoratedFunctor,nRes,argsSizesHead...>;
                                return CostFunctionData{new CostFunc(new DecoratedFunctor(constructorArgs...)), parameters};
                            } else {
                                using NextCollapserHead = CollapserHelperHead<DecoratedFunctor>;
                                using NextCollapserTail =
                                    typename NextCollapserHead::template CollapserHelperTail<currentArgId+1, currentArgId+2, argsSizesHead...>;

                                return NextCollapserTail::buildCollapsedArgsCostFunctionImpl(parameters, constructorArgs...);
                            }
                        } else {
                            return CollapserHelperHead<DecoratedFunctor, argsSizesHead...>:: template
                                CollapserHelperTail<currentArgId, targetArgId, argsSizesTail...>::
                                buildCollapsedArgsCostFunctionImpl(parameters, constructorArgs...);
                        }
                    }

                    //no need to remove the last argument
                    if constexpr (currentArgId == targetArgId-1 and sizeof...(argsSizesTail) == 0) { //end of chain
                        using CostFunc = ceres::AutoDiffCostFunction<F_T,nRes,argsSizesHead..., targetSize>;
                        return CostFunctionData{new CostFunc(new F_T(constructorArgs...)), parameters};
                    } else if constexpr (sizeof...(argsSizesTail) == 0) { //still need to check next argument for duplicates

                        using NextCollapserHead = CollapserHelperHead<F_T>;
                        using NextCollapserTail =
                            typename NextCollapserHead::template CollapserHelperTail<currentArgId+1, currentArgId+2, argsSizesHead..., targetSize>;

                        return NextCollapserTail::buildCollapsedArgsCostFunctionImpl(parameters, constructorArgs...);

                    } else {

                        using NextCollapserHead = CollapserHelperHead<F_T, argsSizesHead..., targetSize>;
                        using NextCollapserTail = typename NextCollapserHead::template CollapserHelperTail<currentArgId, targetArgId+1, argsSizesTail...>;

                        return NextCollapserTail::buildCollapsedArgsCostFunctionImpl(parameters, constructorArgs...);
                    }
                }
            }
        };
    };

public:

    template <typename ... T>
    static CostFunctionData buildCollapsedArgsCostFunction(std::vector<double*> const& parameters, T ... constructorArgs) {

        constexpr int nArgs = sizeof...(argsSizes);

        std::array<int,nArgs> mapping;
        std::fill(mapping.begin(), mapping.end(), -1);
        bool duplicateDetected = false;

        std::array<int,nArgs> parametersSizes = {argsSizes...};

        std::vector<double*> filteredParameters;
        std::vector<int> filteredParametersSizes;
        filteredParameters.reserve(nArgs);
        filteredParametersSizes.reserve(nArgs);

        for (size_t i = 0; i < nArgs; i++) {
            if (mapping[i] >= 0) {
                continue; //parameters has been remapped already
            }
            mapping[i] = filteredParameters.size();
            filteredParameters.push_back(parameters[i]);
            filteredParametersSizes.push_back(parametersSizes[i]);

            for (int j = i+1; j < nArgs; j++) {
                if (parameters[j] == parameters[i]) {
                    duplicateDetected = true;
                    mapping[j] = mapping[i];
                }
            }
        }

        if (duplicateDetected) {
            constexpr int stride = 4;
            using DynFunctorT = CollapseArgsStatic2Dynamic<FunctorT, argsSizes...>;
            using CostFunc = ceres::DynamicAutoDiffCostFunction<DynFunctorT,stride>;

            CostFunc* costFunct = new CostFunc(new DynFunctorT(mapping, constructorArgs...));

            for (size_t i = 0; i < filteredParametersSizes.size(); i++) {
                costFunct->AddParameterBlock(filteredParametersSizes[i]);
            }

            costFunct->SetNumResiduals(nRes);

            return {costFunct, filteredParameters};
        }

        using CostFunc = ceres::AutoDiffCostFunction<FunctorT,nRes,argsSizes...>;
        return CostFunctionData{new CostFunc(new FunctorT(constructorArgs...)), parameters};
    }

    template <typename ... T>
    static CostFunctionData buildCollapsedArgsDynamicCostFunction(std::vector<double*> const& parameters,
                                                                  std::vector<int> const& dynamicParametersSizes,
                                                                  T ... constructorArgs) {


        int nArgs = parameters.size();

        std::vector<int> mapping(nArgs);
        std::fill(mapping.begin(), mapping.end(), -1);
        bool duplicateDetected = false;

        std::vector<double*> filteredParameters;
        std::vector<int> filteredParametersSizes;
        filteredParameters.reserve(nArgs);
        filteredParametersSizes.reserve(nArgs);

        for (size_t i = 0; i < nArgs; i++) {
            if (mapping[i] >= 0) {
                continue; //parameters has been remapped already
            }
            mapping[i] = filteredParameters.size();
            filteredParameters.push_back(parameters[i]);
            filteredParametersSizes.push_back(dynamicParametersSizes[i]);

            for (int j = i+1; j < nArgs; j++) {
                if (parameters[j] == parameters[i]) {
                    duplicateDetected = true;
                    mapping[j] = mapping[i];
                }
            }
        }

        constexpr int stride = 4;

        if (duplicateDetected) {
            using DynFunctorT = CollapseArgsDynamic<FunctorT>;
            using CostFunc = ceres::DynamicAutoDiffCostFunction<DynFunctorT,stride>;

            CostFunc* costFunct = new CostFunc(new DynFunctorT(mapping, constructorArgs...)); //CollapseArgsDynamic manage the nArguments for the functor

            for (size_t i = 0; i < filteredParametersSizes.size(); i++) {
                costFunct->AddParameterBlock(filteredParametersSizes[i]);
            }

            costFunct->SetNumResiduals(nRes);

            return {costFunct, filteredParameters};
        }

        using CostFunc = ceres::DynamicAutoDiffCostFunction<FunctorT,stride>;

        FunctorT* functor = new FunctorT(constructorArgs...);
        //need to indicate number of arguments to dynamic functor
        functor->setNParams(parameters.size());
        CostFunc* costFunct = new CostFunc(functor);

        for (size_t i = 0; i < dynamicParametersSizes.size(); i++) {
            costFunct->AddParameterBlock(dynamicParametersSizes[i]);
        }

        costFunct->SetNumResiduals(nRes);

        return {costFunct, parameters};
    }

};


}

#endif // FUNCTIONAL_H
