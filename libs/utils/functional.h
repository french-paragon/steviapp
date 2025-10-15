#ifndef FUNCTIONAL_H
#define FUNCTIONAL_H

#include <tuple>
#include <cmath>

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
 * Using this class, it is possible to modify arguments parameters packs to creat function modificator on the fly very easily.
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
            return std::tuple_cat(std::tuple<decltype (std::get<start>(initial))>(std::get<start>(initial)),
                                  TupleSlice<start+1,std::max(start+1,end),TupleT>::val(initial)); //using std::max(start+1,end) ensure to kill unwanted recursion
        }
    };
    template <int id, typename TupleT>
    struct TupleSlice<id,id,TupleT> {
        static inline auto val(TupleT const& initial) {
            return std::tuple<decltype (std::get<id>(initial))>(std::get<id>(initial));
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

}

#endif // FUNCTIONAL_H
